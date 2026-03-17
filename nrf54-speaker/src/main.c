/*
 * XIAO nRF54L15 Sense — IMU motion detection + I2S audio playback
 *
 * Plays embedded 8kHz/16-bit/mono PCM through a MAX98357A I2S amplifier
 * (HAT SPK2) whenever motion is detected.  BLE notify and OTA are
 * preserved from nrf54-motion.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "audio_data.h"

LOG_MODULE_REGISTER(nrf54_speaker, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu0)
#define LED0_NODE DT_ALIAS(led0)

/* ============================================================================
 * Motion detection parameters (unchanged from nrf54-motion)
 * ============================================================================ */

#define MOTION_THRESHOLD_MS2        1.5
#define MOTION_DURATION_SAMPLES     2
#define ACCEL_ODR_HZ                26
#define POLL_INTERVAL_MS            100
#define CALIBRATION_SAMPLES         25
#define ACTIVITY_WINDOW_SAMPLES     4
#define MOTION_ENTRY_ACTIVITY_MS2   8.0
#define MOTION_ENTRY_PEAK_MS2       2.4
#define MOTION_CONTINUE_ACTIVITY_MS2 4.0
#define MOTION_CONTINUE_PEAK_MS2    1.4
#define MOTION_SETTLE_ACTIVITY_MS2  0.9
#define MOTION_SETTLE_PEAK_MS2      0.35
#define MOTION_START_WINDOWS        2
#define MOTION_SETTLE_WINDOWS       4
#define BASELINE_ALPHA              0.03
#define REPORT_COOLDOWN_MS          700

/* ============================================================================
 * I2S / Audio parameters
 * ============================================================================ */

/*
 * 30 ms worth of samples at 8 kHz; each frame is stereo int16_t so the
 * DMA block holds BLOCK_SAMPLES * 2 channels * sizeof(int16_t) bytes.
 */
#define I2S_BLOCK_SAMPLES  256                     /* ~32ms @ 8kHz */
#define I2S_BLOCK_SIZE     (I2S_BLOCK_SAMPLES * sizeof(int16_t))
#define I2S_NUM_BLOCKS     6
#define PLAYBACK_OUTPUT_SAMPLE_RATE  32000U
#define PLAYBACK_TONE_FREQ_HZ       1000U
#define PLAYBACK_TONE_DURATION_MS   800U
#define PLAYBACK_TONE_AMPLITUDE     24000
#define PLAYBACK_TONE_SAMPLES \
    ((PLAYBACK_OUTPUT_SAMPLE_RATE * PLAYBACK_TONE_DURATION_MS) / 1000U)
#define PLAYBACK_UPSAMPLE_FACTOR \
    (PLAYBACK_OUTPUT_SAMPLE_RATE / AUDIO_SAMPLE_RATE)

K_MEM_SLAB_DEFINE(i2s_mem_slab, I2S_BLOCK_SIZE, I2S_NUM_BLOCKS, 4);

static K_SEM_DEFINE(audio_sem, 0, 1);
static atomic_t audio_playing;

#define I2S_NODE DT_NODELABEL(i2s20)

static inline size_t playback_sample_count(void)
{
    return PLAYBACK_TONE_SAMPLES + (audio_pcm_sample_count * PLAYBACK_UPSAMPLE_FACTOR);
}

static int16_t playback_sample_at(size_t index)
{
    if (index < PLAYBACK_TONE_SAMPLES) {
        uint32_t half_period = PLAYBACK_OUTPUT_SAMPLE_RATE / (PLAYBACK_TONE_FREQ_HZ * 2U);
        if (half_period == 0U) {
            return PLAYBACK_TONE_AMPLITUDE;
        }

        return ((index / half_period) & 1U) == 0U
               ? PLAYBACK_TONE_AMPLITUDE
               : -PLAYBACK_TONE_AMPLITUDE;
    }

    return audio_pcm_data[(index - PLAYBACK_TONE_SAMPLES) / PLAYBACK_UPSAMPLE_FACTOR];
}

/* ============================================================================
 * BLE UUIDs
 * ============================================================================ */

#define MOTION_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00

#define MOTION_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00

#define MOTION_UUID_BUILD_INFO_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00

/* Write 0x01 to this characteristic (or send 'p' over serial) to trigger playback */
#define MOTION_UUID_PLAY_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00

#define BLE_DEVICE_NAME "SpeakerBridge"

#define UART_NODE DT_NODELABEL(uart20)

/* ============================================================================
 * BLE Global State
 * ============================================================================ */

static const char build_timestamp[] = __DATE__ " " __TIME__;

static ssize_t read_build_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             build_timestamp, strlen(build_timestamp));
}

static struct bt_conn *current_conn;

static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("TX CCCD updated: %d", value);
}

/* BLE play command: write 0x01 to trigger audio playback */
static ssize_t write_play_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len, uint16_t offset,
                               uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len >= 1 && ((const uint8_t *)buf)[0] == 0x01) {
        LOG_INF("BLE play command received");
        k_sem_give(&audio_sem);
    }
    return len;
}

BT_GATT_SERVICE_DEFINE(motion_svc,
    BT_GATT_PRIMARY_SERVICE(
        BT_UUID_DECLARE_128(MOTION_UUID_SERVICE)),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(MOTION_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        NULL, NULL, NULL),
    BT_GATT_CCC(tx_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(MOTION_UUID_BUILD_INFO_CHAR),
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_build_info, NULL, NULL),

    /* Write 0x01 here to trigger audio playback */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(MOTION_UUID_PLAY_CHAR),
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, write_play_cmd, NULL),
);

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE connection failed: %d", err);
        return;
    }
    LOG_INF("BLE connected");
    current_conn = conn;
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected: %d", reason);
    current_conn = NULL;
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = ble_connected,
    .disconnected = ble_disconnected,
};

static void notify_motion_event(uint8_t count, double activity, double peak,
                                uint32_t elapsed_time_ms)
{
    if (!current_conn) {
        return;
    }

    uint8_t pkt[14];
    float act_f  = (float)activity;
    float peak_f = (float)peak;

    pkt[0] = 0x01;
    pkt[1] = count;
    memcpy(&pkt[2], &act_f,  4);
    memcpy(&pkt[6], &peak_f, 4);
    memcpy(&pkt[10], &elapsed_time_ms, 4);

    bt_gatt_notify(NULL, &motion_svc.attrs[2], pkt, sizeof(pkt));
}

/* ============================================================================
 * Motion detection state (unchanged from nrf54-motion)
 * ============================================================================ */

static atomic_t detected_motion_count;
static uint32_t last_motion_time_ms;
static bool baseline_valid;
static double baseline_x;
static double baseline_y;
static double baseline_z;
static uint8_t calibration_count;
static uint8_t active_high_count;
static bool motion_active;
static uint8_t settle_count;
static int64_t last_report_ms;
static bool previous_sample_valid;
static double previous_accel_x;
static double previous_accel_y;
static double previous_accel_z;
static double step_window[ACTIVITY_WINDOW_SAMPLES];
static uint8_t step_window_index;
static uint8_t step_window_count;
static double step_window_sum;

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);

static int configure_led(void)
{
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
    if (!gpio_is_ready_dt(&led)) {
        LOG_WRN("LED device is not ready");
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
#else
    return 0;
#endif
}

static void pulse_led(void)
{
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
    gpio_pin_set_dt(&led, 1);
    k_msleep(80);
    gpio_pin_set_dt(&led, 0);
#endif
}

static double abs_double(double value)
{
    return value < 0.0 ? -value : value;
}

static void set_baseline(double accel_x, double accel_y, double accel_z)
{
    baseline_x = accel_x;
    baseline_y = accel_y;
    baseline_z = accel_z;
    baseline_valid = true;
}

static void update_baseline(double accel_x, double accel_y, double accel_z)
{
    if (!baseline_valid) {
        set_baseline(accel_x, accel_y, accel_z);
        return;
    }
    baseline_x += (accel_x - baseline_x) * BASELINE_ALPHA;
    baseline_y += (accel_y - baseline_y) * BASELINE_ALPHA;
    baseline_z += (accel_z - baseline_z) * BASELINE_ALPHA;
}

static double motion_delta(double accel_x, double accel_y, double accel_z)
{
    return abs_double(accel_x - baseline_x) +
           abs_double(accel_y - baseline_y) +
           abs_double(accel_z - baseline_z);
}

static double sample_delta(double accel_x, double accel_y, double accel_z)
{
    if (!previous_sample_valid) {
        previous_accel_x = accel_x;
        previous_accel_y = accel_y;
        previous_accel_z = accel_z;
        previous_sample_valid = true;
        return 0.0;
    }

    double delta = abs_double(accel_x - previous_accel_x) +
                   abs_double(accel_y - previous_accel_y) +
                   abs_double(accel_z - previous_accel_z);

    previous_accel_x = accel_x;
    previous_accel_y = accel_y;
    previous_accel_z = accel_z;
    return delta;
}

static void reset_step_window(void)
{
    for (size_t i = 0; i < ACTIVITY_WINDOW_SAMPLES; ++i) {
        step_window[i] = 0.0;
    }
    step_window_index = 0;
    step_window_count = 0;
    step_window_sum = 0.0;
}

static void update_step_window(double motion_step)
{
    if (step_window_count == ACTIVITY_WINDOW_SAMPLES) {
        step_window_sum -= step_window[step_window_index];
    } else {
        step_window_count++;
    }
    step_window[step_window_index] = motion_step;
    step_window_sum += motion_step;
    step_window_index = (step_window_index + 1U) % ACTIVITY_WINDOW_SAMPLES;
}

static double step_window_peak(void)
{
    double peak = 0.0;

    for (size_t i = 0; i < step_window_count; ++i) {
        if (step_window[i] > peak) {
            peak = step_window[i];
        }
    }
    return peak;
}

static void accumulate_calibration(double accel_x, double accel_y, double accel_z)
{
    if (calibration_count == 0) {
        set_baseline(accel_x, accel_y, accel_z);
    } else {
        baseline_x = ((baseline_x * calibration_count) + accel_x) / (calibration_count + 1);
        baseline_y = ((baseline_y * calibration_count) + accel_y) / (calibration_count + 1);
        baseline_z = ((baseline_z * calibration_count) + accel_z) / (calibration_count + 1);
    }

    calibration_count++;

    if (calibration_count == CALIBRATION_SAMPLES) {
        LOG_INF("Baseline ready: accel=(%.3f, %.3f, %.3f) m/s^2",
                baseline_x, baseline_y, baseline_z);
        previous_accel_x = baseline_x;
        previous_accel_y = baseline_y;
        previous_accel_z = baseline_z;
        previous_sample_valid = true;
        reset_step_window();
        last_report_ms = k_uptime_get();
        last_motion_time_ms = 0;
    }
}

static int configure_motion_detection(void)
{
    struct sensor_value odr = { .val1 = ACCEL_ODR_HZ, .val2 = 0 };
    struct sensor_value threshold = { .val1 = 1, .val2 = 500000 };
    struct sensor_value duration = { .val1 = MOTION_DURATION_SAMPLES, .val2 = 0 };
    int ret;

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) {
        LOG_ERR("Failed to set accelerometer ODR: %d", ret);
        return ret;
    }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SLOPE_TH, &threshold);
    if (ret == -ENOTSUP) {
        LOG_WRN("Motion threshold attribute is not supported; using sensor default");
    } else if (ret < 0) {
        LOG_ERR("Failed to set motion threshold: %d", ret);
        return ret;
    }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SLOPE_DUR, &duration);
    if (ret == -ENOTSUP) {
        LOG_WRN("Motion duration attribute is not supported; using sensor default");
    } else if (ret < 0) {
        LOG_ERR("Failed to set motion duration: %d", ret);
        return ret;
    }

    LOG_INF("Using software polling for motion detection");
    LOG_INF("Motion detection ready: ODR=%d Hz, poll=%d ms, entry_activity=%.2f peak=%.2f",
            ACCEL_ODR_HZ, POLL_INTERVAL_MS,
            MOTION_ENTRY_ACTIVITY_MS2, MOTION_ENTRY_PEAK_MS2);
    LOG_INF("Calibrating baseline for %.1f seconds; keep the board still",
            (double)(CALIBRATION_SAMPLES * POLL_INTERVAL_MS) / 1000.0);
    return 0;
}

/* ============================================================================
 * Audio playback thread
 * ============================================================================ */

static void audio_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *i2s_dev = DEVICE_DT_GET(I2S_NODE);

    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("I2S device not ready; audio playback disabled");
        return;
    }

    /* Configure I2S TX for mono output to the MAX98357A on SPK2. */
    struct i2s_config cfg = {
        .word_size    = 16,
        .channels     = 1,
        .format       = I2S_FMT_DATA_FORMAT_I2S,
        .options      = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = PLAYBACK_OUTPUT_SAMPLE_RATE,
        .mem_slab     = &i2s_mem_slab,
        .block_size   = I2S_BLOCK_SIZE,
        .timeout      = 2000,
    };

    int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
    LOG_INF("i2s_configure ret=%d", ret);
    if (ret < 0) {
        LOG_ERR("i2s_configure failed: %d", ret);
        return;
    }

    LOG_INF("Audio thread ready: %u samples @ %u Hz (%.2f s), block=%u bytes",
            (unsigned)playback_sample_count(),
            PLAYBACK_OUTPUT_SAMPLE_RATE,
            (double)playback_sample_count() / PLAYBACK_OUTPUT_SAMPLE_RATE,
            (unsigned)I2S_BLOCK_SIZE);

    while (1) {
        /* Wait for motion trigger */
        k_sem_take(&audio_sem, K_FOREVER);

        if (atomic_set(&audio_playing, 1) != 0) {
            /* Already playing — skip this trigger */
            continue;
        }

        LOG_INF("Audio playback start");

        const size_t total_samples = playback_sample_count();
        size_t sample_offset = 0;
        bool triggered = false;

        while (sample_offset < total_samples) {
            void *block;
            ret = k_mem_slab_alloc(&i2s_mem_slab, &block, K_MSEC(2000));
            if (ret < 0) {
                LOG_ERR("mem_slab_alloc failed: %d (free=%u/%u)",
                        ret,
                        k_mem_slab_num_free_get(&i2s_mem_slab),
                        I2S_NUM_BLOCKS);
                break;
            }

            int16_t *buf = (int16_t *)block;
            size_t frames = I2S_BLOCK_SAMPLES;
            size_t remaining = total_samples - sample_offset;

            if (remaining < frames) {
                frames = remaining;
                /* Zero-pad the rest of the block */
                memset(buf + frames, 0,
                       (I2S_BLOCK_SAMPLES - frames) * sizeof(int16_t));
            }

            /* Feed mono samples directly; SPK2/MAX98357A is a mono sink. */
            for (size_t i = 0; i < frames; ++i) {
                buf[i] = playback_sample_at(sample_offset + i);
            }

            sample_offset += frames;

            ret = i2s_write(i2s_dev, block, I2S_BLOCK_SIZE);
            if (ret < 0) {
                LOG_ERR("i2s_write failed: %d (offset=%u)", ret, (unsigned)sample_offset);
                k_mem_slab_free(&i2s_mem_slab, block);
                break;
            }

            if (!triggered) {
                ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
                LOG_INF("i2s_trigger START ret=%d", ret);
                if (ret < 0) {
                    LOG_ERR("i2s_trigger START failed: %d", ret);
                    break;
                }
                triggered = true;
            }
        }

        if (triggered) {
            ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
            LOG_INF("i2s_trigger DRAIN ret=%d", ret);
        }

        LOG_INF("Audio playback done");
        atomic_set(&audio_playing, 0);
    }
}

K_THREAD_DEFINE(audio_thread, 4096,
                audio_thread_fn, NULL, NULL, NULL,
                K_PRIO_PREEMPT(5), 0, 0);

/* ============================================================================
 * Motion sample processing
 * ============================================================================ */

static void process_motion_sample(void)
{
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;
    double accel_x_ms2;
    double accel_y_ms2;
    double accel_z_ms2;
    double delta;
    double motion_step;
    double activity;
    double peak_step;
    int64_t now_ms;
    int ret;

    ret = sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
    if (ret < 0) {
        LOG_ERR("Failed to fetch acceleration sample: %d", ret);
        return;
    }

    ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &accel_x);
    if (ret < 0) {
        LOG_ERR("Failed to read accel X: %d", ret);
        return;
    }

    ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &accel_y);
    if (ret < 0) {
        LOG_ERR("Failed to read accel Y: %d", ret);
        return;
    }

    ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &accel_z);
    if (ret < 0) {
        LOG_ERR("Failed to read accel Z: %d", ret);
        return;
    }

    accel_x_ms2 = sensor_value_to_double(&accel_x);
    accel_y_ms2 = sensor_value_to_double(&accel_y);
    accel_z_ms2 = sensor_value_to_double(&accel_z);

    if (calibration_count < CALIBRATION_SAMPLES) {
        accumulate_calibration(accel_x_ms2, accel_y_ms2, accel_z_ms2);
        return;
    }

    delta = motion_delta(accel_x_ms2, accel_y_ms2, accel_z_ms2);
    motion_step = sample_delta(accel_x_ms2, accel_y_ms2, accel_z_ms2);
    update_step_window(motion_step);
    activity = step_window_sum;
    peak_step = step_window_peak();
    now_ms = k_uptime_get();

    if (!motion_active) {
        if (activity >= MOTION_ENTRY_ACTIVITY_MS2 &&
            peak_step >= MOTION_ENTRY_PEAK_MS2) {
            active_high_count++;
        } else {
            active_high_count = 0;
            update_baseline(accel_x_ms2, accel_y_ms2, accel_z_ms2);
        }

         if (active_high_count >= MOTION_START_WINDOWS &&
             (now_ms - last_report_ms) >= REPORT_COOLDOWN_MS) {
             motion_active = true;
             settle_count = 0;
             active_high_count = 0;
            last_report_ms = now_ms;
            atomic_inc(&detected_motion_count);
            pulse_led();
            uint32_t elapsed_ms = (last_motion_time_ms == 0) ? 0 :
                                  (uint32_t)(now_ms - last_motion_time_ms);
             LOG_INF("Motion detected! count=%d elapsed=%u ms activity=%.3f peak=%.3f delta=%.3f step=%.3f accel=(%.3f, %.3f, %.3f) m/s^2",
                     (int)atomic_get(&detected_motion_count),
                     elapsed_ms,
                     activity,
                     peak_step,
                     delta,
                     motion_step,
                     accel_x_ms2, accel_y_ms2, accel_z_ms2);
             notify_motion_event((uint8_t)atomic_get(&detected_motion_count),
                                 activity, peak_step, elapsed_ms);
             if (atomic_get(&audio_playing) == 0) {
                 LOG_INF("Motion trigger -> audio playback");
                 k_sem_give(&audio_sem);
             }
             last_motion_time_ms = now_ms;
         }

        return;
    }

    if ((activity >= MOTION_CONTINUE_ACTIVITY_MS2 ||
         peak_step >= MOTION_CONTINUE_PEAK_MS2) &&
        (now_ms - last_report_ms) >= REPORT_COOLDOWN_MS) {
        last_report_ms = now_ms;
        atomic_inc(&detected_motion_count);
        pulse_led();
        uint32_t elapsed_ms = (last_motion_time_ms == 0) ? 0 :
                              (uint32_t)(now_ms - last_motion_time_ms);
        LOG_INF("Motion detected! count=%d elapsed=%u ms activity=%.3f peak=%.3f delta=%.3f step=%.3f accel=(%.3f, %.3f, %.3f) m/s^2",
                (int)atomic_get(&detected_motion_count),
                elapsed_ms,
                activity,
                peak_step,
                     delta,
                     motion_step,
                     accel_x_ms2, accel_y_ms2, accel_z_ms2);
             notify_motion_event((uint8_t)atomic_get(&detected_motion_count),
                                 activity, peak_step, elapsed_ms);
             last_motion_time_ms = now_ms;
         }

    if (activity <= MOTION_SETTLE_ACTIVITY_MS2 &&
        peak_step <= MOTION_SETTLE_PEAK_MS2) {
        settle_count++;
    } else {
        settle_count = 0;
    }

    if (settle_count >= MOTION_SETTLE_WINDOWS) {
        motion_active = false;
        settle_count = 0;
        set_baseline(accel_x_ms2, accel_y_ms2, accel_z_ms2);
        reset_step_window();
        LOG_INF("Motion settled: baseline updated to (%.3f, %.3f, %.3f) m/s^2",
                accel_x_ms2, accel_y_ms2, accel_z_ms2);
    }
}

/* ============================================================================
 * main
 * ============================================================================ */

int main(void)
{
    int ret;
    const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

    printk("*** nrf54-speaker boot ***\n");
    LOG_INF("XIAO nRF54L15 Sense speaker+motion OTA (build: %s)", build_timestamp);
    LOG_INF("Audio trigger: BLE write 0x01 to play char, or serial 'p'");

    if (!device_is_ready(imu)) {
        LOG_ERR("IMU device %s is not ready", imu->name);
        return -ENODEV;
    }

    ret = configure_led();
    if (ret < 0) {
        LOG_WRN("LED setup failed: %d", ret);
    }

    ret = configure_motion_detection();
    if (ret < 0) {
        return ret;
    }

    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("BLE enable failed: %d", ret);
        return ret;
    }

    bt_conn_cb_register(&conn_callbacks);

    ret = bt_set_name(BLE_DEVICE_NAME);
    if (ret) {
        LOG_ERR("Failed to set BLE name: %d", ret);
        return ret;
    }

    const struct bt_data adv_data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                      0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                      0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00),
    };

    const struct bt_data scan_rsp[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
    };

    ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data),
                          scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret) {
        LOG_ERR("BLE advertising failed: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising started: %s", BLE_DEVICE_NAME);
    LOG_INF("Ready — BLE: write 0x01 to play char | Serial: send 'p'");

    while (1) {
        k_sleep(K_MSEC(POLL_INTERVAL_MS));

        /* Serial 'p' command: trigger audio playback */
        unsigned char c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            if (c == 'p' || c == 'P') {
                LOG_INF("Serial play command received");
                k_sem_give(&audio_sem);
            }
        }

        process_motion_sample();
    }

    return 0;
}
