/*
 * nrf54-handy: XIAO nRF54L15 Sense → Handy BLE firmware
 *
 * Audio streaming via BLE with Handy-compatible protocol.
 * IMU gesture detection triggers BLE event packets to control Handy recording.
 * Single LED indicates device state.
 *
 * BLE Protocol (Handy-compatible):
 *   Audio Service:  00000001-0000-1000-8000-00805f9b34fb
 *     TX 0x0002 (NOTIFY): [seq][0xAA][PCM...]  — audio stream
 *                         [0x00][0x55][event]  — event packets
 *     RX 0x0003 (WRITE):  0x01=start, 0x00=stop
 *
 * LED states (active-low, Zephyr GPIO_ACTIVE_LOW flag handles inversion):
 *   Boot        ON  1s then off
 *   Advertising 500ms ON / 1500ms OFF blink
 *   Connected   ON  solid
 *   Recording   250ms fast blink
 *   Error       200ms fast blink
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
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/dfu/mcuboot.h>

#include <math.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(nrf54_handy, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME         "XIAOVoice"
#define PCM_PACKET_SIZE         200
#define WDT_NODE                DT_NODELABEL(wdt31)
#define WDT_TIMEOUT_MS          5000
#define MAIN_LOOP_INTERVAL_MS   100
#define ADV_BLINK_ON_MS         500
#define ADV_BLINK_OFF_MS        1500
#define RECORDING_BLINK_MS      250
#define ERROR_BLINK_MS          200

#define LED_NODE    DT_ALIAS(led0)

/* IMU / motion detection */
#define IMU_NODE                    DT_ALIAS(imu0)
#define ACCEL_ODR_HZ                416   /* 416 Hz for tap detection resolution */
#define MOTION_SAMPLE_INTERVAL_MS   25
#define CALIBRATION_SAMPLES         25
#define ACTIVITY_WINDOW_SAMPLES     4
#define MOTION_ENTRY_ACTIVITY_MS2   8.0
#define MOTION_ENTRY_PEAK_MS2       2.4
#define MOTION_CONTINUE_ACTIVITY_MS2 4.0
#define MOTION_CONTINUE_PEAK_MS2    1.4
#define MOTION_SETTLE_ACTIVITY_MS2  4.0
#define MOTION_SETTLE_PEAK_MS2      1.4
#define MOTION_START_WINDOWS        2
#define MOTION_SETTLE_WINDOWS       2
#define BASELINE_ALPHA              0.03
#define REPORT_COOLDOWN_MS          700
#define MOTION_DURATION_SAMPLES     2

/* Gesture classifier thresholds */
#define GESTURE_SETTLE_Z_MIN_MS2     8.0f   /* motion_settled z must be >= this */
#define GESTURE_SETTLE_PEAK_SPEED_MIN 2.5f  /* settle時のpeak速度下限 */
#define GESTURE_SETTLE_DIST_MIN       0.25f /* 移動距離下限 */
#define GESTURE_WINDOW_MS            2000   /* max ms between active and settle */

/* ============================================================================
 * BLE UUIDs (Handy-compatible)
 * ============================================================================ */

/* Audio Service: 00000001-0000-1000-8000-00805f9b34fb */
#define AUDIO_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00

/* Audio TX: 00000002-... (Notify) */
#define AUDIO_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00

/* Audio RX: 00000003-... (Write) */
#define AUDIO_UUID_RX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00

/* ============================================================================
 * Global Variables
 * ============================================================================ */

/* Advertising data */
static const uint8_t adv_flags_data[] = { BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR };
static const uint8_t adv_uuid_data[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
};
static const struct bt_data adv_data[] = {
    BT_DATA(BT_DATA_FLAGS, adv_flags_data, sizeof(adv_flags_data)),
    BT_DATA(BT_DATA_UUID128_ALL, adv_uuid_data, sizeof(adv_uuid_data)),
};
static const struct bt_data scan_rsp[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
};

static struct bt_conn *current_conn;

/* Audio state */
static uint8_t tx_packet[512];
static uint8_t seq_num;
static volatile bool is_recording;
static volatile bool recording_requested;
static volatile bool stop_requested;

/* Audio thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static struct k_thread audio_thread_data;

/* IMU device */
static const struct device *const imu     = DEVICE_DT_GET(IMU_NODE);

/* pdm_imu_pwr regulator (shared PDM + IMU supply, GPIO0.1) */
static const struct device *const pdm_imu_reg =
    DEVICE_DT_GET(DT_NODELABEL(pdm_imu_pwr));

/* Gesture state */
static int64_t motion_active_start_ms;
static float gesture_active_z;

/* Motion detection state */
static atomic_t detected_motion_count;
static uint32_t last_motion_time_ms;
static bool baseline_valid;
static double baseline_x, baseline_y, baseline_z;
static uint8_t calibration_count, active_high_count, settle_count;
static bool motion_active;
static bool inhibit_next_settle;   /* 録音中断直後のsettleをスキップ */
static int64_t last_report_ms;
static bool previous_sample_valid;
static double previous_accel_x, previous_accel_y, previous_accel_z;
static double step_window[ACTIVITY_WINDOW_SAMPLES];
static uint8_t step_window_index, step_window_count;
static double step_window_sum;
static double z_excursion_peak;

/* Extended motion metrics */
static float motion_vel_x, motion_vel_y, motion_vel_z;
static float motion_distance;
static float motion_peak_speed;
static float motion_speed_sum;
static uint32_t motion_speed_samples;

/* Watchdog */
static const struct device *const wdt = DEVICE_DT_GET(WDT_NODE);
static int wdt_channel_id = -1;

/* Single LED GPIO spec */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* LED state machine */
typedef enum {
    LED_BOOT,
    LED_ADVERTISING,
    LED_CONNECTED,
    LED_RECORDING,
    LED_ERROR,
} led_state_t;

static led_state_t current_led_state = LED_BOOT;
static int64_t led_state_enter_ms;
static bool blink_on;
static int64_t blink_next_ms;

/* ============================================================================
 * Single LED
 * ============================================================================ */

static int configure_leds(void)
{
    if (!gpio_is_ready_dt(&led)) {
        LOG_WRN("LED GPIO not ready");
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
}

static void led_set_single(bool on)
{
    gpio_pin_set_dt(&led, on ? 1 : 0);
}

static void led_set_state(led_state_t state)
{
    current_led_state = state;
    led_state_enter_ms = k_uptime_get();

    switch (state) {
    case LED_BOOT:
        led_set_single(true);
        break;
    case LED_ADVERTISING:
        led_set_single(true);
        blink_on = true;
        blink_next_ms = k_uptime_get() + ADV_BLINK_ON_MS;
        break;
    case LED_CONNECTED:
        led_set_single(true);
        break;
    case LED_RECORDING:
        led_set_single(true);
        blink_on = true;
        blink_next_ms = k_uptime_get() + RECORDING_BLINK_MS;
        break;
    case LED_ERROR:
        led_set_single(true);
        blink_on = true;
        blink_next_ms = k_uptime_get() + ERROR_BLINK_MS;
        break;
    }
}

/* Called from main loop every MAIN_LOOP_INTERVAL_MS to handle blink patterns. */
static void led_tick(void)
{
    int64_t now = k_uptime_get();

    if (current_led_state == LED_ADVERTISING) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            led_set_single(blink_on);
            blink_next_ms = now + (blink_on ? ADV_BLINK_ON_MS : ADV_BLINK_OFF_MS);
        }
    } else if (current_led_state == LED_RECORDING) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            led_set_single(blink_on);
            blink_next_ms = now + RECORDING_BLINK_MS;
        }
    } else if (current_led_state == LED_ERROR) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            led_set_single(blink_on);
            blink_next_ms = now + ERROR_BLINK_MS;
        }
    } else if (current_led_state == LED_BOOT) {
        /* Turn off after 1 second */
        if ((now - led_state_enter_ms) >= 1000) {
            led_set_single(false);
        }
    }
}

/* ============================================================================
 * Reset Cause & Watchdog
 * ============================================================================ */

static void log_reset_cause(void)
{
    uint32_t cause = 0;

    hwinfo_get_reset_cause(&cause);
    hwinfo_clear_reset_cause();

    printk("Reset cause: 0x%08x", cause);
    if (cause & RESET_WATCHDOG)   printk(" [WATCHDOG — firmware hang]");
    if (cause & RESET_BROWNOUT)   printk(" [BROWNOUT — low voltage/battery]");
    if (cause & RESET_PIN)        printk(" [PIN — external reset]");
    if (cause & RESET_SOFTWARE)   printk(" [SOFTWARE]");
    if (cause & RESET_CPU_LOCKUP) printk(" [CPU_LOCKUP]");
    if (cause & RESET_POR)        printk(" [POR — power-on]");
    if (cause == 0)               printk(" [POR — power-on / battery removed]");
    printk("\n");
}

static void configure_watchdog(void)
{
    if (!device_is_ready(wdt)) {
        LOG_WRN("WDT not ready — firmware hang detection disabled");
        return;
    }
    struct wdt_timeout_cfg cfg = {
        .flags = WDT_FLAG_RESET_SOC,
        .window = { .min = 0U, .max = WDT_TIMEOUT_MS },
    };
    wdt_channel_id = wdt_install_timeout(wdt, &cfg);
    if (wdt_channel_id < 0) {
        LOG_ERR("WDT install failed: %d", wdt_channel_id);
        return;
    }
    if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) {
        LOG_ERR("WDT setup failed");
        return;
    }
    LOG_INF("WDT armed: %d ms timeout", WDT_TIMEOUT_MS);
    printk("Watchdog armed (%d ms)\n", WDT_TIMEOUT_MS);
}

/* ============================================================================
 * Mic Power
 *
 * The pdm_imu_pwr regulator on XIAO nRF54L15 is marked regulator-boot-on
 * in the board DTS, so power is automatically enabled at startup.
 * No GPIO control needed.
 * ============================================================================ */

static void mic_power_on(void)
{
    k_msleep(50);   /* wait for MEMS bias to stabilise */
    printk("Mic power ON (auto)\n");
}

static void mic_power_off(void)
{
    printk("Mic power OFF (auto)\n");
}

/* Forward declarations */
static void send_event_packet(uint8_t event_code);
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z);
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance);

/* ============================================================================
 * Motion Detection
 * ============================================================================ */

static double abs_double(double v) { return v < 0.0 ? -v : v; }

static void set_baseline(double x, double y, double z)
{
    baseline_x = x; baseline_y = y; baseline_z = z;
    baseline_valid = true;
}

static void update_baseline(double x, double y, double z)
{
    if (!baseline_valid) { set_baseline(x, y, z); return; }
    baseline_x += (x - baseline_x) * BASELINE_ALPHA;
    baseline_y += (y - baseline_y) * BASELINE_ALPHA;
    baseline_z += (z - baseline_z) * BASELINE_ALPHA;
}

static double motion_delta(double x, double y, double z)
{
    return abs_double(x - baseline_x) + abs_double(y - baseline_y) +
           abs_double(z - baseline_z);
}

static double sample_delta(double x, double y, double z)
{
    if (!previous_sample_valid) {
        previous_accel_x = x; previous_accel_y = y; previous_accel_z = z;
        previous_sample_valid = true;
        return 0.0;
    }
    double d = abs_double(x - previous_accel_x) + abs_double(y - previous_accel_y) +
               abs_double(z - previous_accel_z);
    previous_accel_x = x; previous_accel_y = y; previous_accel_z = z;
    return d;
}

static void reset_step_window(void)
{
    for (size_t i = 0; i < ACTIVITY_WINDOW_SAMPLES; ++i) step_window[i] = 0.0;
    step_window_index = step_window_count = 0;
    step_window_sum = 0.0;
}

static void update_step_window(double s)
{
    if (step_window_count == ACTIVITY_WINDOW_SAMPLES) {
        step_window_sum -= step_window[step_window_index];
    } else {
        step_window_count++;
    }
    step_window[step_window_index] = s;
    step_window_sum += s;
    step_window_index = (step_window_index + 1U) % ACTIVITY_WINDOW_SAMPLES;
}

static double step_window_peak(void)
{
    double p = 0.0;
    for (size_t i = 0; i < step_window_count; ++i) {
        if (step_window[i] > p) p = step_window[i];
    }
    return p;
}

static void accumulate_calibration(double x, double y, double z)
{
    if (calibration_count == 0) {
        set_baseline(x, y, z);
    } else {
        baseline_x = ((baseline_x * calibration_count) + x) / (calibration_count + 1);
        baseline_y = ((baseline_y * calibration_count) + y) / (calibration_count + 1);
        baseline_z = ((baseline_z * calibration_count) + z) / (calibration_count + 1);
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
        last_motion_time_ms = (uint32_t)k_uptime_get();
    }
}

static int configure_motion_detection(void)
{
    struct sensor_value odr = {ACCEL_ODR_HZ, 0};
    struct sensor_value threshold = {1, 500000};
    struct sensor_value duration = {MOTION_DURATION_SAMPLES, 0};
    int ret;

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) { LOG_ERR("ODR set failed: %d", ret); return ret; }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &threshold);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_TH not supported");
    else if (ret < 0) { LOG_ERR("threshold set failed: %d", ret); return ret; }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &duration);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_DUR not supported");
    else if (ret < 0) { LOG_ERR("duration set failed: %d", ret); return ret; }

    LOG_INF("Motion detection ready: ODR=%d Hz, motion_sample=%d ms",
            ACCEL_ODR_HZ, MOTION_SAMPLE_INTERVAL_MS);
    LOG_INF("Calibrating for %.1f s; keep the board still",
            (double)(CALIBRATION_SAMPLES * MOTION_SAMPLE_INTERVAL_MS) / 1000.0);
    return 0;
}

/* ============================================================================
 * Gesture Detection
 * ============================================================================ */

static void on_motion_started(float x, float y, float z)
{
    motion_active_start_ms = k_uptime_get();
    gesture_active_z = z;

    motion_vel_x = 0.0f; motion_vel_y = 0.0f; motion_vel_z = 0.0f;
    motion_distance   = 0.0f;
    motion_peak_speed = 0.0f;
    motion_speed_sum  = 0.0f;
    motion_speed_samples = 0;

    send_event_packet_xyz(0x10, x, y, z);
    printk(">>> Motion active x=%.2f y=%.2f z=%.2f\n", (double)x, (double)y, (double)z);

    if (is_recording) {
        printk(">>> Motion active while recording → stop\n");
        stop_requested = true;
        inhibit_next_settle = true;
    }
}

static void on_motion_settled(float x, float y, float z)
{
    if (is_recording) {
        last_motion_time_ms = (uint32_t)k_uptime_get();
        int64_t elapsed = k_uptime_get() - motion_active_start_ms;
        uint32_t elapsed_ms = (uint32_t)(elapsed < 0 ? 0 : elapsed);
        float avg_speed = motion_speed_samples > 0
            ? motion_speed_sum / (float)motion_speed_samples : 0.0f;
        send_event_packet_settle(x, y, z, elapsed_ms, avg_speed, motion_peak_speed, motion_distance);
        printk(">>> Motion settled x=%.2f y=%.2f z=%.2f elapsed=%u ms avg=%.3f peak=%.3f dist=%.3f\n",
               (double)x, (double)y, (double)z, elapsed_ms,
               (double)avg_speed, (double)motion_peak_speed, (double)motion_distance);
        return;
    }

    int64_t elapsed = k_uptime_get() - motion_active_start_ms;
    uint32_t elapsed_ms = (uint32_t)(elapsed < 0 ? 0 : elapsed);
    float avg_speed = motion_speed_samples > 0
        ? motion_speed_sum / (float)motion_speed_samples : 0.0f;
    if (inhibit_next_settle) {
        inhibit_next_settle = false;
        printk(">>> Motion settled (inhibited after recording stop)\n");
        return;
    }

    bool settle_z_ok   = (z >= GESTURE_SETTLE_Z_MIN_MS2);
    bool window_ok     = (elapsed <= GESTURE_WINDOW_MS);
    bool peak_ok       = (motion_peak_speed >= GESTURE_SETTLE_PEAK_SPEED_MIN);
    bool dist_ok       = (motion_distance >= GESTURE_SETTLE_DIST_MIN);
    bool active_z_ok   = (gesture_active_z < 0.0f);

    last_motion_time_ms = (uint32_t)k_uptime_get();
    send_event_packet_settle(x, y, z, elapsed_ms, avg_speed, motion_peak_speed, motion_distance);
    printk(">>> Motion settled x=%.2f y=%.2f z=%.2f elapsed=%u ms avg=%.3f peak=%.3f dist=%.3f\n",
           (double)x, (double)y, (double)z, elapsed_ms,
           (double)avg_speed, (double)motion_peak_speed, (double)motion_distance);

    printk(">>> Gesture check: active_z=%.2f settle_z=%.2f elapsed=%lld ms peak=%.3f dist=%.3f\n",
           (double)gesture_active_z, (double)z, (long long)elapsed,
           (double)motion_peak_speed, (double)motion_distance);

    if (settle_z_ok && window_ok && peak_ok && dist_ok && active_z_ok) {
        printk(">>> Gesture MATCH → recording_start\n");
        recording_requested = true;
    }
}


static void process_motion_sample(void)
{
    struct sensor_value ax, ay, az;
    int ret = sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
    if (ret < 0) { LOG_ERR("fetch failed: %d", ret); return; }
    if (sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax) < 0 ||
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay) < 0 ||
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az) < 0) return;

    double x = sensor_value_to_double(&ax);
    double y = sensor_value_to_double(&ay);
    double z = sensor_value_to_double(&az);

    if (calibration_count < CALIBRATION_SAMPLES) {
        accumulate_calibration(x, y, z);
        return;
    }

    double delta    = motion_delta(x, y, z);
    double step     = sample_delta(x, y, z);
    update_step_window(step);
    double activity = step_window_sum;
    double peak     = step_window_peak();
    int64_t now     = k_uptime_get();

    ARG_UNUSED(delta);

    if (!motion_active) {
        if (activity >= MOTION_ENTRY_ACTIVITY_MS2 && peak >= MOTION_ENTRY_PEAK_MS2) {
            active_high_count++;
        } else {
            active_high_count = 0;
            update_baseline(x, y, z);
        }

        if (active_high_count >= MOTION_START_WINDOWS &&
            (now - last_report_ms) >= REPORT_COOLDOWN_MS) {
            motion_active = true;
            settle_count = 0;
            active_high_count = 0;
            last_report_ms = now;
            z_excursion_peak = abs_double(z - baseline_z);
            atomic_inc(&detected_motion_count);

            LOG_INF("Motion! count=%d activity=%.3f peak=%.3f",
                    (int)atomic_get(&detected_motion_count), activity, peak);
            last_motion_time_ms = now;
            on_motion_started((float)x, (float)y, (float)z);
        }
        return;
    }

    /* Motion is active — update running z_excursion_peak */
    double z_dev = abs_double(z - baseline_z);
    if (z_dev > z_excursion_peak) {
        z_excursion_peak = z_dev;
    }

    /* Velocity & distance integration */
    static const float DT = MOTION_SAMPLE_INTERVAL_MS / 1000.0f;
    motion_vel_x += (float)(x - baseline_x) * DT;
    motion_vel_y += (float)(y - baseline_y) * DT;
    motion_vel_z += (float)(z - baseline_z) * DT;
    float spd = sqrtf(motion_vel_x * motion_vel_x +
                      motion_vel_y * motion_vel_y +
                      motion_vel_z * motion_vel_z);
    if (spd > motion_peak_speed) { motion_peak_speed = spd; }
    motion_distance += spd * DT;
    motion_speed_sum += spd;
    motion_speed_samples++;

    if ((activity >= MOTION_CONTINUE_ACTIVITY_MS2 || peak >= MOTION_CONTINUE_PEAK_MS2) &&
        (now - last_report_ms) >= REPORT_COOLDOWN_MS) {
        last_report_ms = now;
        last_motion_time_ms = now;
    }

    if (activity <= MOTION_SETTLE_ACTIVITY_MS2 && peak <= MOTION_SETTLE_PEAK_MS2) {
        settle_count++;
    } else {
        settle_count = 0;
    }

    if (settle_count >= MOTION_SETTLE_WINDOWS) {
        motion_active = false;
        settle_count = 0;
        set_baseline(x, y, z);
        reset_step_window();
        LOG_INF("Motion settled: baseline=(%.3f, %.3f, %.3f)", x, y, z);
        on_motion_settled((float)x, (float)y, (float)z);
    }
}

/* ============================================================================
 * BLE Audio Service
 * ============================================================================ */

static void audio_tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Audio TX CCCD updated: %d", value);
    printk(">>> Audio CCCD: %d (1=notify enabled)\n", value);
}

static ssize_t audio_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset,
                              uint8_t flags)
{
    const uint8_t *data = buf;

    if (len >= 1) {
        if (data[0] == 0x01) {
            printk(">>> START command\n");
            stop_requested = false;
            recording_requested = true;
        } else if (data[0] == 0x00) {
            printk(">>> STOP command\n");
            stop_requested = true;
        }
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(audio_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(AUDIO_UUID_SERVICE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(AUDIO_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(audio_tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(AUDIO_UUID_RX_CHAR),
        BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, audio_rx_write, NULL),
);

/* Send event packet via TX characteristic: [0x00][0x55][event_code] */
static void send_event_packet(uint8_t event_code)
{
    if (!current_conn) {
        return;
    }
    uint8_t pkt[3] = { 0x00, 0x55, event_code };
    bt_gatt_notify(NULL, &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* motion_active packet: [0x00][0x55][code][f32_x 4B][f32_y 4B][f32_z 4B] = 15 bytes */
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z)
{
    if (!current_conn) {
        return;
    }
    uint8_t pkt[15];
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = event_code;
    memcpy(&pkt[3], &x, 4);
    memcpy(&pkt[7], &y, 4);
    memcpy(&pkt[11], &z, 4);
    bt_gatt_notify(NULL, &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* motion_settled extended packet: [0x00][0x55][0x11][f32_x 4B][f32_y 4B][f32_z 4B]
 *   [u32_elapsed_ms 4B][f32_avg_speed 4B][f32_peak_speed 4B][f32_distance 4B] = 31 bytes */
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance)
{
    if (!current_conn) { return; }
    uint8_t pkt[31];
    pkt[0] = 0x00; pkt[1] = 0x55; pkt[2] = 0x11;
    memcpy(&pkt[3],  &x,          4);
    memcpy(&pkt[7],  &y,          4);
    memcpy(&pkt[11], &z,          4);
    memcpy(&pkt[15], &elapsed_ms, 4);
    memcpy(&pkt[19], &avg_speed,  4);
    memcpy(&pkt[23], &peak_speed, 4);
    memcpy(&pkt[27], &distance,   4);
    bt_gatt_notify(NULL, &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* ============================================================================
 * Audio Streaming
 * ============================================================================ */

static void stream_audio_frame(const int16_t *audio_buffer, size_t audio_size)
{
    size_t total_samples = audio_size / sizeof(int16_t);
    size_t offset = 0;

    while (offset < total_samples) {
        size_t samples_to_send = total_samples - offset;
        if (samples_to_send > PCM_PACKET_SIZE / sizeof(int16_t)) {
            samples_to_send = PCM_PACKET_SIZE / sizeof(int16_t);
        }

        tx_packet[0] = seq_num++;
        tx_packet[1] = 0xAA;
        memcpy(&tx_packet[2], &audio_buffer[offset],
               samples_to_send * sizeof(int16_t));

        int ret = bt_gatt_notify(NULL, &audio_svc.attrs[2],
                                 tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        if (ret == -ENOMEM) {
            k_msleep(1);
            ret = bt_gatt_notify(NULL, &audio_svc.attrs[2],
                                 tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        }

        offset += samples_to_send;
        k_msleep(1);
    }
}

static void audio_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;
    int16_t *audio_buffer;
    size_t audio_size;
    bool dmic_available = false;
    bool dmic_session_active = false;

    printk("Audio thread started\n");

    ret = audio_capture_init();
    if (ret < 0) {
        printk("Audio init failed: %d — audio disabled\n", ret);
    } else {
        dmic_available = true;
        printk("Audio capture initialized\n");
    }

    while (1) {
        if (recording_requested && !is_recording) {
            recording_requested = false;
            stop_requested = false;
            seq_num = 0;

            if (dmic_available) {
                mic_power_on();
                ret = audio_capture_start();
                if (ret < 0) {
                    printk("Audio start failed: %d\n", ret);
                    mic_power_off();
                    dmic_session_active = false;
                } else {
                    dmic_session_active = true;
                    printk("Audio capture running\n");
                }
            }

            is_recording = true;
            printk("Recording started\n");
            send_event_packet(0x01);
        }

        if (stop_requested && is_recording) {
            stop_requested = false;
            is_recording = false;

            if (dmic_session_active) {
                ret = audio_capture_stop();
                if (ret < 0) {
                    printk("Audio stop failed: %d\n", ret);
                }
                mic_power_off();
                dmic_session_active = false;
            }

            printk("Recording stopped\n");
            send_event_packet(0x02);
        }

        if (is_recording && current_conn) {
            if (dmic_session_active) {
                ret = audio_capture_get_data(&audio_buffer, &audio_size);
                if (ret < 0) {
                    printk("Audio read failed: %d\n", ret);
                    (void)audio_capture_stop();
                    dmic_session_active = false;
                    continue;
                }
                stream_audio_frame(audio_buffer, audio_size);
            }
            continue;
        }

        if (!current_conn && dmic_session_active) {
            (void)audio_capture_stop();
            mic_power_off();
            dmic_session_active = false;
        }

        k_msleep(20);
    }
}

/* ============================================================================
 * BLE Connection Callbacks
 * ============================================================================ */

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    if (att_err) {
        printk(">>> MTU exchange failed: %d\n", att_err);
    } else {
        printk(">>> MTU exchange done\n");
    }
}

static struct bt_gatt_exchange_params exchange_params = {
    .func = exchange_func,
};

/* ============================================================================
 * pdm_imu_pwr Power Management
 * ============================================================================ */

static void pdm_imu_power_on(void)
{
    if (!device_is_ready(pdm_imu_reg)) { return; }
    regulator_enable(pdm_imu_reg);
    k_msleep(10); /* regulator settle (DTS startup-delay-us=5000 already handled) */
    calibration_count = 0;
    baseline_valid    = false;
    previous_sample_valid = false;
    motion_active     = false;
    printk("pdm_imu_pwr ON — IMU recalibrating\n");
}

static void pdm_imu_power_off(void)
{
    if (!device_is_ready(pdm_imu_reg)) { return; }
    regulator_disable(pdm_imu_reg);
    calibration_count = 0;
    baseline_valid    = false;
    motion_active     = false;
    printk("pdm_imu_pwr OFF\n");
}

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("BLE connection failed: %d", err); return; }

    LOG_INF("BLE connected");
    printk(">>> Connected!\n");
    current_conn = conn;
    led_set_state(LED_CONNECTED);
    pdm_imu_power_on();

    int ret = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (ret) {
        printk(">>> MTU exchange request failed: %d\n", ret);
    }

    struct bt_le_conn_param conn_param = {
        .interval_min = 6,
        .interval_max = 12,
        .latency = 0,
        .timeout = 400,
    };
    ret = bt_conn_le_param_update(conn, &conn_param);
    if (ret) {
        LOG_WRN("Conn param update failed: %d", ret);
    }
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected: %d", reason);
    printk(">>> Disconnected: %d\n", reason);
    current_conn = NULL;
    is_recording = false;
    stop_requested = true;
    led_set_state(LED_ADVERTISING);
    pdm_imu_power_off();

    int ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data),
                              scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret && ret != -EALREADY) {
        LOG_WRN("Advertising restart failed: %d", ret);
        printk(">>> Adv restart failed: %d\n", ret);
    } else {
        printk(">>> Advertising restarted\n");
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = ble_connected,
    .disconnected = ble_disconnected,
};

/* ============================================================================
 * Main
 * ============================================================================ */

int main(void)
{
    int ret;

    printk("\n");
    printk("============================================\n");
    printk("nrf54-handy: XIAO nRF54L15 Sense\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("============================================\n\n");

    log_reset_cause();

    LOG_INF("Starting nrf54-handy");

    ret = configure_leds();
    if (ret < 0) LOG_WRN("LED setup failed: %d", ret);

    led_set_state(LED_BOOT);  /* ON 1 second */

    if (!device_is_ready(imu)) {
        LOG_WRN("IMU not ready — gesture detection disabled");
    } else {
        ret = configure_motion_detection();
        if (ret < 0) LOG_WRN("Motion detection setup failed: %d", ret);
    }

    ret = bt_enable(NULL);
    if (ret) { LOG_ERR("BLE enable failed: %d", ret); return ret; }

    bt_conn_cb_register(&conn_callbacks);

    ret = bt_set_name(BLE_DEVICE_NAME);
    if (ret) { LOG_ERR("BLE name set failed: %d", ret); return ret; }

    ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data),
                          scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret) { LOG_ERR("BLE adv failed: %d", ret); return ret; }

    LOG_INF("BLE advertising: %s", BLE_DEVICE_NAME);

    /* Start audio thread (priority 14, below BLE stack) */
    k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread, NULL, NULL, NULL, 14, 0, K_NO_WAIT);

    LOG_INF("nrf54-handy ready");

    /* Boot LED: on for ~1 second, then switch to advertising blink */
    k_msleep(1000);
    led_set_state(LED_ADVERTISING);

    /* Auto-confirm OTA image after 3s total — basic sanity that we didn't crash */
    k_msleep(2000);
    if (!boot_is_img_confirmed()) {
        int img_ret = boot_write_img_confirmed();
        if (img_ret < 0) {
            LOG_WRN("Image confirm failed: %d", img_ret);
            printk("Image confirm failed: %d\n", img_ret);
        } else {
            LOG_INF("Image confirmed");
            printk("Image confirmed (permanent)\n");
        }
    }

    /* Arm watchdog after image confirmation */
    configure_watchdog();

    /* Main loop: watchdog feed + LED state management + blink tick + IMU motion */
    int64_t last_motion_sample_ms = k_uptime_get();
    while (1) {
        k_msleep(MAIN_LOOP_INTERVAL_MS);

        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }

        /* Sync LED to recording state when connected */
        if (current_conn) {
            if (is_recording && current_led_state != LED_RECORDING) {
                led_set_state(LED_RECORDING);
            } else if (!is_recording && current_led_state == LED_RECORDING) {
                led_set_state(LED_CONNECTED);
            }
        }

        int64_t now_ms = k_uptime_get();

        /* IMU motion detection */
        int64_t imu_poll_ms = MOTION_SAMPLE_INTERVAL_MS;
        if (device_is_ready(imu) &&
            (now_ms - last_motion_sample_ms) >= imu_poll_ms) {
            last_motion_sample_ms = now_ms;
            process_motion_sample();
        }

        led_tick();
    }

    return 0;
}
