/*
 * nrf52-handy: XIAO nRF52840 Sense → Handy BLE firmware
 *
 * Audio streaming via BLE with Handy-compatible protocol.
 * IMU gesture detection triggers BLE event packets to control Handy recording.
 * RGB LED indicates device state.
 *
 * BLE Protocol (Handy-compatible):
 *   Audio Service:  00000001-0000-1000-8000-00805f9b34fb
 *     TX 0x0002 (NOTIFY): [seq][0xAA][PCM...]  — audio stream
 *                         [0x00][0x55][event]  — event packets
 *     RX 0x0003 (WRITE):  0x01=start, 0x00=stop
 *
 * LED states (active-low, Zephyr GPIO_ACTIVE_LOW flag handles inversion):
 *   Boot        White  1s then off
 *   Advertising Blue   500ms ON / 1500ms OFF blink
 *   Connected   Green  solid
 *   Recording   Red    solid
 *   Error       Red    200ms fast blink
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
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/dfu/mcuboot.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(nrf52_handy, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME         "XIAOVoice"
#define PCM_PACKET_SIZE         200
#define WDT_NODE                DT_NODELABEL(wdt0)
#define WDT_TIMEOUT_MS          5000
#define MAIN_LOOP_INTERVAL_MS   100
#define ADV_BLINK_ON_MS         500
#define ADV_BLINK_OFF_MS        1500
#define ERROR_BLINK_MS          200

#define LED_RED_NODE    DT_ALIAS(led0)
#define LED_GREEN_NODE  DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)

/* IMU / motion detection */
#define IMU_NODE                    DT_ALIAS(imu0)
#define ACCEL_ODR_HZ                52
#define MOTION_SAMPLE_INTERVAL_MS   25
#define CALIBRATION_SAMPLES         25
#define ACTIVITY_WINDOW_SAMPLES     4
#define MOTION_ENTRY_ACTIVITY_MS2   8.0
#define MOTION_ENTRY_PEAK_MS2       2.4
#define MOTION_CONTINUE_ACTIVITY_MS2 4.0
#define MOTION_CONTINUE_PEAK_MS2    1.4
#define MOTION_SETTLE_ACTIVITY_MS2  2.0
#define MOTION_SETTLE_PEAK_MS2      0.8
#define MOTION_START_WINDOWS        2
#define MOTION_SETTLE_WINDOWS       4
#define BASELINE_ALPHA              0.03
#define REPORT_COOLDOWN_MS          700
#define MOTION_DURATION_SAMPLES     2

/* Gesture classifier thresholds */
#define GESTURE_ACTIVE_Z_MIN_MS2    (-3.0f)  /* motion_active z must be <= this */
#define GESTURE_ACTIVE_Z_MAX_MS2    3.0f     /* motion_active z must be >= this */
#define GESTURE_SETTLE_Z_MIN_MS2    8.0f     /* motion_settled z must be >= this */
#define GESTURE_WINDOW_MS           2000     /* max ms between active and settle */

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
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);

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
static int64_t last_report_ms;
static bool previous_sample_valid;
static double previous_accel_x, previous_accel_y, previous_accel_z;
static double step_window[ACTIVITY_WINDOW_SAMPLES];
static uint8_t step_window_index, step_window_count;
static double step_window_sum;
static double z_excursion_peak;

/* Watchdog */
static const struct device *const wdt = DEVICE_DT_GET(WDT_NODE);
static int wdt_channel_id = -1;

/* RGB LED GPIO specs */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE,   gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE,  gpios);

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

/* Microphone power enable (P1.10, active-high) */
#define MIC_PWR_NODE DT_NODELABEL(msm261d3526hicpm_c_en)
static const struct gpio_dt_spec mic_pwr =
    GPIO_DT_SPEC_GET(MIC_PWR_NODE, enable_gpios);

/* ============================================================================
 * RGB LED
 * ============================================================================ */

static int configure_leds(void)
{
    if (!gpio_is_ready_dt(&led_red) ||
        !gpio_is_ready_dt(&led_green) ||
        !gpio_is_ready_dt(&led_blue)) {
        LOG_WRN("RGB LED GPIOs not ready");
        return -ENODEV;
    }
    int ret = 0;

    ret |= gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
    return ret;
}

/* Set logical RGB values. Zephyr's GPIO_ACTIVE_LOW flag handles active-low inversion. */
static void rgb_set(bool r, bool g, bool b)
{
    gpio_pin_set_dt(&led_red,   r ? 1 : 0);
    gpio_pin_set_dt(&led_green, g ? 1 : 0);
    gpio_pin_set_dt(&led_blue,  b ? 1 : 0);
}

static void led_set_state(led_state_t state)
{
    current_led_state = state;
    led_state_enter_ms = k_uptime_get();

    switch (state) {
    case LED_BOOT:
        rgb_set(true, true, true);   /* White */
        break;
    case LED_ADVERTISING:
        rgb_set(false, false, true); /* Blue ON to start */
        blink_on = true;
        blink_next_ms = k_uptime_get() + ADV_BLINK_ON_MS;
        break;
    case LED_CONNECTED:
        rgb_set(false, true, false); /* Green solid */
        break;
    case LED_RECORDING:
        rgb_set(true, false, false); /* Red solid */
        break;
    case LED_ERROR:
        rgb_set(true, false, false); /* Red ON to start */
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
            rgb_set(false, false, blink_on);
            blink_next_ms = now + (blink_on ? ADV_BLINK_ON_MS : ADV_BLINK_OFF_MS);
        }
    } else if (current_led_state == LED_ERROR) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            rgb_set(blink_on, false, false);
            blink_next_ms = now + ERROR_BLINK_MS;
        }
    } else if (current_led_state == LED_BOOT) {
        /* Turn off white after 1 second */
        if ((now - led_state_enter_ms) >= 1000) {
            rgb_set(false, false, false);
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
 * ============================================================================ */

static int configure_mic_power(void)
{
    if (!gpio_is_ready_dt(&mic_pwr)) {
        LOG_ERR("Mic power GPIO not ready");
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&mic_pwr, GPIO_OUTPUT_INACTIVE);
}

static void mic_power_on(void)
{
    gpio_pin_set_dt(&mic_pwr, 1);
    k_msleep(50);   /* wait for MEMS bias to stabilise */
    printk("Mic power ON\n");
}

static void mic_power_off(void)
{
    gpio_pin_set_dt(&mic_pwr, 0);
    printk("Mic power OFF\n");
}

/* Forward declarations */
static void send_event_packet(uint8_t event_code);
static void send_event_packet_f32(uint8_t event_code, float val);

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
        last_motion_time_ms = 0;
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

static void on_motion_started(float z)
{
    motion_active_start_ms = k_uptime_get();
    gesture_active_z = z;
    send_event_packet_f32(0x10, z);
    printk(">>> Motion active z=%.2f\n", (double)z);

    if (is_recording) {
        printk(">>> Motion active while recording → stop\n");
        stop_requested = true;
    }
}

static void on_motion_settled(float z)
{
    send_event_packet_f32(0x11, z);
    printk(">>> Motion settled z=%.2f\n", (double)z);

    if (is_recording) {
        return;
    }

    int64_t elapsed = k_uptime_get() - motion_active_start_ms;
    bool active_z_ok  = (gesture_active_z >= GESTURE_ACTIVE_Z_MIN_MS2 &&
                         gesture_active_z <= GESTURE_ACTIVE_Z_MAX_MS2);
    bool settle_z_ok  = (z >= GESTURE_SETTLE_Z_MIN_MS2);
    bool window_ok    = (elapsed <= GESTURE_WINDOW_MS);

    printk(">>> Gesture check: active_z=%.2f settle_z=%.2f elapsed=%lld ms\n",
           (double)gesture_active_z, (double)z, (long long)elapsed);

    if (active_z_ok && settle_z_ok && window_ok) {
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
            on_motion_started((float)z);
        }
        return;
    }

    /* Motion is active — update running z_excursion_peak */
    double z_dev = abs_double(z - baseline_z);
    if (z_dev > z_excursion_peak) {
        z_excursion_peak = z_dev;
    }

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
        on_motion_settled((float)z);
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

/* Event packet with float payload: [0x00][0x55][code][val f32] = 7 bytes */
static void send_event_packet_f32(uint8_t event_code, float val)
{
    if (!current_conn) {
        return;
    }
    uint8_t pkt[7];
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = event_code;
    memcpy(&pkt[3], &val, 4);
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

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("BLE connection failed: %d", err); return; }

    LOG_INF("BLE connected");
    printk(">>> Connected!\n");
    current_conn = conn;
    led_set_state(LED_CONNECTED);

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

    usb_enable(NULL);
    k_msleep(500);

    printk("\n");
    printk("============================================\n");
    printk("nrf52-handy: XIAO nRF52840 Sense\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("============================================\n\n");

    log_reset_cause();

    LOG_INF("Starting nrf52-handy");

    ret = configure_leds();
    if (ret < 0) LOG_WRN("LED setup failed: %d", ret);

    led_set_state(LED_BOOT);  /* White 1 second */

    ret = configure_mic_power();
    if (ret < 0) LOG_WRN("Mic power GPIO setup failed: %d", ret);

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

    LOG_INF("nrf52-handy ready");

    /* Boot LED: white for ~1 second, then switch to advertising blink */
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
        if (device_is_ready(imu) &&
            (now_ms - last_motion_sample_ms) >= MOTION_SAMPLE_INTERVAL_MS) {
            last_motion_sample_ms = now_ms;
            process_motion_sample();
        }

        led_tick();
    }

    return 0;
}
