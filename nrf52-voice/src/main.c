/*
 * Voice Bridge BLE - XIAO nRF52840 Sense
 * Audio + Motion + OTA firmware
 *
 * Integrates PDM audio capture (from nrf54l15) with IMU motion detection
 * (from nrf52-motion). Motion onset starts audio streaming; motion settle
 * stops it. A 2-second cooldown prevents rapid toggling.
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
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/dfu/mcuboot.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(nrf52_voice, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME             "VoiceBridge52"
#define PCM_PACKET_SIZE             200
#define MOTION_AUDIO_COOLDOWN_MS    2000

/* IMU / motion detection */
#define WDT_NODE                    DT_NODELABEL(wdt0)
#define WDT_TIMEOUT_MS              5000

#define IMU_NODE                    DT_ALIAS(imu0)
#define LED0_NODE                   DT_ALIAS(led0)
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
#define MOTION_DURATION_SAMPLES     2

/* ============================================================================
 * BLE UUIDs
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

/* Motion Service: 00000010-... */
#define MOTION_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00

/* Motion TX: 00000011-... (Notify) */
#define MOTION_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00

/* Build Info: 00000012-... (Read) */
#define MOTION_UUID_BUILD_INFO_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00

/* Wakeup TX: 00000013-... (Notify) */
#define MOTION_UUID_TAP_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00

/* ============================================================================
 * Global Variables
 * ============================================================================ */

static const char build_timestamp[] = __DATE__ " " __TIME__;

/* Advertising data (global so ble_disconnected can restart advertising) */
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

/* Motion → audio cooldown */
static int64_t last_audio_transition_ms;

/* Audio thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static struct k_thread audio_thread_data;

/* I2C / wakeup detection */
#define LSM6DS3_I2C_NODE        DT_BUS(DT_NODELABEL(lsm6ds3tr_c))
#define LSM6DS3_I2C_ADDR        0x6a

#define LSM6DS3_REG_WAKE_UP_SRC     0x1B
#define LSM6DS3_REG_TAP_CFG         0x58
#define LSM6DS3_REG_WAKE_UP_THS     0x5B
#define LSM6DS3_REG_WAKE_UP_DUR     0x5C
#define LSM6DS3_REG_MD1_CFG         0x5E

static const struct device *const i2c_bus = DEVICE_DT_GET(LSM6DS3_I2C_NODE);
static atomic_t detected_wakeup_count;

/* Watchdog */
static const struct device *const wdt = DEVICE_DT_GET(WDT_NODE);
static int wdt_channel_id = -1;

/* Motion detection state */
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
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
static double z_excursion_peak;  /* peak |z - baseline_z| during current motion event */

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

/* Microphone power enable (P1.10, active-high) */
#define MIC_PWR_NODE DT_NODELABEL(msm261d3526hicpm_c_en)
static const struct gpio_dt_spec mic_pwr =
    GPIO_DT_SPEC_GET(MIC_PWR_NODE, enable_gpios);

/* ============================================================================
 * Build Info Characteristic
 * ============================================================================ */

static ssize_t read_build_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             build_timestamp, strlen(build_timestamp));
}

/* ============================================================================
 * Audio BLE Service
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
            printk(">>> Manual START command\n");
            recording_requested = true;
            last_audio_transition_ms = k_uptime_get();
        } else if (data[0] == 0x00) {
            printk(">>> Manual STOP command\n");
            stop_requested = true;
            last_audio_transition_ms = k_uptime_get();
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

/* ============================================================================
 * Motion BLE Service
 * ============================================================================ */

static void motion_tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Motion TX CCCD updated: %d", value);
}

static void wakeup_tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Wakeup TX CCCD updated: %d", value);
}

BT_GATT_SERVICE_DEFINE(motion_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(MOTION_UUID_SERVICE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(MOTION_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(motion_tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(MOTION_UUID_TAP_TX_CHAR),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(wakeup_tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(MOTION_UUID_BUILD_INFO_CHAR),
        BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_build_info, NULL, NULL),
);

/* axes: bit2=Z, bit1=Y, bit0=X (from WAKE_UP_SRC) */
static void notify_wakeup_event(uint8_t axes, uint8_t count)
{
    if (!current_conn) {
        return;
    }
    uint8_t pkt[2] = { axes, count };
    bt_gatt_notify(NULL, &motion_svc.attrs[5], pkt, sizeof(pkt));
}

/* event_type: 0x01 = motion active, 0x00 = motion settled */
/* Packet (18 bytes):
 *   [0]    event_type
 *   [1]    count
 *   [2-5]  activity (float)
 *   [6-9]  peak (float)
 *   [10-13] elapsed_ms (uint32)
 *   [14-17] z_excursion_peak (float) — max |z − baseline_z| during this motion
 */
static void notify_motion_event(uint8_t event_type, uint8_t count,
                                double activity, double peak, uint32_t elapsed_ms,
                                double z_excursion)
{
    if (!current_conn) {
        return;
    }

    uint8_t pkt[18];
    float act_f  = (float)activity;
    float peak_f = (float)peak;
    float zexc_f = (float)z_excursion;

    pkt[0] = event_type;
    pkt[1] = count;
    memcpy(&pkt[2],  &act_f,    4);
    memcpy(&pkt[6],  &peak_f,   4);
    memcpy(&pkt[10], &elapsed_ms, 4);
    memcpy(&pkt[14], &zexc_f,   4);

    bt_gatt_notify(NULL, &motion_svc.attrs[2], pkt, sizeof(pkt));
}

/* ============================================================================
 * Motion → Audio Control (2-second cooldown)
 * ============================================================================ */

static void on_motion_started(void)
{
    int64_t now = k_uptime_get();

    if ((now - last_audio_transition_ms) >= MOTION_AUDIO_COOLDOWN_MS) {
        printk(">>> Motion started → requesting audio start\n");
        recording_requested = true;
        last_audio_transition_ms = now;
    } else {
        printk(">>> Motion started (cooldown active, ignoring)\n");
    }
}

static void on_motion_settled(void)
{
    int64_t now = k_uptime_get();

    if ((now - last_audio_transition_ms) >= MOTION_AUDIO_COOLDOWN_MS) {
        printk(">>> Motion settled → requesting audio stop\n");
        stop_requested = true;
        last_audio_transition_ms = now;
    } else {
        printk(">>> Motion settled (cooldown active, ignoring)\n");
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
    /* Pause WDT when halted by debugger so single-stepping doesn't trigger reset */
    if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) {
        LOG_ERR("WDT setup failed");
        return;
    }
    LOG_INF("WDT armed: %d ms timeout", WDT_TIMEOUT_MS);
    printk("Watchdog armed (%d ms)\n", WDT_TIMEOUT_MS);
}

/* ============================================================================
 * LED
 * ============================================================================ */

static int configure_led(void)
{
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
    if (!gpio_is_ready_dt(&led)) {
        LOG_WRN("LED not ready");
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

static int configure_mic_power(void)
{
    if (!gpio_is_ready_dt(&mic_pwr)) {
        LOG_ERR("Mic power GPIO not ready");
        return -ENODEV;
    }
    /* Start with mic OFF; audio thread powers it on when recording starts */
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

static int configure_wakeup_detection(void)
{
    if (!device_is_ready(i2c_bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* TAP_CFG: INTERRUPTS_ENABLE=1 (required for wakeup events to propagate) */
    i2c_reg_write_byte(i2c_bus, LSM6DS3_I2C_ADDR, LSM6DS3_REG_TAP_CFG,     0x80);
    /* WAKE_UP_THS: WK_THS=4 → 4 × (2g/64) = 125 mg threshold */
    i2c_reg_write_byte(i2c_bus, LSM6DS3_I2C_ADDR, LSM6DS3_REG_WAKE_UP_THS, 0x04);
    /* WAKE_UP_DUR: 1 sample at 26 Hz ≈ 38 ms (most responsive) */
    i2c_reg_write_byte(i2c_bus, LSM6DS3_I2C_ADDR, LSM6DS3_REG_WAKE_UP_DUR, 0x00);
    /* MD1_CFG: INT1_WU (bit5) only */
    i2c_reg_write_byte(i2c_bus, LSM6DS3_I2C_ADDR, LSM6DS3_REG_MD1_CFG,     0x20);

    printk("Wakeup configured (125mg threshold)\n");
    return 0;
}

static void check_wakeup_src(void)
{
    uint8_t wu_src = 0;
    if (i2c_reg_read_byte(i2c_bus, LSM6DS3_I2C_ADDR, LSM6DS3_REG_WAKE_UP_SRC, &wu_src) < 0) {
        return;
    }
    if (!(wu_src & BIT(3))) {           /* WU_IA: no wakeup activity */
        return;
    }
    uint8_t axes = wu_src & 0x07;       /* bit2=Z, bit1=Y, bit0=X */
    uint8_t count = (uint8_t)(atomic_inc(&detected_wakeup_count) + 1);
    printk(">>> Wakeup! axes=0x%02x count=%u\n", axes, count);
    notify_wakeup_event(axes, count);
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

    LOG_INF("Motion detection ready: ODR=%d Hz, poll=%d ms", ACCEL_ODR_HZ, POLL_INTERVAL_MS);
    LOG_INF("Calibrating for %.1f s; keep the board still",
            (double)(CALIBRATION_SAMPLES * POLL_INTERVAL_MS) / 1000.0);
    return 0;
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
            pulse_led();

            uint32_t elapsed = last_motion_time_ms ? (uint32_t)(now - last_motion_time_ms) : 0;
            LOG_INF("Motion! count=%d elapsed=%u ms activity=%.3f peak=%.3f z_exc=%.3f",
                    (int)atomic_get(&detected_motion_count), elapsed, activity, peak, z_excursion_peak);
            notify_motion_event(0x01, (uint8_t)atomic_get(&detected_motion_count),
                                activity, peak, elapsed, z_excursion_peak);
            last_motion_time_ms = now;
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
        atomic_inc(&detected_motion_count);
        pulse_led();
        uint32_t elapsed = last_motion_time_ms ? (uint32_t)(now - last_motion_time_ms) : 0;
        LOG_INF("Motion! count=%d elapsed=%u ms activity=%.3f peak=%.3f z_exc=%.3f",
                (int)atomic_get(&detected_motion_count), elapsed, activity, peak, z_excursion_peak);
        notify_motion_event(0x01, (uint8_t)atomic_get(&detected_motion_count),
                            activity, peak, elapsed, z_excursion_peak);
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

        uint32_t elapsed = last_motion_time_ms ? (uint32_t)(k_uptime_get() - last_motion_time_ms) : 0;
        notify_motion_event(0x00, (uint8_t)atomic_get(&detected_motion_count),
                            activity, peak, elapsed, z_excursion_peak);
    }
}

/* ============================================================================
 * Audio Streaming
 * ============================================================================ */

static void stream_audio_frame(const int16_t *audio_buffer, size_t audio_size)
{
    size_t total_samples = audio_size / sizeof(int16_t);
    size_t offset = 0;
    size_t packets_sent = 0;
    size_t notify_errors = 0;

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

        if (ret == 0) {
            packets_sent++;
        } else {
            notify_errors++;
        }

        offset += samples_to_send;
        k_msleep(1);
    }

    if (notify_errors > 0) {
        printk("Audio: %zu samples, sent %zu pkts, errors %zu\n",
               total_samples, packets_sent, notify_errors);
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

    /* Restart advertising so the device stays discoverable */
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
    printk("Voice Bridge BLE - XIAO nRF52840 Sense\n");
    printk("Build: %s\n", build_timestamp);
    printk("============================================\n\n");

    log_reset_cause();

    LOG_INF("Starting VoiceBridge52");

    if (!device_is_ready(imu)) {
        LOG_ERR("IMU %s not ready", imu->name);
        return -ENODEV;
    }

    ret = configure_led();
    if (ret < 0) LOG_WRN("LED setup failed: %d", ret);

    ret = configure_mic_power();
    if (ret < 0) LOG_WRN("Mic power GPIO setup failed: %d", ret);

    ret = configure_motion_detection();
    if (ret < 0) return ret;

    ret = configure_wakeup_detection();
    if (ret < 0) LOG_WRN("Wakeup detection setup failed: %d", ret);

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

    LOG_INF("VoiceBridge52 ready");
    printk("System ready. Calibrating IMU...\n");

    /* Auto-confirm the running image after advertising starts successfully.
     * A 3-second delay is used as a basic sanity check that the firmware
     * has initialized without crashing. This prevents MCUboot from
     * reverting to the previous image on the next reboot.
     */
    k_msleep(3000);
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

    /* Arm watchdog after image confirmation — main loop must kick every POLL_INTERVAL_MS */
    configure_watchdog();

    /* Main loop: poll IMU and check tap events */
    while (1) {
        k_sleep(K_MSEC(POLL_INTERVAL_MS));
        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }
        process_motion_sample();
        check_wakeup_src();
    }

    return 0;
}
