/*
 * XIAO nRF54L15 Sense IMU motion detection test
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
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(nrf54_motion, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu0)
#define LED0_NODE DT_ALIAS(led0)

#define MOTION_THRESHOLD_MS2 1.5
#define MOTION_DURATION_SAMPLES 2
#define ACCEL_ODR_HZ 26
#define POLL_INTERVAL_MS 100
#define CALIBRATION_SAMPLES 25
#define ACTIVITY_WINDOW_SAMPLES 4
#define MOTION_ENTRY_ACTIVITY_MS2 8.0
#define MOTION_ENTRY_PEAK_MS2 2.4
#define MOTION_CONTINUE_ACTIVITY_MS2 4.0
#define MOTION_CONTINUE_PEAK_MS2 1.4
#define MOTION_SETTLE_ACTIVITY_MS2 0.9
#define MOTION_SETTLE_PEAK_MS2 0.35
#define MOTION_START_WINDOWS 2
#define MOTION_SETTLE_WINDOWS 4
#define BASELINE_ALPHA 0.03
#define REPORT_COOLDOWN_MS 700

/* ============================================================================
 * BLE UUIDs
 * ============================================================================ */

/* Motion Service UUID: 00000010-0000-1000-8000-00805f9b34fb */
#define MOTION_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00

/* TX Characteristic UUID: 00000011-0000-1000-8000-00805f9b34fb (Notify) */
#define MOTION_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00

#define BLE_DEVICE_NAME "MotionBridge"

/* ============================================================================
 * BLE Global State
 * ============================================================================ */

static struct bt_conn *current_conn;

static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("TX CCCD updated: %d", value);
}

BT_GATT_SERVICE_DEFINE(motion_svc,
    BT_GATT_PRIMARY_SERVICE(
        BT_UUID_DECLARE_128(MOTION_UUID_SERVICE)),

    /* TX Characteristic (Notify) */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(MOTION_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        NULL, NULL, NULL),
    BT_GATT_CCC(tx_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
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

static void notify_motion_event(uint8_t count, double activity, double peak)
{
    if (!current_conn) {
        return;
    }

    uint8_t pkt[10];
    float act_f  = (float)activity;
    float peak_f = (float)peak;

    pkt[0] = 0x01; /* event_type: motion detected */
    pkt[1] = count;
    memcpy(&pkt[2], &act_f,  4);
    memcpy(&pkt[6], &peak_f, 4);

    bt_gatt_notify(NULL, &motion_svc.attrs[2], pkt, sizeof(pkt));
}

static atomic_t detected_motion_count;

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
    }
}

static int configure_motion_detection(void)
{
    struct sensor_value odr = {
        .val1 = ACCEL_ODR_HZ,
        .val2 = 0,
    };
    struct sensor_value threshold = {
        .val1 = 1,
        .val2 = 500000,
    };
    struct sensor_value duration = {
        .val1 = MOTION_DURATION_SAMPLES,
        .val2 = 0,
    };
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
            LOG_INF("Motion detected! count=%d activity=%.3f peak=%.3f delta=%.3f step=%.3f accel=(%.3f, %.3f, %.3f) m/s^2",
                    (int)atomic_get(&detected_motion_count),
                    activity,
                    peak_step,
                    delta,
                    motion_step,
                    accel_x_ms2, accel_y_ms2, accel_z_ms2);
            notify_motion_event((uint8_t)atomic_get(&detected_motion_count),
                                activity, peak_step);
        }

        return;
    }

    if ((activity >= MOTION_CONTINUE_ACTIVITY_MS2 ||
         peak_step >= MOTION_CONTINUE_PEAK_MS2) &&
        (now_ms - last_report_ms) >= REPORT_COOLDOWN_MS) {
        last_report_ms = now_ms;
        atomic_inc(&detected_motion_count);
        pulse_led();
        LOG_INF("Motion detected! count=%d activity=%.3f peak=%.3f delta=%.3f step=%.3f accel=(%.3f, %.3f, %.3f) m/s^2",
                (int)atomic_get(&detected_motion_count),
                activity,
                peak_step,
                delta,
                motion_step,
                accel_x_ms2, accel_y_ms2, accel_z_ms2);
        notify_motion_event((uint8_t)atomic_get(&detected_motion_count),
                            activity, peak_step);
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

int main(void)
{
    int ret;

    LOG_INF("XIAO nRF54L15 Sense motion detection test starting");

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

    /* Initialize BLE */
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
    LOG_INF("Waiting for motion on %s", imu->name);

    while (1) {
        k_sleep(K_MSEC(POLL_INTERVAL_MS));
        process_motion_sample();
    }

    return 0;
}
