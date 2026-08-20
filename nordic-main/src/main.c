/*
 * nordic-main: XIAO nRF52840 Sense → Handy BLE firmware
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
 *   Idle        Off    advertising / connected idle
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
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/dfu/mcuboot.h>

#include <hal/nrf_power.h>
#include <math.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(nordic_main, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME         "HarnessNode"
#define PCM_PACKET_SIZE         200
#define AUDIO_FRAME_SAMPLES     320   /* 20 ms @ 16 kHz */
#define AUDIO_FRAME_BYTES       (AUDIO_FRAME_SAMPLES * sizeof(int16_t))
/* Software queue decouples DMIC capture from BLE Notify backpressure. */
#define AUDIO_FRAME_QUEUE_LEN   16    /* 320 ms of PCM */
#define AUDIO_CAPTURE_MAX_CONSEC_FAIL 20
#define AUDIO_NOTIFY_IN_FLIGHT  6
#define AUDIO_NOTIFY_RETRY_MS   2
#define AUDIO_NOTIFY_DRAIN_TIMEOUT_MS 1000
#define AUDIO_QUEUE_DRAIN_TIMEOUT_MS 500
#define WDT_NODE                DT_NODELABEL(wdt0)
#define WDT_TIMEOUT_MS          5000
#define MAIN_LOOP_INTERVAL_MS   25
#define ERROR_BLINK_MS          200
#define BATTERY_POLL_INTERVAL_MS 60000  /* 1 minute */
#define BATTERY_CAPACITY_MAH        40   /* LiPo capacity in mAh */
#define BATTERY_CHARGE_CURRENT_MA  100   /* BQ25120 charge current on XIAO nRF52840 */
#define BATTERY_CHARGE_MAX_EST_PCT 100   /* linear to 100% (slightly early, CV phase ignored) */
#define SLEEP_IDLE_TIMEOUT_MS   10000  /* idle this long with no motion → light sleep */
#define SLEEP_POLL_INTERVAL_MS  50

#define LED_RED_NODE    DT_ALIAS(led0)
#define LED_GREEN_NODE  DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)

/* IMU / motion detection (gyro disabled for power saving) */
#define IMU_NODE                    DT_ALIAS(imu0)
#define ACCEL_ODR_HZ                416   /* 416 Hz for tap detection resolution */
#define MOTION_SAMPLE_INTERVAL_MS   25
#define CALIBRATION_SAMPLES         25
#define CALIBRATION_ACCEL_SPAN_MAX_MS2 0.8f
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

/*
 * Dorsal-side gesture classifier (gyro-free).
 *
 * Board on the back of the wrist, component-side against skin.  Axes: X across
 * the forearm, Y along the forearm (pronation axis), Z normal to the board
 * (component-out).  Sequence: palm-down shake → palm-up (3D tilt or
 * XZ phi from the pre-shake gravity LP) → lift → rotate from the
 * palm-up pose and hold still 0.5 s.
 * Recording stops on a palm-up flip from the pose frozen at start
 * (same loose gates as outbound). Dropping the hand does not stop.
 */
#define GESTURE_GRAVITY_MS2                  9.80665f
#define GESTURE_RAD_TO_DEG                   57.29577951308232
#define GESTURE_START_ARM_MS                2500
/* Board-flat gate: |z / |a||. Hold palm-down is a flip from the palm-up pose. */
#define GESTURE_START_PALM_UP_Z_MIN_RATIO       0.80f
#define GESTURE_START_GRAVITY_MIN_MS2           8.5f
#define GESTURE_START_GRAVITY_MAX_MS2          11.5f
#define GESTURE_PHASE_MIN_DURATION_MS           120
#define GESTURE_OUTBOUND_MAX_DURATION_MS       2500
/* Hold flip: gravity rotation in XZ (phi) or a clear Z-axis direction change. */
#define GESTURE_PRONATION_MIN_DEG               20.0f
#define GESTURE_PRONATION_Z_RATIO_DONE           0.50f
#define GESTURE_PRONATION_Z_SIGN_MIN_MS2         2.0f
#define GESTURE_PRONATION_GRAVITY_MIN_MS2        7.5f
#define GESTURE_PRONATION_GRAVITY_MAX_MS2       12.5f
/* Outbound palm-up is looser than hold: pitch (3D) counts, not only XZ phi. */
#define GESTURE_OUTBOUND_MIN_DEG                 8.0f
#define GESTURE_OUTBOUND_Z_RATIO_DONE            0.25f
#define GESTURE_OUTBOUND_TILT_MIN_DEG            15.0f
/* Final: upward acceleration pulse, braking pulse, then a quiet hold. */
#define GESTURE_LIFT_ACCEL_MIN_MS2               0.40f
#define GESTURE_LIFT_BRAKE_MIN_MS2               0.15f
#define GESTURE_LIFT_POS_IMPULSE_MIN_MS           0.04f
#define GESTURE_LIFT_NEG_IMPULSE_MIN_MS           0.015f
#define GESTURE_LIFT_BRAKE_RATIO_MIN              0.05f
#define GESTURE_LIFT_PULSE_MIN_MS                  150
#define GESTURE_LIFT_PULSE_MAX_MS                 1800
#define GESTURE_LIFT_CONSECUTIVE_SAMPLES             2
#define GESTURE_LIFT_FINAL_TILT_MAX_DEG           10.0f
#define GESTURE_FINAL_HOLD_MS                     500
#define GESTURE_FINAL_HOLD_TIMEOUT_MS            5000
#define GESTURE_FINAL_STILL_RMS_MS2              2.0f
#define GESTURE_FINAL_RMS_WINDOW_SAMPLES             4
#define GESTURE_SEQUENCE_TIMEOUT_MS              9000
#define GESTURE_RETRIGGER_BLOCK_MS           1200
#define GESTURE_GRAVITY_LP_TAU_S                 0.30f
#define GESTURE_QUIET_ACCEL_MS2                  3.0f
/* Palm-down shake: gravity-orthogonal linear accel, 500 ms window. */
#define GESTURE_SHAKE_WINDOW_SAMPLES                20
#define GESTURE_SHAKE_PTP_MIN_MS2                 5.0f
#define GESTURE_SHAKE_MEAN_RATIO_MAX              0.4f
#define GESTURE_SHAKE_AXIS_MIN_MS2                2.0f

/* BLE gesture diagnostics: event 0x30, stage/reason + three float values. */
#define GESTURE_DIAG_OUTBOUND_START           0x01
#define GESTURE_DIAG_OUTBOUND_READY           0x02
#define GESTURE_DIAG_FINAL_HOLD_START         0x07
#define GESTURE_DIAG_FINAL_READY              0x08
#define GESTURE_DIAG_MATCH                    0x09
#define GESTURE_DIAG_STOP_PALM_UP             0x0C
#define GESTURE_DIAG_FINAL_SAMPLE             0x21
#define GESTURE_DIAG_WAIT_REJECT               0x10
#define GESTURE_DIAG_RESET                     0x80
#define GESTURE_DEBUG_FINAL_PERIOD_MS           100

#define GESTURE_DIAG_REASON_NONE               0x00
#define GESTURE_DIAG_REASON_QUIET_NOT_READY    0x01
#define GESTURE_DIAG_REASON_START_NOT_PALM_UP  0x02
#define GESTURE_DIAG_REASON_OUTBOUND_RATE_LOW  0x03
#define GESTURE_DIAG_REASON_OUTBOUND_TIMEOUT   0x11
#define GESTURE_DIAG_REASON_INCOMPLETE_OUTBOUND 0x12
#define GESTURE_DIAG_REASON_FINAL_HOLD_INTERRUPTED 0x1a
#define GESTURE_DIAG_REASON_FINAL_HOLD_TIMEOUT 0x1b
#define GESTURE_DIAG_REASON_SEQUENCE_TIMEOUT   0x1c
#define GESTURE_DIAG_REASON_FINAL_ACCEL_MISSING 0x1d
#define GESTURE_DIAG_REASON_FINAL_BRAKE_MISSING 0x1e
#define GESTURE_DIAG_REASON_FINAL_BRAKE_RATIO_LOW 0x1f
#define GESTURE_DIAG_REASON_FINAL_TILT_UNSTABLE 0x20
#define GESTURE_DIAG_REASON_FINAL_PULSE_DURATION_INVALID 0x21
#define GESTURE_DIAG_REASON_SHAKE_NOT_OSCILLATORY 0x22
#define GESTURE_DIAG_REASON_LIFT_PALM_STILL_UP     0x23

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

#define MAX_CONNS 2
static struct bt_conn *connections[MAX_CONNS];
static int primary_idx = -1;

static struct bt_conn *get_primary_conn(void) {
    return (primary_idx >= 0) ? connections[primary_idx] : NULL;
}

static int conn_index(struct bt_conn *conn) {
    for (int i = 0; i < MAX_CONNS; i++)
        if (connections[i] == conn) return i;
    return -1;
}

static int free_slot(void) {
    for (int i = 0; i < MAX_CONNS; i++)
        if (connections[i] == NULL) return i;
    return -1;
}

static int active_count(void) {
    int n = 0;
    for (int i = 0; i < MAX_CONNS; i++) if (connections[i]) n++;
    return n;
}

/* Audio state */
static uint8_t tx_packet[512];
static uint8_t seq_num;
static volatile bool is_recording;
static volatile bool recording_requested;
static volatile bool stop_requested;
static volatile bool capture_enabled;
static volatile bool capture_fatal;
K_SEM_DEFINE(audio_notify_slots, AUDIO_NOTIFY_IN_FLIGHT, AUDIO_NOTIFY_IN_FLIGHT);

struct audio_pcm_frame {
    uint16_t nbytes;
    uint8_t pcm[AUDIO_FRAME_BYTES];
};

K_MSGQ_DEFINE(audio_frame_q, sizeof(struct audio_pcm_frame),
              AUDIO_FRAME_QUEUE_LEN, 4);

struct audio_session_stats {
    uint32_t frames_captured;
    uint32_t frames_sent;
    uint32_t bytes_sent;
    uint32_t notifies_sent;
    uint32_t notify_wait_count;
    uint32_t notify_wait_max_ms;
    uint32_t notify_retry_count;
    uint32_t dmic_read_errors;
    uint32_t last_dmic_errno;
    uint32_t dmic_overruns;
    uint32_t queue_high_watermark;
    int64_t session_start_ms;
    int64_t dmic_stop_ms;
};

static struct audio_session_stats audio_stats;

/* Audio control/sender thread + dedicated DMIC capture thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static K_THREAD_STACK_DEFINE(audio_capture_stack, 2048);
static struct k_thread audio_thread_data;
static struct k_thread audio_capture_thread_data;

/* Serial command thread */
static K_THREAD_STACK_DEFINE(serial_stack, 1024);
static struct k_thread serial_thread_data;

/* Deferred advertising restart — calling bt_le_adv_start() directly from
 * within a BT callback can fail silently on Nordic NCS because the BT stack
 * lock may already be held.  Schedule via a work item instead. */
static struct k_work_delayable adv_work;

/* Deferred BLE connection parameter update.
 * After primary/secondary role changes we schedule this work item so that
 * bt_conn_le_param_update() is called from the system workqueue (not from
 * inside a BT/GATT callback where the BT lock is already held). */
static struct k_work_delayable conn_param_work;

static void conn_param_work_handler(struct k_work *work)
{
    /* Fast params for primary: 7.5–15 ms interval → max audio throughput */
    static const struct bt_le_conn_param fast_param = {
        .interval_min = 6,    /* 6 × 1.25 ms = 7.5 ms */
        .interval_max = 12,   /* 12 × 1.25 ms = 15 ms  */
        .latency      = 0,
        .timeout      = 400,
    };
    /* Slow params for secondary: 200–500 ms → frees radio for primary */
    static const struct bt_le_conn_param slow_param = {
        .interval_min = 160,  /* 200 ms */
        .interval_max = 400,  /* 500 ms */
        .latency      = 0,
        .timeout      = 400,
    };

    for (int i = 0; i < MAX_CONNS; i++) {
        if (!connections[i]) continue;
        const struct bt_le_conn_param *p = (i == primary_idx) ? &fast_param : &slow_param;
        int ret = bt_conn_le_param_update(connections[i], p);
        if (ret && ret != -EALREADY) {
            printk(">>> conn_param_update[%d] failed: %d\n", i, ret);
        } else {
            printk(">>> conn_param_update[%d]: %s\n", i,
                   (i == primary_idx) ? "fast(7.5ms)" : "slow(200ms)");
        }
    }
}

/* IMU device */
static const struct device *const imu     = DEVICE_DT_GET(IMU_NODE);

/* Gesture state */
typedef enum {
    GESTURE_WAITING,
    GESTURE_OUTBOUND,
    GESTURE_HOLDING_FINAL,
} gesture_phase_t;

typedef enum {
    GESTURE_LIFT_WAIT_ACCEL,
    GESTURE_LIFT_WAIT_BRAKE,
    GESTURE_LIFT_WAIT_HOLD,
} gesture_lift_stage_t;

static int64_t motion_active_start_ms;
static gesture_phase_t gesture_phase;
static int64_t gesture_sequence_start_ms;
static int64_t gesture_phase_start_ms;
static int64_t gesture_final_since_ms;
static int64_t gesture_last_sample_ms;
static int64_t gesture_quiet_since_ms;
static int64_t gesture_armed_until_ms;
static int64_t gesture_block_until_ms;
static int64_t gesture_diag_last_report_ms;
static float gesture_quiet_accel_x, gesture_quiet_accel_y, gesture_quiet_accel_z;
static bool gesture_quiet_accel_valid;
static float gesture_start_accel_x, gesture_start_accel_y, gesture_start_accel_z;
static float gesture_z_window_min, gesture_z_window_max;
static int64_t gesture_z_window_start_ms;
static float gesture_pronation_phi_deg;
static float gesture_pronation_peak_deg;
static float gesture_pronation_peak_tilt_deg;
static float gesture_pronation_ref_phi_deg;
static float gesture_armed_az;
static float gesture_armed_z_ratio;
static float gesture_armed_gx, gesture_armed_gy, gesture_armed_gz;
static bool gesture_outbound_qualified;
static bool gesture_shake_lp_ref_valid;
static float gesture_shake_lp_ref_x, gesture_shake_lp_ref_y, gesture_shake_lp_ref_z;
static float gesture_shake_lp_ref_phi;
static float gesture_shake_lp_ref_z_ratio;
static float gesture_gravity_lp_x, gesture_gravity_lp_y, gesture_gravity_lp_z;
static float gesture_linear_world_x, gesture_linear_world_y;
static float gesture_linear_world_z;
static float gesture_linear_accel_norm_ms2;
static gesture_lift_stage_t gesture_lift_stage;
static float gesture_lift_hold_axis_x;
static float gesture_lift_hold_axis_y;
static float gesture_lift_hold_axis_z;
static float gesture_lift_pos_impulse_ms;
static float gesture_lift_neg_impulse_ms;
static float gesture_lift_net_impulse_ms;
static float gesture_lift_final_tilt_deg;
static float gesture_lift_max_hold_tilt_deg;
static bool gesture_lift_hold_axis_valid;
static bool gesture_lift_pose_failed;
static int64_t gesture_lift_event_start_ms;
static uint8_t gesture_lift_accel_samples;
static uint8_t gesture_lift_brake_samples;
static int64_t gesture_debug_final_last_ms;
static float gesture_accel_history_ms2[GESTURE_FINAL_RMS_WINDOW_SAMPLES];
static uint8_t gesture_accel_history_index;
static uint8_t gesture_accel_history_count;
/* Palm-down pose frozen at recording start; stop on a loose palm-up flip. */
static bool recording_stop_ref_valid;
static float recording_stop_ref_phi_deg;
static float recording_stop_ref_az;
static float recording_stop_ref_z_ratio;
static float recording_stop_ref_gx, recording_stop_ref_gy, recording_stop_ref_gz;
static float recording_stop_peak_phi_deg;
static float recording_stop_peak_tilt_deg;
static float gesture_shake_axis_x, gesture_shake_axis_y, gesture_shake_axis_z;
static bool gesture_shake_axis_valid;
static float gesture_shake_signed[GESTURE_SHAKE_WINDOW_SAMPLES];
static uint8_t gesture_shake_index, gesture_shake_count;
static float gesture_shake_last_ptp_ms2;
static float gesture_shake_last_mean_ms2;

/* Motion detection state */
static atomic_t detected_motion_count;
static uint32_t last_motion_time_ms;
static bool baseline_valid;
static double baseline_x, baseline_y, baseline_z;
static uint8_t calibration_count, active_high_count, settle_count;
static bool motion_active;
static bool inhibit_next_settle;   /* 録音中断直後のsettleをスキップ */
static bool light_sleep_active;    /* true = IMUポーリングを低速化して省電力 */
static int64_t last_activity_ms;   /* motion_active/settled/recording_stop の最終時刻 */
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

/* Battery ADC (P0.31 = AIN7) + Enable GPIO (P0.14) */
static const struct adc_dt_spec adc_bat =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct gpio_dt_spec bat_en =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), bat_read_enable_gpios);
static int16_t bat_sample_buf;
static struct adc_sequence bat_sequence = {
    .buffer      = &bat_sample_buf,
    .buffer_size = sizeof(bat_sample_buf),
};
static bool    bat_adc_ready;
static uint8_t bat_last_pct      = 50;  /* last ADC-measured % (initial: 50% fallback) */
static uint32_t bat_charge_start_ms;    /* k_uptime when USB was connected */
static uint8_t  bat_charge_start_pct;   /* battery % when USB was connected */
static bool     bat_vbus_prev;          /* previous VBUS state for edge detection */

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
    LED_IDLE,
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
    case LED_IDLE:
        rgb_set(false, false, false); /* Off */
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

static void led_sync_runtime_state(void)
{
    led_state_t desired = LED_IDLE;

    if (current_led_state == LED_BOOT || current_led_state == LED_ERROR) {
        return;
    }

    if (is_recording) {
        desired = LED_RECORDING;
    }

    if (current_led_state != desired) {
        led_set_state(desired);
    }
}

/* Called from main loop every MAIN_LOOP_INTERVAL_MS to handle blink patterns. */
static void led_tick(void)
{
    int64_t now = k_uptime_get();

    if (current_led_state == LED_ERROR) {
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
 * Battery Monitor
 * ============================================================================ */

static int battery_init(void)
{
    if (!adc_is_ready_dt(&adc_bat)) {
        LOG_WRN("Battery ADC not ready");
        return -ENODEV;
    }
    int err = adc_channel_setup_dt(&adc_bat);
    if (err < 0) {
        LOG_ERR("Battery ADC channel setup failed: %d", err);
        return err;
    }
    /* Configure P0.14 as output (battery measurement enable) */
    if (gpio_is_ready_dt(&bat_en)) {
        gpio_pin_configure_dt(&bat_en, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_WRN("Battery enable GPIO not ready");
    }
    bat_adc_ready = true;
    LOG_INF("Battery ADC ready (AIN7/P0.31, enable P0.14)");
    return 0;
}

static int battery_get_millivolt(void)
{
    int err;
    int32_t val_mv;

    /* P0.14 must remain LOW — HIGH injects 3.3V into P0.31 and saturates ADC */

    err = adc_sequence_init_dt(&adc_bat, &bat_sequence);
    if (err < 0) { return err; }

    err = adc_read(adc_bat.dev, &bat_sequence);
    if (err < 0) { return err; }

    val_mv = bat_sample_buf;
    err = adc_raw_to_millivolts_dt(&adc_bat, &val_mv);
    if (err < 0) { return err; }

    /*
     * Correction factor ×3: XIAO nRF52840 divider is 2MΩ:1MΩ (not 1:1).
     * Verified empirically: pin_mv=1398mV × 3 = 4194mV ≈ 4200mV (USB charge).
     */
    return (int)(val_mv * 3);
}

static uint8_t battery_millivolt_to_percent(int mv)
{
    /*
     * OCV (Open Circuit Voltage) based LUT for BQ25101 on XIAO nRF52840 Sense.
     * The charger terminates at 4.2V CV / 5mA (ITERM=10% of 50mA default).
     * After termination and USB removal, the battery OCV settles at ~4.08-4.15V
     * (not 4.2V, which is only the charger-applied terminal voltage).
     * This LUT uses resting OCV values so that a fully charged battery reads ~100%.
     */
    static const struct { int mv; int pct; } lut[] = {
        { 4150, 100 }, { 4050, 90 }, { 3950, 80 },
        { 3850,  70 }, { 3750, 60 }, { 3650, 50 },
        { 3550,  40 }, { 3450, 30 }, { 3350, 20 },
        { 3250,  10 }, { 3000,  0 },
    };
    if (mv >= lut[0].mv) { return 100; }
    for (int i = 0; i < (int)ARRAY_SIZE(lut) - 1; i++) {
        if (mv >= lut[i + 1].mv) {
            int mv_range  = lut[i].mv  - lut[i + 1].mv;
            int pct_range = lut[i].pct - lut[i + 1].pct;
            return (uint8_t)(lut[i + 1].pct +
                   (mv - lut[i + 1].mv) * pct_range / mv_range);
        }
    }
    return 0;
}

static void battery_update(void)
{
    if (!bat_adc_ready) { return; }

    bool vbus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);

    /* USBが新たに接続された → 充電開始を記録 */
    if (vbus && !bat_vbus_prev) {
        bat_charge_start_ms  = k_uptime_get_32();
        bat_charge_start_pct = bat_last_pct;
        LOG_INF("Battery: USB connected, charging from %d%%", bat_charge_start_pct);
    }
    bat_vbus_prev = vbus;

    if (vbus) {
        /* 充電中：経過時間から残量を線形推定 */
        uint32_t elapsed_min = (k_uptime_get_32() - bat_charge_start_ms) / 60000U;
        /* 充電レート [%/min] = charge_current_mA * 100 / (capacity_mAh * 60) */
        uint32_t added_pct = (elapsed_min * BATTERY_CHARGE_CURRENT_MA * 100U)
                             / (BATTERY_CAPACITY_MAH * 60U);
        uint8_t est_pct = (uint8_t)MIN(bat_charge_start_pct + added_pct,
                                       BATTERY_CHARGE_MAX_EST_PCT);
        LOG_INF("Battery: charging ~%d%% (est, %u min)", est_pct, elapsed_min);
        printk("Battery: charging ~%d%% (est, %u min)\n", est_pct, elapsed_min);
        bt_bas_set_battery_level(est_pct);
        return;
    }

    /* USB切断中：ADCで実測 */
    int mv = battery_get_millivolt();
    if (mv <= 0) {
        LOG_WRN("Battery read failed: %d", mv);
        return;
    }
    uint8_t pct = battery_millivolt_to_percent(mv);
    bat_last_pct = pct;
    LOG_INF("Battery: %d mV (%d%%)", mv, pct);
    printk("Battery: %d mV (%d%%)\n", mv, pct);
    bt_bas_set_battery_level(pct);
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
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z);
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance);
static void send_gesture_diag(uint8_t stage, uint8_t reason,
                              float value1, float value2, float value3);
static float deg_diff(float a, float b);

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
    /* Suppress baseline drift when the board is vibrating (e.g., in a car).
     * Only update when the total acceleration change is very small,
     * indicating the device is truly still. */
    double step = abs_double(x - baseline_x) + abs_double(y - baseline_y) +
                  abs_double(z - baseline_z);
    if (step < 0.5) {
        baseline_x += (x - baseline_x) * BASELINE_ALPHA;
        baseline_y += (y - baseline_y) * BASELINE_ALPHA;
        baseline_z += (z - baseline_z) * BASELINE_ALPHA;
    }
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
    /* Use z-axis only for step calculation.  The arm-lift gesture is
     * predominantly a change in the gravity (z) direction, while
     * horizontal vibrations (e.g., from a car) are ignored. */
    double d = abs_double(z - previous_accel_z);
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
    if (calibration_count > 0) {
        double accel_dx = x - baseline_x;
        double accel_dy = y - baseline_y;
        double accel_dz = z - baseline_z;
        double accel_span = sqrt(accel_dx * accel_dx +
                                 accel_dy * accel_dy +
                                 accel_dz * accel_dz);

        /* Restart the window until all calibration samples describe one stable pose. */
        if (accel_span > (double)CALIBRATION_ACCEL_SPAN_MAX_MS2) {
            calibration_count = 0;
        }
    }

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
        gesture_quiet_accel_x = (float)baseline_x;
        gesture_quiet_accel_y = (float)baseline_y;
        gesture_quiet_accel_z = (float)baseline_z;
        gesture_quiet_accel_valid = true;
        gesture_quiet_since_ms = k_uptime_get();
        gesture_last_sample_ms = gesture_quiet_since_ms;
        reset_step_window();
        last_report_ms = k_uptime_get();
        last_motion_time_ms = (uint32_t)k_uptime_get();
    }
}

static int configure_motion_detection(void)
{
    struct sensor_value accel_odr = {ACCEL_ODR_HZ, 0};
    struct sensor_value threshold = {1, 500000};
    struct sensor_value duration = {MOTION_DURATION_SAMPLES, 0};
    int ret;

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr);
    if (ret < 0) { LOG_ERR("ODR set failed: %d", ret); return ret; }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &threshold);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_TH not supported");
    else if (ret < 0) { LOG_ERR("threshold set failed: %d", ret); return ret; }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &duration);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_DUR not supported");
    else if (ret < 0) { LOG_ERR("duration set failed: %d", ret); return ret; }

    LOG_INF("Motion detection ready: accel=%d Hz, sample=%d ms",
            ACCEL_ODR_HZ, MOTION_SAMPLE_INTERVAL_MS);
    LOG_INF("Calibrating for %.1f s; keep the board still",
            (double)(CALIBRATION_SAMPLES * MOTION_SAMPLE_INTERVAL_MS) / 1000.0);
    return 0;
}

/* ============================================================================
 * Gesture Detection
 * ============================================================================ */

static float vector_norm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void reset_gesture_motion(void)
{
    gesture_gravity_lp_x = gesture_start_accel_x;
    gesture_gravity_lp_y = gesture_start_accel_y;
    gesture_gravity_lp_z = gesture_start_accel_z;
    gesture_linear_accel_norm_ms2 = 0.0f;
    gesture_linear_world_x = 0.0f;
    gesture_linear_world_y = 0.0f;
    gesture_linear_world_z = 0.0f;
    gesture_lift_stage = GESTURE_LIFT_WAIT_ACCEL;
    gesture_lift_hold_axis_x = 0.0f;
    gesture_lift_hold_axis_y = 0.0f;
    gesture_lift_hold_axis_z = 0.0f;
    gesture_lift_pos_impulse_ms = 0.0f;
    gesture_lift_neg_impulse_ms = 0.0f;
    gesture_lift_net_impulse_ms = 0.0f;
    gesture_lift_final_tilt_deg = 0.0f;
    gesture_lift_max_hold_tilt_deg = 0.0f;
    gesture_lift_hold_axis_valid = false;
    gesture_lift_pose_failed = false;
    gesture_lift_event_start_ms = 0;
    gesture_lift_accel_samples = 0;
    gesture_lift_brake_samples = 0;
    gesture_debug_final_last_ms = 0;
    memset(gesture_accel_history_ms2, 0, sizeof(gesture_accel_history_ms2));
    gesture_accel_history_index = 0;
    gesture_accel_history_count = 0;
}

/* Low-pass raw acceleration to estimate gravity (in the body frame).  With the
 * gyro disabled this is the only orientation information available.  The LP
 * removes vehicle accelerations that are slow compared to the lift motion. */
static void update_gravity_lp(float ax, float ay, float az, float dt_s)
{
    float alpha = dt_s / (GESTURE_GRAVITY_LP_TAU_S + dt_s);
    gesture_gravity_lp_x += (ax - gesture_gravity_lp_x) * alpha;
    gesture_gravity_lp_y += (ay - gesture_gravity_lp_y) * alpha;
    gesture_gravity_lp_z += (az - gesture_gravity_lp_z) * alpha;

    float linear_x = ax - gesture_gravity_lp_x;
    float linear_y = ay - gesture_gravity_lp_y;
    float linear_z = az - gesture_gravity_lp_z;
    gesture_linear_world_x = linear_x;
    gesture_linear_world_y = linear_y;
    gesture_linear_world_z = linear_z;
    gesture_linear_accel_norm_ms2 = vector_norm3(linear_x, linear_y, linear_z);
}

static void reset_shake_window(void)
{
    memset(gesture_shake_signed, 0, sizeof(gesture_shake_signed));
    gesture_shake_index = 0;
    gesture_shake_count = 0;
    gesture_shake_axis_valid = false;
    gesture_shake_axis_x = 0.0f;
    gesture_shake_axis_y = 0.0f;
    gesture_shake_axis_z = 0.0f;
    gesture_shake_last_ptp_ms2 = 0.0f;
    gesture_shake_last_mean_ms2 = 0.0f;
    gesture_shake_lp_ref_valid = false;
    gesture_shake_lp_ref_x = 0.0f;
    gesture_shake_lp_ref_y = 0.0f;
    gesture_shake_lp_ref_z = 0.0f;
    gesture_shake_lp_ref_phi = 0.0f;
    gesture_shake_lp_ref_z_ratio = 0.0f;
}

static void snapshot_shake_gravity_reference(void)
{
    float gx = gesture_gravity_lp_x;
    float gy = gesture_gravity_lp_y;
    float gz = gesture_gravity_lp_z;
    float n = vector_norm3(gx, gy, gz);

    if (n <= 0.1f) {
        gesture_shake_lp_ref_valid = false;
        return;
    }
    gesture_shake_lp_ref_x = gx;
    gesture_shake_lp_ref_y = gy;
    gesture_shake_lp_ref_z = gz;
    gesture_shake_lp_ref_phi = atan2f(-gx, gz) * (float)GESTURE_RAD_TO_DEG;
    gesture_shake_lp_ref_z_ratio = gz / n;
    gesture_shake_lp_ref_valid = true;
}

static bool gesture_board_flat(float z_gravity_ratio)
{
    return fabsf(z_gravity_ratio) >= GESTURE_START_PALM_UP_Z_MIN_RATIO;
}

static bool push_shake_sample(float dt_s)
{
    float g_norm = vector_norm3(gesture_gravity_lp_x,
                                gesture_gravity_lp_y,
                                gesture_gravity_lp_z);
    if (g_norm <= 0.1f) {
        reset_shake_window();
        return false;
    }

    float g_hat_x = gesture_gravity_lp_x / g_norm;
    float g_hat_y = gesture_gravity_lp_y / g_norm;
    float g_hat_z = gesture_gravity_lp_z / g_norm;
    float a_vert = gesture_linear_world_x * g_hat_x +
                   gesture_linear_world_y * g_hat_y +
                   gesture_linear_world_z * g_hat_z;
    float a_hx = gesture_linear_world_x - a_vert * g_hat_x;
    float a_hy = gesture_linear_world_y - a_vert * g_hat_y;
    float a_hz = gesture_linear_world_z - a_vert * g_hat_z;
    float a_h_norm = vector_norm3(a_hx, a_hy, a_hz);

    if (!gesture_shake_axis_valid && a_h_norm >= GESTURE_SHAKE_AXIS_MIN_MS2) {
        gesture_shake_axis_x = a_hx / a_h_norm;
        gesture_shake_axis_y = a_hy / a_h_norm;
        gesture_shake_axis_z = a_hz / a_h_norm;
        gesture_shake_axis_valid = true;
    }
    if (!gesture_shake_axis_valid) {
        return false;
    }

    float signed_h = a_hx * gesture_shake_axis_x +
                     a_hy * gesture_shake_axis_y +
                     a_hz * gesture_shake_axis_z;
    gesture_shake_signed[gesture_shake_index] = signed_h;
    gesture_shake_index =
        (uint8_t)((gesture_shake_index + 1U) % GESTURE_SHAKE_WINDOW_SAMPLES);
    if (gesture_shake_count < GESTURE_SHAKE_WINDOW_SAMPLES) {
        gesture_shake_count++;
        if (gesture_shake_count == 1U) {
            snapshot_shake_gravity_reference();
        }
    }
    if (gesture_shake_count < GESTURE_SHAKE_WINDOW_SAMPLES) {
        return false;
    }

    float min_s = gesture_shake_signed[0];
    float max_s = gesture_shake_signed[0];
    float sum_s = 0.0f;
    for (uint8_t i = 0; i < GESTURE_SHAKE_WINDOW_SAMPLES; i++) {
        float s = gesture_shake_signed[i];
        if (s < min_s) {
            min_s = s;
        }
        if (s > max_s) {
            max_s = s;
        }
        sum_s += s;
    }

    float ptp = max_s - min_s;
    float mean = sum_s / (float)GESTURE_SHAKE_WINDOW_SAMPLES;
    gesture_shake_last_ptp_ms2 = ptp;
    gesture_shake_last_mean_ms2 = mean;
    ARG_UNUSED(dt_s);
    return ptp >= GESTURE_SHAKE_PTP_MIN_MS2 &&
           fabsf(mean) < GESTURE_SHAKE_MEAN_RATIO_MAX * ptp;
}

static void reset_recording_stop_state(void)
{
    recording_stop_ref_valid = false;
    recording_stop_ref_phi_deg = 0.0f;
    recording_stop_ref_az = 0.0f;
    recording_stop_ref_z_ratio = 0.0f;
    recording_stop_ref_gx = 0.0f;
    recording_stop_ref_gy = 0.0f;
    recording_stop_ref_gz = 0.0f;
    recording_stop_peak_phi_deg = 0.0f;
    recording_stop_peak_tilt_deg = 0.0f;
}

static void capture_recording_stop_reference(void)
{
    float gx = gesture_gravity_lp_x;
    float gy = gesture_gravity_lp_y;
    float gz = gesture_gravity_lp_z;
    float n = vector_norm3(gx, gy, gz);

    if (n <= 0.1f) {
        return;
    }

    recording_stop_ref_phi_deg = atan2f(-gx, gz) * (float)GESTURE_RAD_TO_DEG;
    recording_stop_ref_az = gz;
    recording_stop_ref_z_ratio = gz / n;
    recording_stop_ref_gx = gx / n;
    recording_stop_ref_gy = gy / n;
    recording_stop_ref_gz = gz / n;
    recording_stop_peak_phi_deg = 0.0f;
    recording_stop_peak_tilt_deg = 0.0f;
    recording_stop_ref_valid = true;
}

static float recording_stop_tilt_deg(float ax, float ay, float az)
{
    float n = vector_norm3(ax, ay, az);
    float rn = vector_norm3(recording_stop_ref_gx,
                            recording_stop_ref_gy,
                            recording_stop_ref_gz);

    if (n <= 0.1f || rn <= 0.1f) {
        return 0.0f;
    }
    float c = (ax * recording_stop_ref_gx +
               ay * recording_stop_ref_gy +
               az * recording_stop_ref_gz) / (n * rn);
    c = CLAMP(c, -1.0f, 1.0f);
    return acosf(c) * (float)GESTURE_RAD_TO_DEG;
}

static bool recording_stop_palm_up_detected(float phi_deg, float tilt_deg,
                                           float z_gravity_ratio, float az)
{
    return phi_deg >= GESTURE_OUTBOUND_MIN_DEG ||
           tilt_deg >= GESTURE_OUTBOUND_TILT_MIN_DEG ||
           fabsf(z_gravity_ratio - recording_stop_ref_z_ratio) >=
               GESTURE_OUTBOUND_Z_RATIO_DONE ||
           ((az * recording_stop_ref_az < 0.0f) &&
            fabsf(az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2 &&
            fabsf(recording_stop_ref_az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2);
}

static void request_recording_stop_palm_up(int64_t now, float phi_deg,
                                          float tilt_deg, float z_delta)
{
    printk(">>> Palm-up while recording → stop phi=%.1f tilt=%.1f z_d=%.2f\n",
           (double)phi_deg, (double)tilt_deg, (double)z_delta);
    send_gesture_diag(GESTURE_DIAG_STOP_PALM_UP, GESTURE_DIAG_REASON_NONE,
                      phi_deg, tilt_deg, z_delta);
    stop_requested = true;
    inhibit_next_settle = true;
    gesture_block_until_ms = now + GESTURE_RETRIGGER_BLOCK_MS;
    reset_recording_stop_state();
}

static void process_recording_stop_sample(float ax, float ay, float az,
                                         int64_t now)
{
    float accel_norm = vector_norm3(ax, ay, az);
    float z_gravity_ratio = accel_norm > 0.1f ? az / accel_norm : 0.0f;
    float phi = atan2f(-ax, az) * (float)GESTURE_RAD_TO_DEG;
    float phi_change = fabsf(deg_diff(phi, recording_stop_ref_phi_deg));
    float tilt_deg = recording_stop_tilt_deg(gesture_gravity_lp_x,
                                            gesture_gravity_lp_y,
                                            gesture_gravity_lp_z);

    if (stop_requested) {
        return;
    }
    if (now < gesture_block_until_ms) {
        return;
    }
    if (!recording_stop_ref_valid) {
        capture_recording_stop_reference();
        if (!recording_stop_ref_valid) {
            return;
        }
        phi_change = 0.0f;
        tilt_deg = 0.0f;
    }

    if (phi_change > recording_stop_peak_phi_deg) {
        recording_stop_peak_phi_deg = phi_change;
    }
    if (tilt_deg > recording_stop_peak_tilt_deg) {
        recording_stop_peak_tilt_deg = tilt_deg;
    }

    if (accel_norm < GESTURE_PRONATION_GRAVITY_MIN_MS2 ||
        accel_norm > GESTURE_PRONATION_GRAVITY_MAX_MS2) {
        return;
    }
    if (!recording_stop_palm_up_detected(recording_stop_peak_phi_deg,
                                         recording_stop_peak_tilt_deg,
                                         z_gravity_ratio, az)) {
        return;
    }

    request_recording_stop_palm_up(
        now, recording_stop_peak_phi_deg, recording_stop_peak_tilt_deg,
        fabsf(z_gravity_ratio - recording_stop_ref_z_ratio));
}

static float deg_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f) {
        d -= 360.0f;
    }
    while (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

/* RMS of a fully populated short linear-acceleration window. */
static float gesture_accel_rms(void)
{
    uint8_t n = gesture_accel_history_count;
    if (n < GESTURE_FINAL_RMS_WINDOW_SAMPLES) {
        return INFINITY;
    }
    float sq = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float a = gesture_accel_history_ms2[i];
        sq += a * a;
    }
    return sqrtf(sq / (float)n);
}

static void push_accel_history(float a)
{
    gesture_accel_history_ms2[gesture_accel_history_index] = a;
    gesture_accel_history_index =
        (gesture_accel_history_index + 1U) %
        GESTURE_FINAL_RMS_WINDOW_SAMPLES;
    if (gesture_accel_history_count < GESTURE_FINAL_RMS_WINDOW_SAMPLES) {
        gesture_accel_history_count++;
    }
}

static void clear_accel_history(void)
{
    memset(gesture_accel_history_ms2, 0, sizeof(gesture_accel_history_ms2));
    gesture_accel_history_index = 0;
    gesture_accel_history_count = 0;
}

static void retry_lift_pulse(void)
{
    gesture_lift_stage = GESTURE_LIFT_WAIT_ACCEL;
    gesture_lift_pos_impulse_ms = 0.0f;
    gesture_lift_neg_impulse_ms = 0.0f;
    gesture_lift_net_impulse_ms = 0.0f;
    gesture_lift_final_tilt_deg = 0.0f;
    gesture_lift_hold_axis_valid = false;
    gesture_lift_pose_failed = false;
    gesture_lift_event_start_ms = 0;
    gesture_lift_accel_samples = 0;
    gesture_lift_brake_samples = 0;
    gesture_final_since_ms = 0;
    clear_accel_history();
}

static float gesture_tilt_from_hold_axis_deg(void)
{
    if (!gesture_lift_hold_axis_valid) {
        return 0.0f;
    }

    float g_norm = vector_norm3(gesture_gravity_lp_x,
                                gesture_gravity_lp_y,
                                gesture_gravity_lp_z);
    if (g_norm <= 0.1f) {
        return 180.0f;
    }

    float cos_tilt = (gesture_gravity_lp_x * gesture_lift_hold_axis_x +
                      gesture_gravity_lp_y * gesture_lift_hold_axis_y +
                      gesture_gravity_lp_z * gesture_lift_hold_axis_z) /
                     g_norm;
    cos_tilt = CLAMP(cos_tilt, -1.0f, 1.0f);
    return acosf(cos_tilt) * (float)GESTURE_RAD_TO_DEG;
}

static void reset_gesture_sequence(void)
{
    gesture_phase = GESTURE_WAITING;
    gesture_sequence_start_ms = 0;
    gesture_phase_start_ms = 0;
    gesture_final_since_ms = 0;
    gesture_z_window_min = 0.0f;
    gesture_z_window_max = 0.0f;
    gesture_z_window_start_ms = 0;
    gesture_pronation_phi_deg = 0.0f;
    gesture_pronation_peak_deg = 0.0f;
    gesture_pronation_peak_tilt_deg = 0.0f;
    gesture_pronation_ref_phi_deg = 0.0f;
    gesture_armed_az = 0.0f;
    gesture_armed_z_ratio = 0.0f;
    gesture_armed_gx = 0.0f;
    gesture_armed_gy = 0.0f;
    gesture_armed_gz = 0.0f;
    gesture_outbound_qualified = false;
    reset_gesture_motion();
    reset_shake_window();
    gesture_armed_until_ms = 0;
}

static void update_quiet_accel_reference(float x, float y, float z)
{
    if (!gesture_quiet_accel_valid) {
        gesture_quiet_accel_x = x;
        gesture_quiet_accel_y = y;
        gesture_quiet_accel_z = z;
        gesture_quiet_accel_valid = true;
        return;
    }

    /* Low-pass only while accelerometrically quiet; vehicle vibration is averaged out. */
    gesture_quiet_accel_x += (x - gesture_quiet_accel_x) * 0.1f;
    gesture_quiet_accel_y += (y - gesture_quiet_accel_y) * 0.1f;
    gesture_quiet_accel_z += (z - gesture_quiet_accel_z) * 0.1f;
}

static void arm_pronation_reference(float phi, float ax, float ay, float az,
                                    float z_gravity_ratio, int64_t now)
{
    float n = vector_norm3(ax, ay, az);

    gesture_armed_until_ms = now + GESTURE_START_ARM_MS;
    gesture_pronation_ref_phi_deg = phi;
    gesture_armed_az = az;
    gesture_armed_z_ratio = z_gravity_ratio;
    if (n > 0.1f) {
        gesture_armed_gx = ax / n;
        gesture_armed_gy = ay / n;
        gesture_armed_gz = az / n;
    } else {
        gesture_armed_gx = 0.0f;
        gesture_armed_gy = 0.0f;
        gesture_armed_gz = 0.0f;
    }
    gesture_z_window_min = az;
    gesture_z_window_max = az;
    gesture_z_window_start_ms = now;
}

static void arm_from_shake_gravity_reference(int64_t now)
{
    if (gesture_shake_lp_ref_valid) {
        arm_pronation_reference(gesture_shake_lp_ref_phi,
                                gesture_shake_lp_ref_x,
                                gesture_shake_lp_ref_y,
                                gesture_shake_lp_ref_z,
                                gesture_shake_lp_ref_z_ratio,
                                now);
        return;
    }

    float gx = gesture_gravity_lp_x;
    float gy = gesture_gravity_lp_y;
    float gz = gesture_gravity_lp_z;
    float n = vector_norm3(gx, gy, gz);
    float zr = n > 0.1f ? gz / n : 0.0f;
    float phi = atan2f(-gx, gz) * (float)GESTURE_RAD_TO_DEG;

    arm_pronation_reference(phi, gx, gy, gz, zr, now);
}

static float gesture_z_ratio_delta(float z_gravity_ratio)
{
    return fabsf(z_gravity_ratio - gesture_armed_z_ratio);
}

static bool gesture_z_sign_flipped(float az)
{
    return (az * gesture_armed_az < 0.0f) &&
           fabsf(az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2 &&
           fabsf(gesture_armed_az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2;
}

static float gesture_gravity_tilt_deg(float ax, float ay, float az)
{
    float n = vector_norm3(ax, ay, az);
    float rn = vector_norm3(gesture_armed_gx, gesture_armed_gy, gesture_armed_gz);

    if (n <= 0.1f || rn <= 0.1f) {
        return 0.0f;
    }
    float c = (ax * gesture_armed_gx +
               ay * gesture_armed_gy +
               az * gesture_armed_gz) / (n * rn);
    c = CLAMP(c, -1.0f, 1.0f);
    return acosf(c) * (float)GESTURE_RAD_TO_DEG;
}

static bool gesture_outbound_palm_up_detected(float phi_deg,
                                             float tilt_deg,
                                             float z_gravity_ratio,
                                             float az)
{
    return phi_deg >= GESTURE_OUTBOUND_MIN_DEG ||
           tilt_deg >= GESTURE_OUTBOUND_TILT_MIN_DEG ||
           gesture_z_ratio_delta(z_gravity_ratio) >=
               GESTURE_OUTBOUND_Z_RATIO_DONE ||
           gesture_z_sign_flipped(az);
}

static bool gesture_pronation_complete_detected(float phi_deg,
                                                float z_gravity_ratio,
                                                float az)
{
    return phi_deg >= GESTURE_PRONATION_MIN_DEG ||
           gesture_z_ratio_delta(z_gravity_ratio) >=
               GESTURE_PRONATION_Z_RATIO_DONE ||
           gesture_z_sign_flipped(az);
}

static void process_gesture_sample(float ax, float ay, float az,
                                   int64_t now)
{

    float accel_norm = vector_norm3(ax, ay, az);
    float z_gravity_ratio = accel_norm > 0.1f ? az / accel_norm : 0.0f;
    float palm_up_z_ratio = fabsf(z_gravity_ratio);
    float dt_s = (gesture_last_sample_ms > 0)
                 ? (float)(now - gesture_last_sample_ms) / 1000.0f
                 : (float)MOTION_SAMPLE_INTERVAL_MS / 1000.0f;

    gesture_last_sample_ms = now;
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = (float)MOTION_SAMPLE_INTERVAL_MS / 1000.0f;
    }

    if (is_recording) {
        update_gravity_lp(ax, ay, az, dt_s);
        if (!stop_requested) {
            if (!recording_stop_ref_valid) {
                capture_recording_stop_reference();
            }
            process_recording_stop_sample(ax, ay, az, now);
        }
        if (gesture_linear_accel_norm_ms2 <= GESTURE_QUIET_ACCEL_MS2) {
            if (gesture_quiet_since_ms == 0) {
                gesture_quiet_since_ms = now;
            }
            update_quiet_accel_reference(ax, ay, az);
        } else {
            gesture_quiet_since_ms = 0;
        }
        return;
    }

    if (recording_requested) {
        update_gravity_lp(ax, ay, az, dt_s);
        if (!recording_stop_ref_valid) {
            capture_recording_stop_reference();
        }
        return;
    }

    if (now < gesture_block_until_ms) {
        reset_gesture_sequence();
        if (gesture_linear_accel_norm_ms2 <= GESTURE_QUIET_ACCEL_MS2) {
            if (gesture_quiet_since_ms == 0) {
                gesture_quiet_since_ms = now;
            }
            update_quiet_accel_reference(ax, ay, az);
        } else {
            gesture_quiet_since_ms = 0;
        }
        return;
    }

    if (gesture_phase != GESTURE_WAITING &&
        (now - gesture_sequence_start_ms) > GESTURE_SEQUENCE_TIMEOUT_MS) {
        send_gesture_diag(GESTURE_DIAG_RESET,
                          GESTURE_DIAG_REASON_SEQUENCE_TIMEOUT,
                          (float)(now - gesture_sequence_start_ms),
                          palm_up_z_ratio, accel_norm);
        reset_gesture_sequence();
        gesture_quiet_since_ms = 0;
        return;
    }

    update_gravity_lp(ax, ay, az, dt_s);

    if (gesture_phase == GESTURE_WAITING) {
        bool board_flat = gesture_board_flat(z_gravity_ratio);

        if (!board_flat) {
            reset_shake_window();
            if ((now - gesture_diag_last_report_ms) >= 250) {
                send_gesture_diag(GESTURE_DIAG_WAIT_REJECT,
                                  GESTURE_DIAG_REASON_START_NOT_PALM_UP,
                                  palm_up_z_ratio,
                                  GESTURE_START_PALM_UP_Z_MIN_RATIO,
                                  gesture_linear_accel_norm_ms2);
                gesture_diag_last_report_ms = now;
            }
            return;
        }

        bool shake_ready = push_shake_sample(dt_s);
        if (shake_ready) {
            arm_from_shake_gravity_reference(now);
            gesture_phase = GESTURE_OUTBOUND;
            gesture_sequence_start_ms = now;
            gesture_phase_start_ms = now;
            gesture_pronation_phi_deg = 0.0f;
            gesture_pronation_peak_deg = 0.0f;
            gesture_pronation_peak_tilt_deg = 0.0f;
            gesture_outbound_qualified = false;
            gesture_quiet_since_ms = 0;
            printk(">>> Gesture shake: ptp=%.2f mean=%.2f z=%.2f\n",
                   (double)gesture_shake_last_ptp_ms2,
                   (double)gesture_shake_last_mean_ms2,
                   (double)palm_up_z_ratio);
            send_gesture_diag(GESTURE_DIAG_OUTBOUND_START,
                              GESTURE_DIAG_REASON_NONE,
                              gesture_shake_last_ptp_ms2,
                              palm_up_z_ratio,
                              gesture_shake_last_mean_ms2);
            return;
        }

        if ((now - gesture_diag_last_report_ms) >= 250) {
            uint8_t reason = GESTURE_DIAG_REASON_SHAKE_NOT_OSCILLATORY;
            float v1 = gesture_shake_last_ptp_ms2;
            float v2 = gesture_shake_last_mean_ms2;
            float v3 = gesture_shake_last_ptp_ms2 > 0.0f
                ? fabsf(gesture_shake_last_mean_ms2) /
                  gesture_shake_last_ptp_ms2
                : 0.0f;
            if (gesture_shake_count < GESTURE_SHAKE_WINDOW_SAMPLES) {
                reason = GESTURE_DIAG_REASON_QUIET_NOT_READY;
                v1 = palm_up_z_ratio;
                v2 = (float)gesture_shake_count;
                v3 = (float)GESTURE_SHAKE_WINDOW_SAMPLES;
            }
            send_gesture_diag(GESTURE_DIAG_WAIT_REJECT, reason, v1, v2, v3);
            gesture_diag_last_report_ms = now;
        }
        return;
    }

    if (gesture_phase == GESTURE_OUTBOUND) {
        int64_t elapsed_ms = now - gesture_phase_start_ms;

        float phi = atan2f(-ax, az) * (float)GESTURE_RAD_TO_DEG;
        float phi_change =
            fabsf(deg_diff(phi, gesture_pronation_ref_phi_deg));

        if (phi_change > gesture_pronation_phi_deg) {
            gesture_pronation_phi_deg = phi_change;
        }
        if (phi_change > gesture_pronation_peak_deg) {
            gesture_pronation_peak_deg = phi_change;
        }

        float tilt_deg = gesture_gravity_tilt_deg(gesture_gravity_lp_x,
                                                  gesture_gravity_lp_y,
                                                  gesture_gravity_lp_z);
        if (tilt_deg > gesture_pronation_peak_tilt_deg) {
            gesture_pronation_peak_tilt_deg = tilt_deg;
        }

        /* Maintain a running peak-to-peak Z deflection during pronation. */
        if (az < gesture_z_window_min) {
            gesture_z_window_min = az;
        }
        if (az > gesture_z_window_max) {
            gesture_z_window_max = az;
        }

        gesture_outbound_qualified =
            elapsed_ms >= GESTURE_PHASE_MIN_DURATION_MS &&
            gesture_outbound_palm_up_detected(
                gesture_pronation_phi_deg, gesture_pronation_peak_tilt_deg,
                z_gravity_ratio, az) &&
            accel_norm >= GESTURE_PRONATION_GRAVITY_MIN_MS2 &&
            accel_norm <= GESTURE_PRONATION_GRAVITY_MAX_MS2;

        if (gesture_outbound_qualified) {
            gesture_phase = GESTURE_HOLDING_FINAL;
            gesture_phase_start_ms = now;
            gesture_final_since_ms = 0;
            gesture_quiet_since_ms = 0;
            gesture_lift_stage = GESTURE_LIFT_WAIT_ACCEL;
            gesture_lift_pos_impulse_ms = 0.0f;
            gesture_lift_neg_impulse_ms = 0.0f;
            gesture_lift_net_impulse_ms = 0.0f;
            gesture_lift_final_tilt_deg = 0.0f;
            gesture_lift_max_hold_tilt_deg = 0.0f;
            gesture_lift_hold_axis_valid = false;
            gesture_lift_pose_failed = false;
            gesture_lift_event_start_ms = 0;
            gesture_lift_accel_samples = 0;
            gesture_lift_brake_samples = 0;
            gesture_debug_final_last_ms = 0;
            clear_accel_history();
            printk(">>> Gesture palm-up: phi=%.1f tilt=%.1f z_d=%.2f\n",
                   (double)gesture_pronation_phi_deg,
                   (double)gesture_pronation_peak_tilt_deg,
                   (double)gesture_z_ratio_delta(z_gravity_ratio));
            send_gesture_diag(GESTURE_DIAG_OUTBOUND_READY,
                              GESTURE_DIAG_REASON_NONE,
                              gesture_pronation_phi_deg,
                              gesture_pronation_peak_tilt_deg,
                              gesture_z_ratio_delta(z_gravity_ratio));
            /* Re-arm so the later hold flip is measured from this palm-up pose. */
            arm_pronation_reference(phi, ax, ay, az, z_gravity_ratio, now);
            gesture_pronation_phi_deg = 0.0f;
            gesture_pronation_peak_deg = 0.0f;
            gesture_pronation_peak_tilt_deg = 0.0f;
            return;
        }

        if (elapsed_ms > GESTURE_OUTBOUND_MAX_DURATION_MS) {
            send_gesture_diag(GESTURE_DIAG_RESET,
                              GESTURE_DIAG_REASON_OUTBOUND_TIMEOUT,
                              gesture_pronation_phi_deg,
                              gesture_pronation_peak_tilt_deg,
                              gesture_z_ratio_delta(z_gravity_ratio));
            reset_gesture_sequence();
            gesture_quiet_since_ms = 0;
            return;
        }

        return;
    }

    /*
     * HOLDING_FINAL: lift pulse (any palm), then rotate away from the
     * palm-up reference and hold still for 500 ms.
     *
     * Linear acceleration is projected onto the current low-pass gravity
     * direction so wrist rotation is not accumulated as translation.  The
     * final pose baseline is captured separately when the hold starts.
     * Integrating each pulse once is substantially less drift-prone than
     * estimating displacement with a double integral.
     */
    {
        float phi = atan2f(-ax, az) * (float)GESTURE_RAD_TO_DEG;
        float phi_change =
            fabsf(deg_diff(phi, gesture_pronation_ref_phi_deg));
        if (phi_change > gesture_pronation_phi_deg) {
            gesture_pronation_phi_deg = phi_change;
        }
        if (phi_change > gesture_pronation_peak_deg) {
            gesture_pronation_peak_deg = phi_change;
        }
        if (az < gesture_z_window_min) {
            gesture_z_window_min = az;
        }
        if (az > gesture_z_window_max) {
            gesture_z_window_max = az;
        }

        float g_norm = vector_norm3(gesture_gravity_lp_x,
                                    gesture_gravity_lp_y,
                                    gesture_gravity_lp_z);
        float a_up = 0.0f;
        if (g_norm > 0.1f) {
            a_up = (gesture_linear_world_x * gesture_gravity_lp_x +
                    gesture_linear_world_y * gesture_gravity_lp_y +
                    gesture_linear_world_z * gesture_gravity_lp_z) /
                   g_norm;
        }
        bool palm_down_ok = gesture_pronation_complete_detected(
            gesture_pronation_phi_deg, z_gravity_ratio, az);

        if (gesture_lift_stage == GESTURE_LIFT_WAIT_ACCEL) {
            if (a_up >= GESTURE_LIFT_ACCEL_MIN_MS2) {
                if (gesture_lift_event_start_ms == 0) {
                    gesture_lift_event_start_ms = now;
                }
                gesture_lift_accel_samples++;
                gesture_lift_pos_impulse_ms += a_up * dt_s;
                gesture_lift_net_impulse_ms += a_up * dt_s;
            } else if (gesture_lift_event_start_ms > 0 && a_up > 0.0f) {
                gesture_lift_pos_impulse_ms += a_up * dt_s;
                gesture_lift_net_impulse_ms += a_up * dt_s;
                gesture_lift_accel_samples = 0;
            } else if (a_up <= 0.0f) {
                gesture_lift_event_start_ms = 0;
                gesture_lift_accel_samples = 0;
                gesture_lift_pos_impulse_ms = 0.0f;
                gesture_lift_net_impulse_ms = 0.0f;
            } else {
                gesture_lift_accel_samples = 0;
            }

            if (gesture_lift_accel_samples >=
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES &&
                gesture_lift_pos_impulse_ms >=
                    GESTURE_LIFT_POS_IMPULSE_MIN_MS) {
                gesture_lift_stage = GESTURE_LIFT_WAIT_BRAKE;
                gesture_lift_brake_samples = 0;
            }
        } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_BRAKE) {
            gesture_lift_net_impulse_ms += a_up * dt_s;
            if (a_up > 0.0f) {
                gesture_lift_pos_impulse_ms += a_up * dt_s;
            } else {
                gesture_lift_neg_impulse_ms += -a_up * dt_s;
            }

            if (a_up <= -GESTURE_LIFT_BRAKE_MIN_MS2) {
                if (gesture_lift_brake_samples <
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES) {
                    gesture_lift_brake_samples++;
                }
            } else if (gesture_lift_brake_samples <
                       GESTURE_LIFT_CONSECUTIVE_SAMPLES) {
                gesture_lift_brake_samples = 0;
            }

            int64_t pulse_ms = now - gesture_lift_event_start_ms;
            bool brake_ready =
                gesture_lift_brake_samples >=
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES &&
                gesture_lift_neg_impulse_ms >=
                    GESTURE_LIFT_NEG_IMPULSE_MIN_MS;
            float brake_ratio =
                gesture_lift_pos_impulse_ms > 0.0f
                    ? gesture_lift_neg_impulse_ms /
                        gesture_lift_pos_impulse_ms
                    : 0.0f;
            if (brake_ready && pulse_ms < GESTURE_LIFT_PULSE_MIN_MS) {
                send_gesture_diag(
                    GESTURE_DIAG_WAIT_REJECT,
                    GESTURE_DIAG_REASON_FINAL_PULSE_DURATION_INVALID,
                    (float)pulse_ms,
                    gesture_lift_pos_impulse_ms,
                    gesture_lift_neg_impulse_ms);
                retry_lift_pulse();
                return;
            } else if (pulse_ms > GESTURE_LIFT_PULSE_MAX_MS) {
                if (gesture_lift_pos_impulse_ms >=
                    GESTURE_LIFT_POS_IMPULSE_MIN_MS) {
                    /*
                     * Slow lifts often lack a sharp braking spike.
                     * Enough upward impulse plus the later hold is the stop.
                     */
                    gesture_lift_stage = GESTURE_LIFT_WAIT_HOLD;
                    gesture_lift_hold_axis_valid = false;
                    gesture_lift_pose_failed = false;
                    clear_accel_history();
                } else {
                    send_gesture_diag(GESTURE_DIAG_WAIT_REJECT,
                                      GESTURE_DIAG_REASON_FINAL_BRAKE_MISSING,
                                      gesture_lift_pos_impulse_ms,
                                      gesture_lift_neg_impulse_ms,
                                      0.0f);
                    retry_lift_pulse();
                    return;
                }
            } else if (brake_ready &&
                       brake_ratio >= GESTURE_LIFT_BRAKE_RATIO_MIN) {
                gesture_lift_stage = GESTURE_LIFT_WAIT_HOLD;
                gesture_lift_hold_axis_valid = false;
                gesture_lift_pose_failed = false;
                clear_accel_history();
            }
        }

        gesture_lift_final_tilt_deg = gesture_tilt_from_hold_axis_deg();
        float final_rms = INFINITY;
        bool final_still = false;
        if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD && !palm_down_ok) {
            gesture_lift_hold_axis_valid = false;
            gesture_final_since_ms = 0;
            clear_accel_history();
        } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD) {
            push_accel_history(gesture_linear_accel_norm_ms2);
            final_rms = gesture_accel_rms();
            final_still = final_rms <= GESTURE_FINAL_STILL_RMS_MS2;
            if (final_still && !gesture_lift_hold_axis_valid) {
                float hold_g_norm = vector_norm3(gesture_gravity_lp_x,
                                                 gesture_gravity_lp_y,
                                                 gesture_gravity_lp_z);
                if (hold_g_norm > 0.1f) {
                    gesture_lift_hold_axis_x =
                        gesture_gravity_lp_x / hold_g_norm;
                    gesture_lift_hold_axis_y =
                        gesture_gravity_lp_y / hold_g_norm;
                    gesture_lift_hold_axis_z =
                        gesture_gravity_lp_z / hold_g_norm;
                    gesture_lift_hold_axis_valid = true;
                    gesture_lift_final_tilt_deg = 0.0f;
                }
            } else if (gesture_lift_hold_axis_valid) {
                gesture_lift_final_tilt_deg =
                    gesture_tilt_from_hold_axis_deg();
                if (gesture_lift_final_tilt_deg >
                    gesture_lift_max_hold_tilt_deg) {
                    gesture_lift_max_hold_tilt_deg =
                        gesture_lift_final_tilt_deg;
                }
            }
        }
        bool final_pose_stable =
            gesture_lift_hold_axis_valid &&
            gesture_lift_final_tilt_deg <=
                GESTURE_LIFT_FINAL_TILT_MAX_DEG;

        if (gesture_debug_final_last_ms == 0 ||
            (now - gesture_debug_final_last_ms) >=
                GESTURE_DEBUG_FINAL_PERIOD_MS) {
            gesture_debug_final_last_ms = now;
            send_gesture_diag(GESTURE_DIAG_FINAL_SAMPLE,
                              GESTURE_DIAG_REASON_NONE,
                              (float)gesture_lift_stage,
                              a_up, gesture_lift_net_impulse_ms);
        }

        if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD &&
            palm_down_ok &&
            final_still && final_pose_stable) {
            if (gesture_final_since_ms == 0) {
                gesture_final_since_ms = now;
                send_gesture_diag(GESTURE_DIAG_FINAL_HOLD_START,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_lift_pos_impulse_ms,
                                  gesture_lift_neg_impulse_ms,
                                  gesture_lift_final_tilt_deg);
            }

            int64_t hold_ms = now - gesture_final_since_ms;
            if (hold_ms >= GESTURE_FINAL_HOLD_MS) {
                printk(">>> Gesture MATCH: shake + palm-up + lift + palm-down hold\n");
                send_gesture_diag(GESTURE_DIAG_FINAL_READY,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_lift_pos_impulse_ms,
                                  (float)hold_ms,
                                  gesture_lift_final_tilt_deg);
                send_gesture_diag(GESTURE_DIAG_MATCH,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_pronation_phi_deg,
                                  gesture_lift_pos_impulse_ms,
                                  (float)hold_ms);
                capture_recording_stop_reference();
                {
                    float gx = gesture_gravity_lp_x;
                    float gy = gesture_gravity_lp_y;
                    float gz = gesture_gravity_lp_z;
                    recording_requested = true;
                    gesture_block_until_ms = now + GESTURE_RETRIGGER_BLOCK_MS;
                    last_activity_ms = now;
                    reset_gesture_sequence();
                    gesture_gravity_lp_x = gx;
                    gesture_gravity_lp_y = gy;
                    gesture_gravity_lp_z = gz;
                }
                gesture_quiet_since_ms = 0;
                return;
            }
        } else {
            bool hold_interrupted = gesture_final_since_ms > 0;
            if (hold_interrupted &&
                !final_pose_stable &&
                gesture_lift_final_tilt_deg >
                    GESTURE_LIFT_FINAL_TILT_MAX_DEG) {
                gesture_lift_pose_failed = true;
            }
            if (hold_interrupted &&
                (now - gesture_diag_last_report_ms) >= 250) {
                send_gesture_diag(GESTURE_DIAG_WAIT_REJECT,
                                  GESTURE_DIAG_REASON_FINAL_HOLD_INTERRUPTED,
                                  final_rms,
                                  gesture_lift_final_tilt_deg,
                                  gesture_lift_pos_impulse_ms);
                gesture_diag_last_report_ms = now;
            }
            if (hold_interrupted) {
                gesture_final_since_ms = 0;
                gesture_lift_hold_axis_valid = false;
                clear_accel_history();
            }
        }

        if ((now - gesture_phase_start_ms) >= GESTURE_FINAL_HOLD_TIMEOUT_MS) {
            uint8_t reason;
            if (gesture_lift_stage == GESTURE_LIFT_WAIT_ACCEL) {
                reason = GESTURE_DIAG_REASON_FINAL_ACCEL_MISSING;
            } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_BRAKE) {
                reason = GESTURE_DIAG_REASON_FINAL_BRAKE_MISSING;
            } else if (gesture_lift_pose_failed) {
                reason = GESTURE_DIAG_REASON_FINAL_TILT_UNSTABLE;
            } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD &&
                       !palm_down_ok) {
                reason = GESTURE_DIAG_REASON_LIFT_PALM_STILL_UP;
            } else {
                reason = GESTURE_DIAG_REASON_FINAL_HOLD_TIMEOUT;
            }
            send_gesture_diag(GESTURE_DIAG_RESET, reason,
                              gesture_lift_pos_impulse_ms,
                              gesture_lift_neg_impulse_ms,
                              gesture_lift_max_hold_tilt_deg);
            reset_gesture_sequence();
            gesture_quiet_since_ms = 0;
        }
    }
}

static void on_motion_started(float x, float y, float z)
{
    motion_active_start_ms = k_uptime_get();
    last_activity_ms = motion_active_start_ms;

    motion_vel_x = 0.0f; motion_vel_y = 0.0f; motion_vel_z = 0.0f;
    motion_distance   = 0.0f;
    motion_peak_speed = 0.0f;
    motion_speed_sum  = 0.0f;
    motion_speed_samples = 0;

    send_event_packet_xyz(0x10, x, y, z);
    printk(">>> Motion active x=%.2f y=%.2f z=%.2f\n", (double)x, (double)y, (double)z);
}

static void on_motion_settled(float x, float y, float z)
{
    last_activity_ms = k_uptime_get();

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

    last_motion_time_ms = (uint32_t)k_uptime_get();
    send_event_packet_settle(x, y, z, elapsed_ms, avg_speed, motion_peak_speed, motion_distance);
    printk(">>> Motion settled x=%.2f y=%.2f z=%.2f elapsed=%u ms avg=%.3f peak=%.3f dist=%.3f\n",
           (double)x, (double)y, (double)z, elapsed_ms,
           (double)avg_speed, (double)motion_peak_speed, (double)motion_distance);

}


static void process_motion_sample(void)
{
    struct sensor_value accel[3];
    int ret = sensor_sample_fetch(imu);
    if (ret < 0) { LOG_ERR("fetch failed: %d", ret); return; }
    if (sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return;

    double x = sensor_value_to_double(&accel[0]);
    double y = sensor_value_to_double(&accel[1]);
    double z = sensor_value_to_double(&accel[2]);

    if (calibration_count < CALIBRATION_SAMPLES) {
        accumulate_calibration(x, y, z);
        return;
    }

    int64_t now = k_uptime_get();
    process_gesture_sample((float)x, (float)y, (float)z, now);

    double delta    = motion_delta(x, y, z);
    double step     = sample_delta(x, y, z);
    update_step_window(step);
    double activity = step_window_sum;
    double peak     = step_window_peak();

    ARG_UNUSED(delta);

    if (!motion_active) {
        bool accel_active = activity >= MOTION_ENTRY_ACTIVITY_MS2 &&
                            peak >= MOTION_ENTRY_PEAK_MS2;
        if (accel_active) {
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

    if (activity <= MOTION_SETTLE_ACTIVITY_MS2 &&
        peak <= MOTION_SETTLE_PEAK_MS2) {
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
        } else if (data[0] == 0x02) {
            /* Claim primary role */
            int idx = conn_index(conn);
            if (idx >= 0) {
                primary_idx = idx;
                inhibit_next_settle = false;
                printk(">>> conn[%d] claimed primary\n", idx);
                /* Update connection params: fast for primary, slow for secondary */
                k_work_schedule(&conn_param_work, K_MSEC(200));
            }
        } else if (data[0] == 0x03) {
            /* Yield primary role to the other connection */
            bool found = false;
            for (int i = 0; i < MAX_CONNS; i++) {
                if (connections[i] && connections[i] != conn) {
                    primary_idx = i;
                    printk(">>> Primary yielded to conn[%d]\n", i);
                    found = true;
                    break;
                }
            }
            if (!found) {
                primary_idx = -1;
                printk(">>> Primary yielded but no peer present; primary cleared\n");
            }
            /* Update connection params after yield */
            k_work_schedule(&conn_param_work, K_MSEC(200));
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
    if (!get_primary_conn()) {
        printk(">>> send_event 0x%02x: no primary conn\n", event_code);
        return;
    }
    printk(">>> send_event 0x%02x -> primary[%d]\n", event_code, primary_idx);
    uint8_t pkt[3] = { 0x00, 0x55, event_code };
    bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* motion_active/xyz packet: [0x00][0x55][code][f32_x][f32_y][f32_z] = 15 bytes */
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z)
{
    if (!get_primary_conn()) { return; }
    uint8_t pkt[15];
    pkt[0] = 0x00; pkt[1] = 0x55; pkt[2] = event_code;
    memcpy(&pkt[3],  &x, 4);
    memcpy(&pkt[7],  &y, 4);
    memcpy(&pkt[11], &z, 4);
    bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* Gesture diagnostic packet:
 * [0x00][0x55][0x30][stage][reason][f32 value1][f32 value2][f32 value3]
 * = 17 bytes. Values depend on stage and are documented in
 * docs/flex_pronation_gesture.md. */
static void send_gesture_diag(uint8_t stage, uint8_t reason,
                              float value1, float value2, float value3)
{
    if (!get_primary_conn()) { return; }
    uint8_t pkt[17];
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = 0x30;
    pkt[3] = stage;
    pkt[4] = reason;
    memcpy(&pkt[5], &value1, 4);
    memcpy(&pkt[9], &value2, 4);
    memcpy(&pkt[13], &value3, 4);
    bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* motion_settled extended packet: [0x00][0x55][0x11][f32_x][f32_y][f32_z][u32_elapsed_ms]
 *   [f32_avg_speed][f32_peak_speed][f32_distance] = 31 bytes */
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance)
{
    if (!get_primary_conn()) { return; }
    uint8_t pkt[31];
    pkt[0] = 0x00; pkt[1] = 0x55; pkt[2] = 0x11;
    memcpy(&pkt[3],  &x,          4);
    memcpy(&pkt[7],  &y,          4);
    memcpy(&pkt[11], &z,          4);
    memcpy(&pkt[15], &elapsed_ms, 4);
    memcpy(&pkt[19], &avg_speed,  4);
    memcpy(&pkt[23], &peak_speed, 4);
    memcpy(&pkt[27], &distance,   4);
    bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
}

/* ============================================================================
 * Audio Streaming
 * ============================================================================ */

static void audio_stats_reset(void)
{
    memset(&audio_stats, 0, sizeof(audio_stats));
    audio_stats.session_start_ms = k_uptime_get();
}

static void audio_stats_print(const char *reason)
{
    int64_t now = k_uptime_get();
    int64_t session_ms = now - audio_stats.session_start_ms;
    int64_t dmic_ms = 0;

    if (audio_stats.dmic_stop_ms > 0) {
        dmic_ms = audio_stats.dmic_stop_ms - audio_stats.session_start_ms;
    } else if (session_ms > 0) {
        dmic_ms = session_ms;
    }

    printk("Audio stats (%s): sess=%lldms dmic=%lldms cap=%u sent=%u bytes=%u "
           "ntfy=%u wait=%u wait_max=%ums retry=%u read_err=%u last_err=%d "
           "overrun=%u q_hi=%u\n",
           reason,
           (long long)session_ms,
           (long long)dmic_ms,
           audio_stats.frames_captured,
           audio_stats.frames_sent,
           audio_stats.bytes_sent,
           audio_stats.notifies_sent,
           audio_stats.notify_wait_count,
           audio_stats.notify_wait_max_ms,
           audio_stats.notify_retry_count,
           audio_stats.dmic_read_errors,
           (int)audio_stats.last_dmic_errno,
           audio_stats.dmic_overruns,
           audio_stats.queue_high_watermark);
}

static void audio_queue_purge(void)
{
    struct audio_pcm_frame drop;

    while (k_msgq_get(&audio_frame_q, &drop, K_NO_WAIT) == 0) {
    }
}

static void audio_queue_note_depth(void)
{
    uint32_t depth = (uint32_t)k_msgq_num_used_get(&audio_frame_q);

    if (depth > audio_stats.queue_high_watermark) {
        audio_stats.queue_high_watermark = depth;
    }
}

static void audio_notify_complete(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&audio_notify_slots);
}

static int send_audio_notification_deadline(const uint8_t *data, uint16_t len,
                                            int64_t deadline_ms)
{
    while (k_uptime_get() < deadline_ms) {
        if (!get_primary_conn()) {
            return -ENOTCONN;
        }

        int64_t wait_start = k_uptime_get();
        int64_t remaining_ms = deadline_ms - wait_start;
        if (remaining_ms <= 0) {
            break;
        }
        if (remaining_ms > 100) {
            remaining_ms = 100;
        }

        if (k_sem_take(&audio_notify_slots, K_MSEC(remaining_ms)) != 0) {
            uint32_t waited = (uint32_t)(k_uptime_get() - wait_start);

            audio_stats.notify_wait_count++;
            if (waited > audio_stats.notify_wait_max_ms) {
                audio_stats.notify_wait_max_ms = waited;
            }
            continue;
        }

        uint32_t waited = (uint32_t)(k_uptime_get() - wait_start);
        if (waited > 0) {
            audio_stats.notify_wait_count++;
            if (waited > audio_stats.notify_wait_max_ms) {
                audio_stats.notify_wait_max_ms = waited;
            }
        }

        struct bt_conn *conn = get_primary_conn();
        if (!conn) {
            k_sem_give(&audio_notify_slots);
            return -ENOTCONN;
        }

        struct bt_gatt_notify_params params = {
            .attr = &audio_svc.attrs[2],
            .data = data,
            .len = len,
            .func = audio_notify_complete,
            .user_data = NULL,
        };
        int ret = bt_gatt_notify_cb(conn, &params);
        if (ret == 0) {
            audio_stats.notifies_sent++;
            return 0;
        }

        k_sem_give(&audio_notify_slots);
        if (ret != -ENOMEM && ret != -EAGAIN) {
            return ret;
        }
        audio_stats.notify_retry_count++;
        k_msleep(AUDIO_NOTIFY_RETRY_MS);
    }

    return -ECANCELED;
}

static int send_audio_notification(const uint8_t *data, uint16_t len)
{
    while (is_recording && !stop_requested) {
        int ret = send_audio_notification_deadline(
            data, len, k_uptime_get() + 100);
        if (ret != -ECANCELED) {
            return ret;
        }
        /* Still recording: keep trying through transient slot starvation. */
    }

    return -ECANCELED;
}

static bool drain_audio_notifications(void)
{
    int acquired = 0;
    int64_t deadline = k_uptime_get() + AUDIO_NOTIFY_DRAIN_TIMEOUT_MS;

    while (acquired < AUDIO_NOTIFY_IN_FLIGHT) {
        int64_t remaining_ms = deadline - k_uptime_get();
        if (remaining_ms <= 0 ||
            k_sem_take(&audio_notify_slots, K_MSEC(remaining_ms)) != 0) {
            break;
        }
        acquired++;
    }

    for (int i = 0; i < acquired; i++) {
        k_sem_give(&audio_notify_slots);
    }

    if (acquired != AUDIO_NOTIFY_IN_FLIGHT) {
        printk("Audio notify drain timed out: %d/%d slots returned\n",
               acquired, AUDIO_NOTIFY_IN_FLIGHT);
        return false;
    }
    return true;
}

static int stream_audio_frame_deadline(const uint8_t *pcm, size_t audio_size,
                                       int64_t deadline_ms, bool use_deadline)
{
    size_t total_samples = audio_size / sizeof(int16_t);
    size_t offset = 0;

    while (offset < total_samples) {
        size_t samples_to_send = total_samples - offset;
        if (samples_to_send > PCM_PACKET_SIZE / sizeof(int16_t)) {
            samples_to_send = PCM_PACKET_SIZE / sizeof(int16_t);
        }

        tx_packet[0] = seq_num;
        tx_packet[1] = 0xAA;
        memcpy(&tx_packet[2], &pcm[offset * sizeof(int16_t)],
               samples_to_send * sizeof(int16_t));

        int ret;
        if (use_deadline) {
            ret = send_audio_notification_deadline(
                tx_packet, 2 + (samples_to_send * sizeof(int16_t)), deadline_ms);
        } else {
            ret = send_audio_notification(
                tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        }
        if (ret != 0) {
            return ret;
        }

        seq_num++;
        offset += samples_to_send;
        audio_stats.bytes_sent += samples_to_send * sizeof(int16_t);
    }

    audio_stats.frames_sent++;
    return 0;
}

static int stream_audio_frame(const uint8_t *pcm, size_t audio_size)
{
    return stream_audio_frame_deadline(pcm, audio_size, 0, false);
}

static int stream_queued_frames(bool blocking)
{
    struct audio_pcm_frame frame;
    k_timeout_t timeout = blocking ? K_MSEC(20) : K_NO_WAIT;

    while (is_recording && !stop_requested && get_primary_conn()) {
        int get_ret = k_msgq_get(&audio_frame_q, &frame, timeout);
        if (get_ret != 0) {
            return 0;
        }

        int ret = stream_audio_frame(frame.pcm, frame.nbytes);
        if (ret < 0 && ret != -ECANCELED && ret != -ENOTCONN) {
            printk("Audio notify failed: %d\n", ret);
            k_msleep(AUDIO_NOTIFY_RETRY_MS);
        } else if (ret == -ENOTCONN || ret == -ECANCELED) {
            return ret;
        }

        /* After the first frame, prefer draining without sleeping. */
        timeout = K_NO_WAIT;
    }

    return 0;
}

static void drain_queued_frames_before_stop(void)
{
    struct audio_pcm_frame frame;
    int64_t deadline = k_uptime_get() + AUDIO_QUEUE_DRAIN_TIMEOUT_MS;

    while (k_uptime_get() < deadline) {
        if (k_msgq_get(&audio_frame_q, &frame, K_MSEC(10)) != 0) {
            if (k_msgq_num_used_get(&audio_frame_q) == 0) {
                break;
            }
            continue;
        }
        if (!get_primary_conn()) {
            break;
        }
        if (stream_audio_frame_deadline(frame.pcm, frame.nbytes,
                                        deadline, true) != 0) {
            break;
        }
    }

    audio_queue_purge();
}

/*
 * Dedicated capture path: always free DMIC blocks promptly by copying into the
 * software queue. BLE Notify backpressure must never stall dmic_read().
 */
static void audio_capture_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int consec_fail = 0;

    printk("Audio capture thread started\n");

    while (1) {
        if (!capture_enabled) {
            consec_fail = 0;
            k_msleep(5);
            continue;
        }

        int16_t *audio_buffer = NULL;
        size_t audio_size = 0;
        int ret = audio_capture_get_data(&audio_buffer, &audio_size);
        if (ret < 0) {
            audio_stats.dmic_read_errors++;
            audio_stats.last_dmic_errno = (uint32_t)(-ret);
            consec_fail++;

            /*
             * Transient stalls are expected under radio contention. Only mark
             * the capture path fatal after repeated failures; the sender keeps
             * draining whatever was already queued.
             */
            if (consec_fail >= AUDIO_CAPTURE_MAX_CONSEC_FAIL) {
                printk("Audio read failed: %d (giving up after %d errors)\n",
                       ret, consec_fail);
                if (audio_stats.dmic_stop_ms == 0) {
                    audio_stats.dmic_stop_ms = k_uptime_get();
                }
                capture_fatal = true;
                capture_enabled = false;
                (void)audio_capture_stop();
                consec_fail = 0;
            }
            continue;
        }

        consec_fail = 0;
        if (audio_buffer == NULL || audio_size == 0) {
            continue;
        }

        struct audio_pcm_frame frame;
        size_t copy_len = audio_size;

        if (copy_len > AUDIO_FRAME_BYTES) {
            copy_len = AUDIO_FRAME_BYTES;
        }
        frame.nbytes = (uint16_t)copy_len;
        memcpy(frame.pcm, audio_buffer, copy_len);
        audio_stats.frames_captured++;

        if (k_msgq_put(&audio_frame_q, &frame, K_NO_WAIT) != 0) {
            audio_stats.dmic_overruns++;
        } else {
            audio_queue_note_depth();
        }
    }
}

static void audio_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;
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
            capture_fatal = false;
            seq_num = 0;
            audio_stats_reset();
            audio_queue_purge();

            if (dmic_available) {
                mic_power_on();
                ret = audio_capture_start();
                if (ret < 0) {
                    printk("Audio start failed: %d\n", ret);
                    mic_power_off();
                    dmic_session_active = false;
                    capture_enabled = false;
                } else {
                    dmic_session_active = true;
                    capture_enabled = true;
                    printk("Audio capture running\n");
                }
            }

            is_recording = true;
            printk("Recording started\n");
            send_event_packet(0x01);
        }

        if ((stop_requested || capture_fatal) && is_recording) {
            bool was_fatal = capture_fatal;

            stop_requested = false;
            capture_fatal = false;
            capture_enabled = false;

            if (dmic_session_active) {
                if (audio_stats.dmic_stop_ms == 0) {
                    audio_stats.dmic_stop_ms = k_uptime_get();
                }
                ret = audio_capture_stop();
                if (ret < 0) {
                    printk("Audio stop failed: %d\n", ret);
                }
                mic_power_off();
                dmic_session_active = false;
            }

            /* Flush any frames captured before stop under light contention. */
            drain_queued_frames_before_stop();
            is_recording = false;
            reset_recording_stop_state();

            /* Preserve ordering: all accepted PCM notifications must complete before stop. */
            (void)drain_audio_notifications();
            printk("Recording stopped%s\n", was_fatal ? " (capture fault)" : "");
            audio_stats_print(was_fatal ? "fault" : "stop");
            send_event_packet(0x02);
            battery_update();
            last_activity_ms = k_uptime_get();   /* reset idle timer after recording */
            continue;
        }

        if (is_recording && get_primary_conn()) {
            (void)stream_queued_frames(true);
            continue;
        }

        if (!get_primary_conn() && dmic_session_active) {
            capture_enabled = false;
            if (audio_stats.dmic_stop_ms == 0) {
                audio_stats.dmic_stop_ms = k_uptime_get();
            }
            (void)audio_capture_stop();
            mic_power_off();
            dmic_session_active = false;
            audio_queue_purge();
            if (is_recording) {
                is_recording = false;
                reset_recording_stop_state();
                (void)drain_audio_notifications();
                printk("Recording stopped (disconnected)\n");
                audio_stats_print("disconnect");
                send_event_packet(0x02);
            }
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

static struct bt_gatt_exchange_params exchange_params[MAX_CONNS];

/* ============================================================================
 * Deferred advertising restart
 * ============================================================================ */

static void adv_work_handler(struct k_work *work)
{
    if (active_count() >= MAX_CONNS) {
        return;
    }
    int ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data),
                              scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret && ret != -EALREADY) {
        LOG_WRN("Advertising restart failed: %d", ret);
        printk(">>> Adv restart failed: %d\n", ret);
    } else {
        printk(">>> Advertising restarted\n");
    }
}

/* ============================================================================
 * Serial command thread — reads from USB CDC ACM UART
 *   'r' → recording start
 *   's' → recording stop
 * ============================================================================ */

static void serial_cmd_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        printk(">>> Serial: console UART not ready\n");
        return;
    }

    printk(">>> Serial command thread ready ('r'=start 's'=stop)\n");

    while (true) {
        unsigned char ch;
        int ret = uart_poll_in(uart_dev, &ch);
        if (ret == 0) {
            if (ch == 'r' || ch == 'R') {
                printk(">>> SERIAL: recording start\n");
                stop_requested = false;
                recording_requested = true;
            } else if (ch == 's' || ch == 'S') {
                printk(">>> SERIAL: recording stop\n");
                stop_requested = true;
            }
        } else {
            k_msleep(20);
        }
    }
}

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("BLE connection failed: %d", err); return; }

    LOG_INF("BLE connected");
    printk(">>> Connected!\n");

    int slot = free_slot();
    if (slot < 0) {
        LOG_WRN("No free connection slot, rejecting");
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
        return;
    }

    connections[slot] = bt_conn_ref(conn);

    if (primary_idx < 0) {
        primary_idx = slot;
        printk(">>> conn[%d] is primary\n", slot);
    } else {
        printk(">>> conn[%d] is secondary, notifying primary\n", slot);
        uint8_t pkt[3] = { 0x00, 0x55, 0x31 };
        bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
    }

    led_sync_runtime_state();

    exchange_params[slot].func = exchange_func;
    int ret = bt_gatt_exchange_mtu(conn, &exchange_params[slot]);
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

    if (active_count() < MAX_CONNS) {
        /* Schedule advertising restart via work queue — calling bt_le_adv_start()
         * directly from within a BT callback can fail on Nordic NCS. */
        k_work_schedule(&adv_work, K_MSEC(100));
        printk(">>> Advertising restart scheduled\n");
    }
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected: %d", reason);
    printk(">>> Disconnected: %d\n", reason);

    int idx = conn_index(conn);
    if (idx < 0) {
        return;
    }

    bt_conn_unref(connections[idx]);
    connections[idx] = NULL;

    if (primary_idx == idx) {
        primary_idx = -1;
        for (int i = 0; i < MAX_CONNS; i++) {
            if (connections[i]) {
                primary_idx = i;
                printk(">>> conn[%d] promoted to primary\n", i);
                break;
            }
        }
        if (primary_idx < 0) {
            is_recording = false;
            stop_requested = true;
            reset_recording_stop_state();
        }
    } else if (primary_idx >= 0) {
        printk(">>> Secondary disconnected, notifying primary\n");
        uint8_t pkt[3] = { 0x00, 0x55, 0x32 };
        bt_gatt_notify(get_primary_conn(), &audio_svc.attrs[2], pkt, sizeof(pkt));
    }

    led_sync_runtime_state();

    /* Schedule advertising restart via work queue */
    k_work_schedule(&adv_work, K_MSEC(100));
    printk(">>> Advertising restart scheduled\n");
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
    printk("nordic-main: XIAO nRF52840 Sense\n");
    printk("Build: %s %s\n", __DATE__, __TIME__);
    printk("============================================\n\n");

    log_reset_cause();

    LOG_INF("Starting nordic-main");

    ret = configure_leds();
    if (ret < 0) LOG_WRN("LED setup failed: %d", ret);

    led_set_state(LED_BOOT);  /* White 1 second */

    ret = configure_mic_power();
    if (ret < 0) LOG_WRN("Mic power GPIO setup failed: %d", ret);

    ret = battery_init();
    if (ret < 0) LOG_WRN("Battery monitor init failed: %d", ret);

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

    /* Initialize deferred advertising work */
    k_work_init_delayable(&adv_work, adv_work_handler);
    k_work_init_delayable(&conn_param_work, conn_param_work_handler);

    /* Capture runs above sender so DMIC blocks are freed under BLE backpressure. */
    k_thread_create(&audio_capture_thread_data, audio_capture_stack,
                    K_THREAD_STACK_SIZEOF(audio_capture_stack),
                    audio_capture_thread, NULL, NULL, NULL, 13, 0, K_NO_WAIT);
    /* Sender/control thread (priority 14, below BLE stack) */
    k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread, NULL, NULL, NULL, 14, 0, K_NO_WAIT);

    /* Start serial command thread (priority 10) */
    k_thread_create(&serial_thread_data, serial_stack, K_THREAD_STACK_SIZEOF(serial_stack),
                    serial_cmd_thread, NULL, NULL, NULL, 10, 0, K_NO_WAIT);

    LOG_INF("nordic-main ready");

    /* Boot LED: white for ~1 second, then switch to idle/off */
    k_msleep(1000);
    led_set_state(LED_IDLE);

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

    /* Initial battery reading after BLE is ready */
    battery_update();

    /* Main loop: watchdog feed + LED state management + blink tick + IMU motion */
    int64_t last_motion_sample_ms = k_uptime_get();
    int64_t last_battery_ms = k_uptime_get();
    last_activity_ms = k_uptime_get();
    while (1) {
        k_msleep(MAIN_LOOP_INTERVAL_MS);

        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }

        led_sync_runtime_state();

        int64_t now_ms = k_uptime_get();

        /* IMU motion detection — poll faster when active, slower when sleeping */
        int64_t imu_poll_ms = light_sleep_active
                              ? SLEEP_POLL_INTERVAL_MS
                              : MOTION_SAMPLE_INTERVAL_MS;
        if (device_is_ready(imu) &&
            (now_ms - last_motion_sample_ms) >= imu_poll_ms) {
            last_motion_sample_ms = now_ms;
            process_motion_sample();
        }

        led_tick();

        /* Light sleep management */
        if (!motion_active && !is_recording &&
            gesture_phase == GESTURE_WAITING) {
            if (!light_sleep_active &&
                (now_ms - last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
                light_sleep_active = true;
                send_event_packet(0x20);
                printk(">>> Light sleep enter\n");
            }
        } else {
            if (light_sleep_active) {
                light_sleep_active = false;
                send_event_packet(0x21);
                printk(">>> Light sleep wake\n");
            }
            last_activity_ms = now_ms;
        }

        /* Battery polling — force immediate re-read on USB disconnect */
        {
            static bool vbus_prev_main;
            bool vbus_now = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
            if (!vbus_now && vbus_prev_main) {
                last_battery_ms = 0; /* trigger immediate ADC read */
            }
            vbus_prev_main = vbus_now;
        }
        if ((now_ms - last_battery_ms) >= BATTERY_POLL_INTERVAL_MS) {
            last_battery_ms = now_ms;
            battery_update();
        }
    }

    return 0;
}
