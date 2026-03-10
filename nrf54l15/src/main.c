/*
 * Voice Bridge BLE - XIAO nRF54L15 Sense
 * PDM Microphone to Mac via BLE
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(voice_bridge, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME     "VoiceBridge"

/* Recording packet size */
#define PCM_PACKET_SIZE     200

/* ============================================================================
 * BLE UUIDs (Same as ESP32/nRF52840 version for compatibility)
 * ============================================================================ */

/* Custom Service UUID: 00000001-0000-1000-8000-00805f9b34fb */
#define LBS_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00

/* TX Characteristic UUID: 00000002-0000-1000-8000-00805f9b34fb (Notify) */
#define LBS_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00

/* RX Characteristic UUID: 00000003-0000-1000-8000-00805f9b34fb (Write) */
#define LBS_UUID_RX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00

/* ============================================================================
 * Global Variables
 * ============================================================================ */

static struct bt_conn *current_conn;

static uint8_t tx_packet[512];  /* PCM frame + 2-byte header */
static uint8_t seq_num = 0;

/* Recording state */
static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;

/* Audio thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static struct k_thread audio_thread_data;
static void stream_audio_frame(const int16_t *audio_buffer, size_t audio_size);

static void generate_simulated_audio(int16_t *buffer, size_t sample_count)
{
    static uint32_t phase;
    const uint32_t freq = 440;
    const uint32_t sample_rate = audio_capture_get_sample_rate();

    for (size_t i = 0; i < sample_count; i++) {
        int32_t angle = (phase & 0xFFFF) * 360 / 0x10000;
        int32_t sample;

        if (angle < 90) {
            sample = angle * angle / 81;
        } else if (angle < 180) {
            sample = 16384 - (angle - 90) * (angle - 90) / 81;
        } else if (angle < 270) {
            sample = -(angle - 180) * (angle - 180) / 81;
        } else {
            sample = -16384 + (angle - 270) * (angle - 270) / 81;
        }

        buffer[i] = (int16_t)(sample * 2);
        phase += (freq * 0x10000) / sample_rate;
    }
}

/* ============================================================================
 * BLE GATT Service
 * ============================================================================ */

/* TX Value Changed Callback */
static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("TX CCCD updated: %d", value);
    printk(">>> CCCD updated: %d (1=notify enabled)\n", value);
}

/* RX Write Callback */
static ssize_t rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset,
                        uint8_t flags)
{
    const uint8_t *data = buf;

    LOG_INF("Received command: len=%d", len);
    printk(">>> RX write: len=%d\n", len);

    if (len >= 1) {
        if (data[0] == 0x01) {
            LOG_INF("Start recording command");
            printk(">>> START recording command received\n");
            recording_requested = true;
        } else if (data[0] == 0x00) {
            LOG_INF("Stop recording command");
            printk(">>> STOP recording command received\n");
            stop_requested = true;
        }
    }

    return len;
}

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(voice_bridge_svc,
    BT_GATT_PRIMARY_SERVICE(
        BT_UUID_DECLARE_128(LBS_UUID_SERVICE)),

    /* TX Characteristic (Notify) */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(LBS_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        NULL, NULL, NULL),
    BT_GATT_CCC(tx_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* RX Characteristic (Write) */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(LBS_UUID_RX_CHAR),
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, rx_write, NULL),
);

static void stream_audio_frame(const int16_t *audio_buffer, size_t audio_size)
{
    int ret;
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

        ret = bt_gatt_notify(NULL, &voice_bridge_svc.attrs[2],
                             tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        if (ret == -ENOMEM) {
            k_msleep(1);
            ret = bt_gatt_notify(NULL, &voice_bridge_svc.attrs[2],
                                 tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        }

        if (ret == 0) {
            packets_sent++;
        } else {
            notify_errors++;
            printk("!!! bt_gatt_notify error: %d\n", ret);
        }

        offset += samples_to_send;
        k_msleep(1);
    }

    printk("Audio: %zu samples, sent %zu packets, errors %zu\n",
           total_samples, packets_sent, notify_errors);
}

/* ============================================================================
 * Audio Streaming Thread
 * ============================================================================ */

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
    static int16_t sim_buffer[320];

    printk("Audio thread started\n");

    /* Initialize audio capture */
    ret = audio_capture_init();
    if (ret < 0) {
        printk("Audio init failed: %d\n", ret);
        printk(">>> Using simulated audio data\n");
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
                ret = audio_capture_start();
                if (ret < 0) {
                    printk("Audio start failed: %d\n", ret);
                    printk(">>> Falling back to simulated audio\n");
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
                dmic_session_active = false;
            }

            printk("Recording stopped\n");
        }

        if (is_recording && current_conn) {
            if (dmic_session_active) {
                ret = audio_capture_get_data(&audio_buffer, &audio_size);
                if (ret < 0) {
                    printk("Audio read failed: %d\n", ret);
                    printk(">>> Falling back to simulated audio\n");
                    (void)audio_capture_stop();
                    dmic_session_active = false;
                    continue;
                }
            } else {
                generate_simulated_audio(sim_buffer, ARRAY_SIZE(sim_buffer));
                audio_buffer = sim_buffer;
                audio_size = sizeof(sim_buffer);
            }

            stream_audio_frame(audio_buffer, audio_size);
            continue;
        }

        if (!current_conn && dmic_session_active) {
            (void)audio_capture_stop();
            dmic_session_active = false;
        }

        if (!is_recording) {
            k_msleep(20);
        } else {
            static int debug_count = 0;
            if (debug_count++ % 50 == 0 && !current_conn) {
                printk("DEBUG: Not connected (current_conn=NULL)\n");
            }
            k_msleep(20);
        }
    }
}

/* ============================================================================
 * BLE Connection Callbacks
 * ============================================================================ */

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    if (att_err) {
        LOG_ERR("MTU exchange failed: %d", att_err);
        printk(">>> MTU exchange failed: %d\n", att_err);
    } else {
        LOG_INF("MTU exchange done");
        printk(">>> MTU exchange done\n");
    }
}

static struct bt_gatt_exchange_params exchange_params = {
    .func = exchange_func,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed: %d", err);
        printk(">>> Connection failed: %d\n", err);
        return;
    }

    LOG_INF("Connected");
    printk(">>> Connected! current_conn set\n");
    current_conn = conn;

    /* Request MTU exchange */
    int ret = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (ret) {
        LOG_ERR("MTU exchange request failed: %d", ret);
        printk(">>> MTU exchange failed: %d\n", ret);
    } else {
        printk(">>> MTU exchange requested\n");
    }

    /* Request short connection interval for audio throughput */
    struct bt_le_conn_param conn_param = {
        .interval_min = 6,   /* 7.5ms */
        .interval_max = 12,  /* 15ms */
        .latency = 0,
        .timeout = 400,      /* 4s */
    };
    ret = bt_conn_le_param_update(conn, &conn_param);
    if (ret) {
        LOG_WRN("Conn param update request failed: %d", ret);
        printk(">>> Conn param update failed: %d\n", ret);
    } else {
        printk(">>> Conn param update requested\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    printk(">>> Disconnected: %d\n", reason);

    current_conn = NULL;

    is_recording = false;
    stop_requested = true;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ============================================================================
 * Main Application
 * ============================================================================ */

int main(void)
{
    int ret;

    printk("\n");
    printk("============================================\n");
    printk("Voice Bridge BLE - XIAO nRF54L15 Sense\n");
    printk("============================================\n\n");
    printk("Starting firmware build: " __DATE__ " " __TIME__ "\n");

    LOG_INF("Starting Voice Bridge BLE");

    /* Initialize BLE (synchronous - pass NULL callback) */
    LOG_INF("Initializing BLE");
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("BLE enable failed: %d", ret);
        printk("!!! BLE enable failed: %d\n", ret);
        return ret;
    }

    LOG_INF("BLE initialized");
    printk("BLE initialized\n");

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Set device name */
    ret = bt_set_name(BLE_DEVICE_NAME);
    if (ret) {
        LOG_ERR("Failed to set name: %d", ret);
        printk("!!! Failed to set name: %d\n", ret);
        return ret;
    }

    printk("Device name: %s\n", BLE_DEVICE_NAME);

    /* Start advertising */
    struct bt_data adv_data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00),
    };

    struct bt_data scan_rsp[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
    };

    ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data),
                          scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret) {
        LOG_ERR("Advertising failed: %d", ret);
        printk("!!! Advertising failed: %d\n", ret);
        return ret;
    }

    LOG_INF("Advertising started: %s", BLE_DEVICE_NAME);
    printk("Advertising: %s\n", BLE_DEVICE_NAME);

    /* Start audio thread */
    /* Priority 14: lower than BLE stack (7) to avoid starving radio */
    k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread, NULL, NULL, NULL, 14, 0, K_NO_WAIT);

    LOG_INF("Voice Bridge BLE ready");
    printk("System ready\n");

    /* Keep main thread alive */
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
