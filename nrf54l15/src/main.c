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

#include "adpcm.h"
#include "audio_capture.h"

LOG_MODULE_REGISTER(voice_bridge, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME     "VoiceBridge"
#define BLE_MTU             512

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
static struct bt_conn_cb conn_callbacks;

static uint8_t tx_packet[256];  /* ADPCM frame + 2-byte header */
static uint8_t seq_num = 0;

/* Recording state */
static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;

/* Audio thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static struct k_thread audio_thread_data;

/* ============================================================================
 * BLE GATT Service
 * ============================================================================ */

/* TX Value Changed Callback */
static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("TX CCCD updated: %d", value);
}

/* RX Write Callback */
static ssize_t rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset,
                        uint8_t flags)
{
    const uint8_t *data = buf;
    
    LOG_INF("Received command: len=%d", len);
    
    if (len >= 1) {
        if (data[0] == 0x01) {
            LOG_INF("Start recording command");
            recording_requested = true;
        } else if (data[0] == 0x00) {
            LOG_INF("Stop recording command");
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

    printk("Audio thread started\n");

    /* Initialize audio capture */
    ret = audio_capture_init();
    if (ret < 0) {
        printk("Audio init failed: %d\n", ret);
        return;
    }

    ret = audio_capture_start();
    if (ret < 0) {
        printk("Audio start failed: %d\n", ret);
        return;
    }
    printk("Audio capture running\n");

    /*
     * Strategy: encode PDM frame to ADPCM, send in 18-byte chunks.
     * Each PDM frame (20ms) = 320 samples → 160 bytes ADPCM
     * Send as 9 packets × 20 bytes (2 hdr + 18 data) with k_msleep(2).
     * 9 packets × 2ms = 18ms send time per 20ms frame → fits real-time.
     */
    #define CHUNK_SIZE 18

    static adpcm_state_t enc_state;
    static uint8_t adpcm_buf[256];

    adpcm_init_state(&enc_state);

    while (1) {
        if (recording_requested && !is_recording) {
            is_recording = true;
            recording_requested = false;
            seq_num = 0;
            adpcm_init_state(&enc_state);
        }

        if (stop_requested && is_recording) {
            is_recording = false;
            stop_requested = false;
        }

        if (is_recording && current_conn) {
            ret = audio_capture_get_data(&audio_buffer, &audio_size);
            if (ret == 0 && audio_buffer != NULL && audio_size > 0) {
                size_t total_samples = audio_size / 2;

                /* Encode entire frame to ADPCM */
                size_t adpcm_len = adpcm_encode(&enc_state,
                    audio_buffer, total_samples, adpcm_buf);

                /* Send ADPCM data in small chunks */
                size_t offset = 0;
                while (offset < adpcm_len) {
                    size_t chunk = adpcm_len - offset;
                    if (chunk > CHUNK_SIZE) {
                        chunk = CHUNK_SIZE;
                    }

                    tx_packet[0] = seq_num++;
                    tx_packet[1] = 0xBB;
                    memcpy(&tx_packet[2], &adpcm_buf[offset], chunk);

                    ret = bt_gatt_notify(NULL,
                        &voice_bridge_svc.attrs[2],
                        tx_packet, 2 + chunk);

                    if (ret == -ENOMEM) {
                        k_msleep(2);
                        ret = bt_gatt_notify(NULL,
                            &voice_bridge_svc.attrs[2],
                            tx_packet, 2 + chunk);
                    }

                    offset += chunk;
                    k_msleep(2);
                }
            }
            /* dmic_read blocks ~20ms, giving BLE stack time */
        } else {
            k_msleep(50);
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
    } else {
        LOG_INF("MTU exchange done");
    }
}

static struct bt_gatt_exchange_params exchange_params = {
    .func = exchange_func,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed: %d", err);
        return;
    }

    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);

    /* Request MTU exchange */
    int ret = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (ret) {
        LOG_ERR("MTU exchange request failed: %d", ret);
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
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    
    is_recording = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ============================================================================
 * BLE Initialization
 * ============================================================================ */

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

    LOG_INF("Starting Voice Bridge BLE");

    /* Initialize BLE (synchronous - pass NULL callback) */
    LOG_INF("Initializing BLE");
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("BLE enable failed: %d", ret);
        return ret;
    }

    LOG_INF("BLE initialized");

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Set device name */
    ret = bt_set_name(BLE_DEVICE_NAME);
    if (ret) {
        LOG_ERR("Failed to set name: %d", ret);
        return ret;
    }

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
        return ret;
    }

    LOG_INF("Advertising started: %s", BLE_DEVICE_NAME);

    /* Start audio thread */
    /* Priority 14: lower than BLE stack (7) to avoid starving radio */
    k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread, NULL, NULL, NULL, 14, 0, K_NO_WAIT);

    LOG_INF("Voice Bridge BLE ready");

    /* Keep main thread alive */
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
