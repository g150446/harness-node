/*
 * Voice Bridge BLE - XIAO nRF52840 Sense
 * Simple BLE Peripheral with Audio Streaming
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(voice_bridge, LOG_LEVEL_INF);

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME     "VoiceBridge"
#define PCM_PACKET_SIZE     200

/* ============================================================================
 * BLE UUIDs (Same as ESP32 version for compatibility)
 * ============================================================================ */

/* Custom Service UUID: 00000000-0000-0000-0000-000000000000 */
#define LBS_UUID_SERVICE { \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
}

/* TX Characteristic UUID: 00000002-0000-1000-8000-00805f9b34fb (Notify) */
#define LBS_UUID_TX_CHAR { \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, \
}

/* RX Characteristic UUID: 00000003-0000-1000-8000-00805f9b34fb (Write) */
#define LBS_UUID_RX_CHAR { \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, \
}

/* ============================================================================
 * Global Variables
 * ============================================================================ */

static struct bt_conn *current_conn;
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_cb conn_callbacks;

static uint8_t tx_packet[PCM_PACKET_SIZE + 2];  /* +2 for header */
static uint8_t seq_num = 0;

/* Recording state */
static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;

/* Timer for audio simulation */
static struct k_work_delayable audio_timer;

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
 * Audio Timer Callback (simulates audio data)
 * ============================================================================ */

static void audio_timer_callback(struct k_work *work)
{
    int ret;
    
    /* Check recording state */
    if (recording_requested && !is_recording) {
        LOG_INF("Starting recording");
        is_recording = true;
        recording_requested = false;
        seq_num = 0;
    }
    
    if (stop_requested && is_recording) {
        LOG_INF("Stopping recording");
        is_recording = false;
        stop_requested = false;
    }
    
    /* Send data if recording and connected */
    if (is_recording && current_conn) {
        /* Create packet: [seq_num][0xAA][PCM data...] */
        tx_packet[0] = seq_num++;
        tx_packet[1] = 0xAA;  /* Sync byte */
        
        /* Fill with dummy audio data (simulating microphone) */
        for (int i = 2; i < sizeof(tx_packet); i++) {
            tx_packet[i] = (uint8_t)(k_cycle_get_32() & 0xFF);
        }
        
        /* Send via BLE Notify */
        ret = bt_gatt_notify(NULL, &voice_bridge_svc.attrs[2],
                            tx_packet, sizeof(tx_packet));
        if (ret < 0) {
            LOG_WRN("Notify failed: %d", ret);
        }
    }
    
    /* Schedule next timer (10ms = 100 packets/sec) */
    k_work_schedule((struct k_work_delayable *)work, K_MSEC(10));
}

/* ============================================================================
 * UART Command Handler
 * ============================================================================ */

static void uart_callback(const struct device *dev, struct uart_event *evt,
                          void *user_data)
{
    if (evt->type == UART_RX_RDY) {
        for (int i = 0; i < evt->data.rx.len; i++) {
            uint8_t c = evt->data.rx.buf[evt->data.rx.offset + i];
            
            if (c == 'r' || c == 'R') {
                LOG_INF("Serial: Start recording");
                recording_requested = true;
            } else if (c == 's' || c == 'S') {
                LOG_INF("Serial: Stop recording");
                stop_requested = true;
            } else if (c == 'h' || c == 'H') {
                printk("\nCommands: r=start, s=stop, h=help\n");
            }
        }
    }
}

/* ============================================================================
 * BLE Connection Callbacks
 * ============================================================================ */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed: %d", err);
        return;
    }
    
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
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

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        return;
    }
    
    LOG_INF("BLE initialized");
    
    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);
    
    /* Set device name */
    err = bt_set_name(BLE_DEVICE_NAME);
    if (err) {
        LOG_ERR("Failed to set name: %d", err);
        return;
    }
    
    /* Start advertising */
    struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };
    
    struct bt_data adv_data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    };
    
    err = bt_le_adv_start(&adv_param, adv_data, ARRAY_SIZE(adv_data),
                          NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed: %d", err);
        return;
    }
    
    LOG_INF("Advertising started: %s", BLE_DEVICE_NAME);
    
    /* Start audio timer */
    k_work_schedule(&audio_timer, K_MSEC(100));
}

/* ============================================================================
 * Main Application
 * ============================================================================ */

int main(void)
{
    int ret;
    
    printk("\n");
    printk("============================================\n");
    printk("Voice Bridge BLE - XIAO nRF52840 Sense\n");
    printk("============================================\n\n");
    
    LOG_INF("Starting Voice Bridge BLE");
    
    /* Initialize UART callback */
    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(uart_dev)) {
        uart_callback_set(uart_dev, uart_callback, NULL);
        uart_rx_enable(uart_dev, NULL, 256, 100);
        printk("Serial commands: r=start, s=stop, h=help\n\n");
    }
    
    /* Initialize audio timer */
    k_work_init_delayable(&audio_timer, audio_timer_callback);
    
    /* Initialize BLE */
    LOG_INF("Initializing BLE");
    ret = bt_enable(bt_ready);
    if (ret) {
        LOG_ERR("BLE enable failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Voice Bridge BLE ready");
    
    return 0;
}
