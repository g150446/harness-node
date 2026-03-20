/*
 * nrf54-speaker: BLE audio RX → I2S/MAX98357A playback
 *
 * Mac sends: [seq:1][0xAA:1][PCM bytes 240] via BLE Write Without Response
 * Firmware: ring-buffer → I2S feeder thread (mono → stereo upmix)
 */

#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(nrf54_speaker, LOG_LEVEL_INF);

/* ============================================================================
 * Audio constants
 * ============================================================================ */

#define I2S_NODE    DT_NODELABEL(i2s20)
#define SAMPLE_RATE 16000
#define BLOCK_SIZE  512   /* stereo 16-bit: 128 sample pairs = 8 ms */
#define MONO_BYTES  256   /* mono side: 128 samples */

/* ============================================================================
 * Ring buffer / sync primitives
 * ============================================================================ */

RING_BUF_DECLARE(audio_ring, 16384);
K_MUTEX_DEFINE(ring_mutex);
K_SEM_DEFINE(audio_sem, 0, 1);
K_MEM_SLAB_DEFINE(i2s_tx_slab, BLOCK_SIZE, 8, 4);

/* ============================================================================
 * BLE UUIDs
 *
 * Speaker Service:   00000020-0000-1000-8000-00805f9b34fb
 * Audio RX char:     00000021-0000-1000-8000-00805f9b34fb  (Write | WWR)
 * Build Info char:   00000022-0000-1000-8000-00805f9b34fb  (Read)
 * ============================================================================ */

#define SPEAKER_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00

#define SPEAKER_UUID_AUDIO_RX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00

#define SPEAKER_UUID_BUILD_INFO_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00

#define BLE_DEVICE_NAME "SpeakerBridge"

/* ============================================================================
 * BLE callbacks
 * ============================================================================ */

static const char build_timestamp[] = __DATE__ " " __TIME__;

static ssize_t read_build_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             build_timestamp, strlen(build_timestamp));
}

static struct bt_conn *current_conn;

static ssize_t audio_rx_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags)
{
    static uint32_t rx_count;
    const uint8_t *data = buf;

    /* Packet format: [seq:1][0xAA:1][PCM ...] */
    if (len < 2 || data[1] != 0xAA) {
        LOG_WRN("bad packet len=%u magic=0x%02x", len, len >= 2 ? data[1] : 0);
        return len;
    }

    k_mutex_lock(&ring_mutex, K_FOREVER);
    ring_buf_put(&audio_ring, &data[2], len - 2);
    k_mutex_unlock(&ring_mutex);
    k_sem_give(&audio_sem);

    rx_count++;
    if (rx_count % 500 == 1) {
        LOG_INF("audio RX: %u pkts, ring=%u/%u bytes", rx_count,
                ring_buf_capacity_get(&audio_ring) - ring_buf_space_get(&audio_ring),
                ring_buf_capacity_get(&audio_ring));
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(speaker_svc,
    BT_GATT_PRIMARY_SERVICE(
        BT_UUID_DECLARE_128(SPEAKER_UUID_SERVICE)),

    /* Audio RX Characteristic (Write | Write Without Response) */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(SPEAKER_UUID_AUDIO_RX_CHAR),
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, audio_rx_write_cb, NULL),

    /* Build Info Characteristic (Read) */
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_128(SPEAKER_UUID_BUILD_INFO_CHAR),
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_build_info, NULL, NULL),
);

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE connection failed: %d", err);
        return;
    }
    LOG_INF("BLE connected");
    current_conn = conn;

    /* Flush ring buffer so every connection starts from empty.
     * Prevents stale data from a previous session from blocking new writes. */
    k_mutex_lock(&ring_mutex, K_FOREVER);
    ring_buf_reset(&audio_ring);
    k_mutex_unlock(&ring_mutex);

    /* Request short connection interval for audio throughput.
     * interval_min/max in units of 1.25ms: 6=7.5ms, 8=10ms */
    int r = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 8, 0, 400));
    if (r) {
        LOG_WRN("conn param update failed: %d", r);
    }
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

/* ============================================================================
 * I2S helpers
 * ============================================================================ */

static int i2s_prefill_and_start(const struct device *dev)
{
    int ret;

    for (int i = 0; i < 4; i++) {
        void *mem;
        ret = k_mem_slab_alloc(&i2s_tx_slab, &mem, K_MSEC(100));
        if (ret < 0) {
            LOG_ERR("slab alloc for prefill failed: %d", ret);
            return ret;
        }
        memset(mem, 0, BLOCK_SIZE);
        ret = i2s_write(dev, mem, BLOCK_SIZE);
        if (ret < 0) {
            LOG_ERR("i2s_write prefill failed: %d", ret);
            k_mem_slab_free(&i2s_tx_slab, mem);
            return ret;
        }
    }

    ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("I2S START failed: %d", ret);
    }
    return ret;
}

static int i2s_recover(const struct device *dev)
{
    int ret;

    LOG_WRN("I2S error — recovering");

    /* Drop any in-flight buffers, return to READY state */
    ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("I2S DROP failed: %d", ret);
    }

    ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
    if (ret < 0) {
        LOG_ERR("I2S PREPARE failed: %d", ret);
        return ret;
    }

    return i2s_prefill_and_start(dev);
}

/* ============================================================================
 * I2S feeder thread
 * ============================================================================ */

static K_THREAD_STACK_DEFINE(feeder_stack, 2048);
static struct k_thread feeder_tid;

static void i2s_feeder_thread(void *p1, void *p2, void *p3)
{
    const struct device *dev = (const struct device *)p1;
    static int16_t mono_buf[MONO_BYTES / sizeof(int16_t)];

    while (1) {
        void *slab;
        int ret = k_mem_slab_alloc(&i2s_tx_slab, &slab, K_MSEC(200));
        if (ret < 0) {
            /* All slabs stuck in stalled I2S driver — force recovery */
            LOG_ERR("slab alloc timeout, recovering I2S");
            i2s_recover(dev);
            continue;
        }

        /* Wait for data; timeout → underrun → fill with silence */
        k_sem_take(&audio_sem, K_MSEC(8));

        k_mutex_lock(&ring_mutex, K_FOREVER);
        uint32_t got = ring_buf_get(&audio_ring, (uint8_t *)mono_buf, MONO_BYTES);
        k_mutex_unlock(&ring_mutex);

        if (got < MONO_BYTES) {
            memset((uint8_t *)mono_buf + got, 0, MONO_BYTES - got);
        }

        /* Upmix mono → stereo */
        int16_t *stereo = (int16_t *)slab;
        for (int i = 0; i < (int)(MONO_BYTES / sizeof(int16_t)); i++) {
            stereo[i * 2]     = mono_buf[i];
            stereo[i * 2 + 1] = mono_buf[i];
        }

        ret = i2s_write(dev, slab, BLOCK_SIZE);
        if (ret < 0) {
            k_mem_slab_free(&i2s_tx_slab, slab);
            i2s_recover(dev);
        }
    }
}

/* ============================================================================
 * main
 * ============================================================================ */

int main(void)
{
    int ret;

    LOG_INF("nrf54-speaker BLE audio bridge (build: %s)", build_timestamp);

    /* --- I2S configure (do NOT start yet — wait until BLE is up) --- */
    const struct device *i2s_dev = DEVICE_DT_GET(I2S_NODE);
    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("I2S device not ready");
        return -1;
    }

    struct i2s_config cfg = {
        .word_size      = 16,
        .channels       = 2,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
        .frame_clk_freq = SAMPLE_RATE,
        .mem_slab       = &i2s_tx_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 1000,
    };

    ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
    if (ret < 0) {
        LOG_ERR("I2S configure failed: %d", ret);
        return ret;
    }
    LOG_INF("I2S configured at %d Hz", SAMPLE_RATE);

    /* --- BLE init first so I2S starts with a full feeder behind it --- */
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
                      0x00, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00),
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
    LOG_INF("BLE advertising as %s", BLE_DEVICE_NAME);

    /* --- Start feeder thread --- */
    k_thread_create(&feeder_tid, feeder_stack, K_THREAD_STACK_SIZEOF(feeder_stack),
                    i2s_feeder_thread, (void *)i2s_dev, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&feeder_tid, "i2s_feeder");

    /* --- Pre-fill silence and start I2S now that feeder is running --- */
    ret = i2s_prefill_and_start(i2s_dev);
    if (ret < 0) {
        return ret;
    }
    LOG_INF("I2S started");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
