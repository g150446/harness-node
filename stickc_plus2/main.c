#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2s_pdm.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "adpcm.h"
#include "smp_ota.h"

static const char *TAG = "hn_plus2";

/* M5StickC Plus2 pins (M5Unified / docs). */
#define POWER_HOLD_GPIO   4
#define PDM_CLK_GPIO      0
#define PDM_DIN_GPIO      34
#define BUTTON_A_GPIO     37
#define LED_GPIO          19

#define BUTTON_POLL_MS          10
#define BUTTON_DEBOUNCE_MS      30
#define BUTTON_DOUBLE_CLICK_MS  500

#define I2S_PORT_NUM      I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define DMA_BUFFER_COUNT  8
#define DMA_FRAME_COUNT   256
/* ESP32 PDM is much quieter than nRF DMIC; match M5Unified magnification. */
#define MIC_GAIN          16
#define MIC_START_DISCARD_MS  400
#define MIC_DC_SHIFT      5   /* offset IIR: offset -= (x + offset) >> MIC_DC_SHIFT */

#define BLE_MTU_SIZE      512
#define BLE_ADV_NAME      "HarnessNode-Plus2"

#define RECORDING_PACKET_SIZE  200
#define AUDIO_PACKET_SYNC_BYTE 0xAA
#define EVENT_PACKET_SYNC_BYTE 0x55
#define EVENT_RECORDING_STARTED 0x01
#define EVENT_RECORDING_STOPPED 0x02
#define EVENT_DOUBLE_CLICK 0x12
#define EVENT_SINGLE_CLICK 0x14
#define COMMAND_SET_OPERATION_MODE 0x05
#define EVENT_OPERATION_MODE 0x40
#define OPERATION_MODE_NORMAL 0x00
#define OPERATION_MODE_DRIVING 0x01
#define OPERATION_MODE_PENDING_NONE 0xff

#define UART_PORT_NUM     UART_NUM_0
#define UART_BAUD_RATE    115200

/* HarnessNode UUIDs (LSB-first). */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_tx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00);
static const ble_uuid128_t gatt_svr_rx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00);

static uint16_t audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t audio_tx_char_handle;
static uint16_t audio_rx_char_handle;

static adpcm_state_t adpcm_enc_state;
static uint8_t seq_num = 0;

static TaskHandle_t audio_task_handle;
static TaskHandle_t button_task_handle;

static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;
static volatile bool stop_event_pending = false;
static volatile uint8_t operation_mode = OPERATION_MODE_NORMAL;
static volatile uint8_t operation_mode_pending = OPERATION_MODE_PENDING_NONE;

static i2s_chan_handle_t i2s_rx_handle = NULL;
static bool mic_enabled = false;
static int32_t mic_dc_offset = 0;
static uint32_t mic_level_log_left = 0;

static void ble_app_advertise(void);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void request_recording_stop(const char *source);
static void on_ota_busy(bool busy);
static esp_err_t set_microphone_enabled(bool enabled);
static void audio_stream_task(void *pvParameters);
static void button_task(void *pvParameters);
static void uart_task(void *pvParameters);
static void send_status_event(uint8_t event_code, const char *event_name);
static void send_operation_mode_status(void);
static void apply_pending_operation_mode(void);
static esp_err_t set_microphone_enabled(bool enabled);

static void power_hold_on(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << POWER_HOLD_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(POWER_HOLD_GPIO, 1);
}

static void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

static void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_set(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    led_set(false);
}

static void request_recording_start(const char *source)
{
    if (is_recording || recording_requested) {
        return;
    }
    ESP_LOGI(TAG, "%s: Start recording requested", source);
    stop_requested = false;
    recording_requested = true;
}

static void request_recording_stop(const char *source)
{
    if (!is_recording && !recording_requested) {
        return;
    }
    ESP_LOGI(TAG, "%s: Stop recording requested", source);
    bool was_recording = is_recording;
    recording_requested = false;
    is_recording = false;
    stop_requested = true;
    stop_event_pending = was_recording;
    led_set(false);
}

static void on_ota_busy(bool busy)
{
    if (busy) {
        request_recording_stop("OTA");
        ESP_LOGI(TAG, "OTA busy: recording blocked");
    }
}

static void emulate_single_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Single-click", source);

    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "%s: Single-click ignored: no BLE connection", source);
        return;
    }

    if (is_recording) {
        request_recording_stop(source);
    } else if (recording_requested) {
        recording_requested = false;
        ESP_LOGI(TAG, "%s: Pending recording start cancelled", source);
        apply_pending_operation_mode();
    } else {
        request_recording_start(source);
    }

    send_status_event(EVENT_SINGLE_CLICK, "single_click");
}

static void emulate_double_click(const char *source)
{
    ESP_LOGI(TAG, "%s: Double-click (notify only)", source);
    send_status_event(EVENT_DOUBLE_CLICK, "double_click");
}

static void handle_serial_command(uint8_t command, const char *source)
{
    if (command == 'r' || command == 'R') {
        request_recording_start(source);
    } else if (command == 's' || command == 'S') {
        request_recording_stop(source);
    } else if (command == 'c' || command == 'C' || command == '1') {
        emulate_single_click(source);
    } else if (command == 'd' || command == 'D' || command == '2') {
        emulate_double_click(source);
    } else if (command == 'h' || command == 'H') {
        ESP_LOGI(TAG,
                 "%s commands: 'r'=start, 's'=stop, "
                 "'c'/'1'=single-click, 'd'/'2'=double-click, 'h'=help",
                 source);
    }
}

static void send_status_event(uint8_t event_code, const char *event_name)
{
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    const uint8_t packet[] = {0x00, EVENT_PACKET_SYNC_BYTE, event_code};
    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, sizeof(packet));
    if (om == NULL) {
        ESP_LOGW(TAG, "Failed to allocate status event buffer for %s", event_name);
        return;
    }

    int rc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send status event %s: rc=%d", event_name, rc);
        return;
    }

    ESP_LOGI(TAG, "Sent status event: %s", event_name);
}

static void send_operation_mode_status(void)
{
    if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    const uint8_t packet[] = {
        0x00,
        EVENT_PACKET_SYNC_BYTE,
        EVENT_OPERATION_MODE,
        operation_mode,
        operation_mode_pending,
    };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, sizeof(packet));
    if (om == NULL) {
        ESP_LOGW(TAG, "Failed to allocate operation mode status buffer");
        return;
    }

    int rc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send operation mode status: rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Sent operation mode status: effective=%u pending=%u",
             operation_mode, operation_mode_pending);
}

static void apply_pending_operation_mode(void)
{
    uint8_t requested = operation_mode_pending;

    if (requested == OPERATION_MODE_PENDING_NONE) {
        return;
    }
    if (requested == operation_mode) {
        operation_mode_pending = OPERATION_MODE_PENDING_NONE;
        send_operation_mode_status();
        return;
    }
    if (requested != OPERATION_MODE_NORMAL &&
        requested != OPERATION_MODE_DRIVING) {
        operation_mode_pending = OPERATION_MODE_PENDING_NONE;
        send_operation_mode_status();
        return;
    }
    if (is_recording || recording_requested) {
        return;
    }

    operation_mode = requested;
    operation_mode_pending = OPERATION_MODE_PENDING_NONE;
    ESP_LOGI(TAG, "Operation mode: %s",
             operation_mode == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL");
    send_operation_mode_status();
}

static int
audio_gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_rx_uuid.u) == 0) {
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
            uint8_t data[2] = {0};
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om) < sizeof(data)
                                    ? OS_MBUF_PKTLEN(ctxt->om)
                                    : sizeof(data);
            if (os_mbuf_copydata(ctxt->om, 0, data_len, data) != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            uint8_t cmd = data[0];
            if (cmd == 0x01) {
                request_recording_start("BLE");
            } else if (cmd == 0x00) {
                request_recording_stop("BLE");
            } else if (cmd == COMMAND_SET_OPERATION_MODE && data_len >= 2) {
                uint8_t requested = data[1];
                if (requested == OPERATION_MODE_NORMAL ||
                    requested == OPERATION_MODE_DRIVING) {
                    operation_mode_pending = requested;
                    ESP_LOGI(TAG, "Operation mode request: %s%s",
                             requested == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL",
                             (is_recording || recording_requested) ? " (pending)" : "");
                    send_operation_mode_status();
                    apply_pending_operation_mode();
                } else {
                    ESP_LOGW(TAG, "Invalid operation mode: %u", requested);
                    send_operation_mode_status();
                }
            }
        }
        return 0;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_tx_uuid.u) == 0) {
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_tx_uuid.u,
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &audio_tx_char_handle,
            },
            {
                .uuid = &gatt_svr_rx_uuid.u,
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &audio_rx_char_handle,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

static void
ble_app_advertise(void)
{
    ble_svc_gap_device_name_set(BLE_ADV_NAME);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_uuid128_t adv_uuids128[] = { gatt_svr_svc_uuid };
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = adv_uuids128;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (uint8_t *)BLE_ADV_NAME;
    rsp_fields.name_len = strlen(BLE_ADV_NAME);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan rsp fields: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started: %s", BLE_ADV_NAME);
}

static void
ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE synced");
    ble_app_advertise();
}

static void
ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset, reason=%d", reason);
}

static int
ble_app_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status != 0) {
            ble_app_advertise();
            return 0;
        }
        audio_conn_handle = event->connect.conn_handle;
        ble_att_set_preferred_mtu(BLE_MTU_SIZE);
        ble_gattc_exchange_mtu(audio_conn_handle, NULL, NULL);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected");
        audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_recording = false;
        recording_requested = false;
        stop_requested = false;
        stop_event_pending = false;
        led_set(false);
        (void)set_microphone_enabled(false);
        apply_pending_operation_mode();
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU exchange: mtu=%d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == audio_tx_char_handle) {
            ESP_LOGI(TAG, "Client %s notifications",
                     event->subscribe.cur_notify ? "enabled" : "disabled");
            if (event->subscribe.cur_notify) {
                send_operation_mode_status();
            }
        }
        break;
    }

    return 0;
}

static esp_err_t set_microphone_enabled(bool enabled)
{
    if (i2s_rx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enabled == mic_enabled) {
        return ESP_OK;
    }

    esp_err_t ret;
    if (enabled) {
        ret = i2s_channel_enable(i2s_rx_handle);
    } else {
        ret = i2s_channel_disable(i2s_rx_handle);
    }
    if (ret == ESP_OK) {
        mic_enabled = enabled;
    }
    return ret;
}

static esp_err_t init_audio(void)
{
    ESP_LOGI(TAG, "Initializing PDM microphone (G0/G34)...");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUFFER_COUNT;
    chan_cfg.dma_frame_num = DMA_FRAME_COUNT;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                   I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_GPIO,
            .din = PDM_DIN_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    /* M5 default uses right slot for SPM1423. */
    pdm_cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_RIGHT;

    ret = i2s_channel_init_pdm_rx_mode(i2s_rx_handle, &pdm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Keep disabled until recording starts. */
    mic_enabled = false;
    ESP_LOGI(TAG, "PDM microphone ready");
    return ESP_OK;
}

static esp_err_t init_button(void)
{
    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_A_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE, /* board has external pull-up; G37 input-only */
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&button_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "button gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Button A on GPIO %d (active low)", BUTTON_A_GPIO);
    return ESP_OK;
}

static void audio_stream_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Audio streaming task started");

    esp_err_t audio_ret = init_audio();
    if (audio_ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio init failed (%s) — audio task exiting",
                 esp_err_to_name(audio_ret));
        vTaskDelete(NULL);
        return;
    }

    size_t i2s_buffer_bytes = DMA_FRAME_COUNT * sizeof(int16_t);
    int16_t *i2s_buffer = (int16_t *)malloc(i2s_buffer_bytes);
    uint8_t *tx_packet = (uint8_t *)malloc(RECORDING_PACKET_SIZE + 2);

    if (i2s_buffer == NULL || tx_packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read = 0;

    while (1) {
        if (recording_requested && !is_recording) {
            ESP_LOGI(TAG, "Starting recording...");
            if (set_microphone_enabled(true) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to enable microphone");
                recording_requested = false;
                continue;
            }
            /* Discard startup frames while PDM DC settles (nordic/M5-like). */
            mic_dc_offset = 0;
            {
                int64_t t0 = esp_timer_get_time();
                while ((esp_timer_get_time() - t0) < (int64_t)MIC_START_DISCARD_MS * 1000) {
                    if (i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes,
                                         &bytes_read, pdMS_TO_TICKS(50)) != ESP_OK) {
                        break;
                    }
                    size_t n = bytes_read / sizeof(int16_t);
                    for (size_t i = 0; i < n; i++) {
                        int32_t x = i2s_buffer[i];
                        mic_dc_offset += (x - mic_dc_offset) >> MIC_DC_SHIFT;
                    }
                }
            }
            mic_level_log_left = I2S_SAMPLE_RATE; /* log peak/rms for ~1 s */

            is_recording = true;
            recording_requested = false;
            seq_num = 0;
            led_set(true);
            send_status_event(EVENT_RECORDING_STARTED, "recording_started");
        }

        if (stop_requested) {
            if (stop_event_pending) {
                ESP_LOGI(TAG, "Stopping recording...");
                send_status_event(EVENT_RECORDING_STOPPED, "recording_stopped");
            }
            (void)set_microphone_enabled(false);
            stop_requested = false;
            stop_event_pending = false;
            led_set(false);
            apply_pending_operation_mode();
        }

        if (!is_recording || audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        esp_err_t ret = i2s_channel_read(i2s_rx_handle, i2s_buffer, i2s_buffer_bytes,
                                         &bytes_read, pdMS_TO_TICKS(100));
        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }
        if (stop_requested || !is_recording) {
            continue;
        }

        size_t samples_read = bytes_read / sizeof(int16_t);

        /* DC remove + digital gain (ESP32 PDM is quiet vs nRF / needs M5-like mag). */
        int32_t peak = 0;
        int64_t sum_sq = 0;
        for (size_t i = 0; i < samples_read; i++) {
            int32_t x = (int32_t)i2s_buffer[i];
            mic_dc_offset += (x - mic_dc_offset) >> MIC_DC_SHIFT;
            int32_t centered = x - mic_dc_offset;
            int32_t v = centered * MIC_GAIN;
            if (v > INT16_MAX) {
                v = INT16_MAX;
            } else if (v < INT16_MIN) {
                v = INT16_MIN;
            }
            i2s_buffer[i] = (int16_t)v;
            int32_t a = v < 0 ? -v : v;
            if (a > peak) {
                peak = a;
            }
            sum_sq += (int64_t)v * (int64_t)v;
        }
        if (mic_level_log_left > 0) {
            uint32_t n = samples_read < mic_level_log_left
                             ? (uint32_t)samples_read
                             : mic_level_log_left;
            mic_level_log_left -= n;
            if (mic_level_log_left == 0 && samples_read > 0) {
                int32_t rms = (int32_t)sqrt((double)sum_sq / (double)samples_read);
                ESP_LOGI(TAG, "Mic level peak=%d rms=%d gain=%d", (int)peak, (int)rms, MIC_GAIN);
            }
        }

        size_t sample_index = 0;
        while (sample_index < samples_read) {
            size_t samples_to_send =
                (samples_read - sample_index < RECORDING_PACKET_SIZE / 2)
                    ? (samples_read - sample_index)
                    : (RECORDING_PACKET_SIZE / 2);

            tx_packet[0] = seq_num++;
            tx_packet[1] = AUDIO_PACKET_SYNC_BYTE;
            memcpy(&tx_packet[2], &i2s_buffer[sample_index], samples_to_send * 2);
            size_t packet_size = 2 + (samples_to_send * 2);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
            if (om != NULL) {
                int nrc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
                if (nrc != 0) {
                    /* Retry once after a short backoff; drop frame if still failing. */
                    vTaskDelay(pdMS_TO_TICKS(2));
                    om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
                    if (om != NULL) {
                        nrc = ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
                        if (nrc != 0) {
                            ESP_LOGW(TAG, "notify failed rc=%d (drop)", nrc);
                        }
                    }
                }
            }

            sample_index += samples_to_send;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

static void button_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Button task started");

    int stable_level = gpio_get_level(BUTTON_A_GPIO);
    int last_level = stable_level;
    TickType_t last_change_tick = xTaskGetTickCount();
    TickType_t first_click_tick = 0;
    uint8_t click_count = 0;

    while (1) {
        int level = gpio_get_level(BUTTON_A_GPIO);
        TickType_t now = xTaskGetTickCount();

        if (level != last_level) {
            last_level = level;
            last_change_tick = now;
        }

        if (level != stable_level &&
            (now - last_change_tick) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            stable_level = level;
            if (stable_level == 0) {
                if (click_count == 0) {
                    click_count = 1;
                    first_click_tick = now;
                } else {
                    TickType_t click_delta = now - first_click_tick;
                    if (click_delta <= pdMS_TO_TICKS(BUTTON_DOUBLE_CLICK_MS)) {
                        emulate_double_click("Button A double-click");
                        click_count = 0;
                        first_click_tick = 0;
                    } else {
                        click_count = 1;
                        first_click_tick = now;
                    }
                }
            }
        }

        if (click_count == 1 &&
            (now - first_click_tick) > pdMS_TO_TICKS(BUTTON_DOUBLE_CLICK_MS)) {
            click_count = 0;
            first_click_tick = 0;
            emulate_single_click("Button A");
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void uart_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Serial command task started");

    uint8_t uart_buf[256];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, uart_buf, sizeof(uart_buf),
                                  pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                handle_serial_command(uart_buf[i], "UART");
            }
        }
    }
}

static void ble_host_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
}

void app_main(void)
{
    /* Must hold power immediately on StickC Plus2. */
    power_hold_on();
    ESP_LOGI(TAG, "HarnessNode Plus2 (M5StickC Plus2) - Starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    smp_ota_init();
    smp_ota_set_busy_callback(on_ota_busy);

    adpcm_init_state(&adpcm_enc_state);
    led_init();

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(init_button());

    ESP_LOGI(TAG, "Initializing NimBLE...");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_count_cfg(smp_ota_svc_defs());
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_gatts_add_svcs(smp_ota_svc_defs());
    nimble_port_freertos_init(ble_host_task);

    xTaskCreatePinnedToCore(audio_stream_task, "audio_stream", 8192, NULL, 5,
                            &audio_task_handle, 1);
    xTaskCreate(button_task, "button_task", 3072, NULL, 4, &button_task_handle);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Init complete. Adv name %s; BtnA single toggles recording; SMP OTA enabled",
             BLE_ADV_NAME);
    ESP_LOGI(TAG, "Serial: 'r'=start 's'=stop 'c'=single 'd'=double 'h'=help");
}
