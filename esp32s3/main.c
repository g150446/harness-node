#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "driver/i2s_pdm.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"

#include "adpcm.h"

static const char *TAG = "voice_bridge";

// PDM Microphone pins (XIAO ESP32S3 Sense)
#define PDM_CLK_GPIO      42
#define PDM_DATA_GPIO     41

// I2S configuration for PDM microphone
#define I2S_SAMPLE_RATE   24000  // Increased from 16kHz to 24kHz for better quality
#define I2S_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT
#define I2S_CHANNEL_MONO  1

// DMA buffer configuration
#define DMA_BUFFER_COUNT  8
#define DMA_BUFFER_SIZE   1024  // bytes

// BLE configuration
#define BLE_MTU_SIZE      512
#define BLE_ADV_NAME      "VoiceBridge"

// Recording configuration
#define RECORDING_PACKET_SIZE  200  // bytes of PCM data per packet (fits in BLE MTU)

// BLE UUIDs
static ble_uuid128_t gatt_svr_svc_sec_uuid = BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00,
                                                              0x00, 0x00, 0x00, 0x00,
                                                              0x00, 0x00, 0x00, 0x00,
                                                              0x00, 0x00, 0x00, 0x00);

#define AUDIO_SERVICE_UUID        0x0001
#define AUDIO_TX_CHAR_UUID        0x0002  // Microphone -> Mac (Notify)
#define AUDIO_RX_CHAR_UUID        0x0003  // Mac -> XIAO (Write for control)

// UART configuration
#define UART_PORT_NUM     UART_NUM_0
#define UART_BAUD_RATE    115200
#define UART_TX_PIN       UART_PIN_NO_CHANGE
#define UART_RX_PIN       UART_PIN_NO_CHANGE

static uint16_t audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t audio_tx_char_handle;
static uint16_t audio_rx_char_handle;

// ADPCM state
static adpcm_state_t adpcm_enc_state;

// Sequence number for packet ordering
static uint8_t seq_num = 0;

// Task handles
static TaskHandle_t audio_task_handle;

// Recording state
static volatile bool is_recording = false;
static volatile bool recording_requested = false;
static volatile bool stop_requested = false;

// PDM handle
static i2s_chan_handle_t pdm_rx_handle = NULL;

// Forward declarations
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void audio_stream_task(void *pvParameters);
static void uart_task(void *pvParameters);

// ============================================================================
// BLE GATT Server Callbacks
// ============================================================================

static int
audio_gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

    switch (uuid) {
    case AUDIO_RX_CHAR_UUID:
        ESP_LOGD(TAG, "Received control command, len=%d", ctxt->om->om_len);
        // Handle control commands from Mac
        if (ctxt->om->om_len >= 1) {
            uint8_t cmd = ctxt->om->om_data[0];
            if (cmd == 0x01) {
                ESP_LOGI(TAG, "Start recording command received");
                recording_requested = true;
            } else if (cmd == 0x00) {
                ESP_LOGI(TAG, "Stop recording command received");
                stop_requested = true;
            }
        }
        break;
    case AUDIO_TX_CHAR_UUID:
        ESP_LOGD(TAG, "TX characteristic accessed");
        break;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_sec_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(AUDIO_TX_CHAR_UUID),
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &audio_tx_char_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(AUDIO_RX_CHAR_UUID),
                .access_cb = audio_gatt_svr_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &audio_rx_char_handle,
            },
            {
                0, /* No more characteristics */
            },
        },
    },
    {
        0, /* No more services */
    },
};

// ============================================================================
// BLE GAP Callbacks
// ============================================================================

static void
ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE synced");

    // Set device name
    ble_svc_gap_device_name_set(BLE_ADV_NAME);

    // Configure advertising
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_set_fields(&(const struct ble_hs_adv_fields){
        .name = (uint8_t *)BLE_ADV_NAME,
        .name_len = strlen(BLE_ADV_NAME),
        .name_is_complete = 1,
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
    });
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
        return;
    }

    // Start advertising
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started: %s", BLE_ADV_NAME);
}

static void
ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset, reason=%d", reason);
}

static int
ble_app_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s", event->connect.status == 0 ? "established" : "failed");
        if (event->connect.status != 0) {
            return 0;
        }
        audio_conn_handle = event->connect.conn_handle;

        // Request MTU size
        ble_att_set_preferred_mtu(BLE_MTU_SIZE);
        ble_gattc_exchange_mtu(audio_conn_handle, NULL, NULL);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected");
        audio_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_recording = false;
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU exchange: mtu=%d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == audio_tx_char_handle) {
            ESP_LOGI(TAG, "Client %s notifications",
                     event->subscribe.cur_notify ? "enabled" : "disabled");
        }
        break;
    }

    return 0;
}

// ============================================================================
// PDM Microphone Initialization
// ============================================================================

static esp_err_t init_pdm_microphone(void)
{
    ESP_LOGI(TAG, "Initializing PDM microphone...");

    // Create I2S PDM RX channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &pdm_rx_handle));

    // Configure PDM RX (microphone input)
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_BITS_PER_SAMPLE, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_GPIO,
            .din = PDM_DATA_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(pdm_rx_handle, &pdm_rx_cfg));

    ESP_LOGI(TAG, "PDM microphone initialized: CLK=%d, DATA=%d, SampleRate=%d",
             PDM_CLK_GPIO, PDM_DATA_GPIO, I2S_SAMPLE_RATE);

    return ESP_OK;
}

// ============================================================================
// Audio Streaming Task
// ============================================================================

static void audio_stream_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Audio streaming task started");

    // Initialize PDM microphone
    ESP_ERROR_CHECK(init_pdm_microphone());
    
    // Enable I2S channel
    ESP_ERROR_CHECK(i2s_channel_enable(pdm_rx_handle));

    // Allocate buffers
    int16_t *pcm_buffer = (int16_t *)malloc(DMA_BUFFER_SIZE);
    uint8_t *tx_packet = (uint8_t *)malloc(RECORDING_PACKET_SIZE + 2);  // +2 for header

    if (pcm_buffer == NULL || tx_packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        vTaskDelete(NULL);
    }

    size_t bytes_read;

    ESP_LOGI(TAG, "Waiting for audio data...");

    while (1) {
        // Check for recording start command
        if (recording_requested && !is_recording) {
            ESP_LOGI(TAG, "Starting recording...");
            is_recording = true;
            recording_requested = false;
            seq_num = 0;
        }

        // Check for recording stop command
        if (stop_requested && is_recording) {
            ESP_LOGI(TAG, "Stopping recording...");
            is_recording = false;
            stop_requested = false;
        }

        // Read from PDM microphone
        esp_err_t ret = i2s_channel_read(pdm_rx_handle, pcm_buffer, DMA_BUFFER_SIZE,
                                         &bytes_read, portMAX_DELAY);

        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }

        // Send data if recording OR if connected (for testing)
        if (audio_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }

        // Process and send PCM data via BLE
        size_t samples_read = bytes_read / 2;  // 16-bit samples
        size_t sample_index = 0;

        while (sample_index < samples_read) {
            // Calculate how many samples to send in this packet
            size_t samples_to_send = (samples_read - sample_index < RECORDING_PACKET_SIZE / 2) ?
                                     (samples_read - sample_index) : (RECORDING_PACKET_SIZE / 2);

            // Create packet: [seq_num][0xAA][PCM data...]
            tx_packet[0] = seq_num++;
            tx_packet[1] = 0xAA;  // Sync byte

            // Copy PCM data
            memcpy(&tx_packet[2], &pcm_buffer[sample_index], samples_to_send * 2);

            size_t packet_size = 2 + (samples_to_send * 2);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(tx_packet, packet_size);
            if (om != NULL) {
                ble_gattc_notify_custom(audio_conn_handle, audio_tx_char_handle, om);
            }

            sample_index += samples_to_send;

            // Small delay to avoid overwhelming BLE
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    free(pcm_buffer);
    free(tx_packet);
    vTaskDelete(NULL);
}

// ============================================================================
// UART Task (for serial commands)
// ============================================================================

static void uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART task started");

    uint8_t uart_buf[256];
    
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, uart_buf, sizeof(uart_buf), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Process serial commands
            for (int i = 0; i < len; i++) {
                if (uart_buf[i] == 'r' || uart_buf[i] == 'R') {
                    ESP_LOGI(TAG, "Serial: Start recording");
                    recording_requested = true;
                } else if (uart_buf[i] == 's' || uart_buf[i] == 'S') {
                    ESP_LOGI(TAG, "Serial: Stop recording");
                    stop_requested = true;
                } else if (uart_buf[i] == 'h' || uart_buf[i] == 'H') {
                    ESP_LOGI(TAG, "Serial commands: 'r'=start, 's'=stop, 'h'=help");
                }
            }
        }
    }
}

// ============================================================================
// BLE Host Task
// ============================================================================

static void
ble_host_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
}

// ============================================================================
// Application Entry Point
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Voice Bridge BLE - Starting");

    // Initialize ADPCM encoder state (for future use)
    adpcm_init_state(&adpcm_enc_state);

    // Initialize UART
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
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized");

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing NimBLE...");

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    // Set security (no pairing required)
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    ESP_LOGI(TAG, "Calling nimble_port_init...");
    nimble_port_init();

    ESP_LOGI(TAG, "Initializing GATT server...");
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ESP_LOGI(TAG, "Starting BLE host task...");
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Creating audio streaming task...");
    xTaskCreatePinnedToCore(audio_stream_task, "audio_stream", 8192, NULL, 5,
                           &audio_task_handle, 1);

    ESP_LOGI(TAG, "Creating UART task...");
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Voice Bridge BLE - Initialization complete");
    ESP_LOGI(TAG, "Serial commands: 'r'=start recording, 's'=stop, 'h'=help");
}
