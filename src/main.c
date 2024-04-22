#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "driver/uart.h"
// #include "esp_bluedroid.h"
#include "driver/gpio.h"
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_system.h"
#include "esp_mac.h"
#include <string.h>
#include "esp_gatts_api.h" // Include for BLE GATT server API
// Constants for UART configuration
#define TX_PIN (UART_PIN_NO_CHANGE)    // Update with actual GPIO number
#define RX_PIN (UART_PIN_NO_CHANGE)    // Update with actual GPIO number
#define BT_TX_PIN (UART_PIN_NO_CHANGE) // Update with actual GPIO number
#define BT_RX_PIN (UART_PIN_NO_CHANGE) // Update with actual GPIO number

// Constants for UART configuration
#define UART_NUM_BLE UART_NUM_0    // UART port used for BLE communication
#define UART_NUM_SERIAL UART_NUM_1 // UART port used for serial communication
#define RX_BUF_SIZE 1024           // UART RX buffer size
#define TX_BUF_SIZE 1024           // UART TX buffer size
#define UART_NUM UART_NUM_0
#define UART_RX_PIN GPIO_NUM_3
#define UART_TX_PIN GPIO_NUM_1
static const char *TAG = "BLE_UART";
#define UART_BUF_SIZE 1024
// Global variables for BLE advertising data
static esp_ble_adv_data_t adv_data;
static esp_ble_adv_params_t adv_params;

// Task function declarations
void ble_task(void *pvParameters);
void uart_task(void *pvParameters);
// Define the structure for a GATT server profile instance
struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    esp_gatt_if_t gatts_if;
    uint16_t app_id;
    uint16_t service_handle;
    uint16_t char_handle;
    // Add other members as needed
};

// Declare the GATT server event handler function
esp_err_t gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Define the array of GATT server profile instances
#define PROFILE_NUM 1
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
        .service_handle = 0, // Initialize service handle
        .char_handle = 0,    // Initialize characteristic handle
        .app_id = 0,         // Assign a unique ID to the profile
        // Add other profile-specific configurations here
    }};
// Event handler for BLE GAP events
static void gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // Advertising data set complete event
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // Advertising start complete event
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
    default:
        break;
    }
}

// Initialize BLE
void initialize_ble()
{
    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    // Initialize Bluedroid library
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Register GAP callback function
    esp_ble_gap_register_callback(gap_handler);

    // Set BLE device name and advertising parameters
    esp_ble_gap_set_device_name("ROLAVI");

    // Configure advertising data
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x20;
    adv_data.max_interval = 0x40;
    adv_data.appearance = 0x00;
    adv_data.manufacturer_len = 0;
    adv_data.p_manufacturer_data = NULL;
    adv_data.service_data_len = 0;
    adv_data.p_service_data = NULL;
    adv_data.service_uuid_len = 0;
    adv_data.p_service_uuid = NULL;
    adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

    // Configure advertising parameters
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    // Set the advertising data
    esp_ble_gap_config_adv_data(&adv_data);
}

void app_main()
{
    // Initialize NVS
    nvs_flash_init();

    // Initialize BLE
    initialize_ble();

    // Create tasks for BLE and UART
    xTaskCreate(ble_task, "ble_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}

void uart_task(void *pvParameters)
{
    uint8_t data;
    while (true)
    {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM, &data, 1, portMAX_DELAY);
        if (len > 0)
        {
            // Print data received from UART
            printf("UART Received: %c\n", data);

            // Transmit data over Bluetooth
            esp_ble_gatts_send_indicate(0, 0, 0, len, &data, false);
        }
    }
}

void ble_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (!data)
    {
        ESP_LOGE("BLE_TASK", "Failed to allocate memory");
        vTaskDelete(NULL);
    }

    while (true)
    {
        // Wait for the next BLE event
        esp_ble_gatts_cb_param_t *param = NULL;
        QueueHandle_t gatts_event_queue = NULL;

        if (xQueueReceive(gatts_event_queue, &param, portMAX_DELAY))
        {
            // Check if it's a write event
            if (param->write.handle == gl_profile_tab[PROFILE_NUM].char_handle)
            {
                // Copy received data to buffer
                memcpy(data, param->write.value, param->write.len);
                int len = param->write.len;

                // Print data received from BLE
                printf("BLE Received: %.*s\n", len, (char *)data);

                // Transmit data over UART
                uart_write_bytes(UART_NUM, (const char *)data, len);
            }
        }
        // Free the event buffer
        free(param);
    }

    free(data);
}
