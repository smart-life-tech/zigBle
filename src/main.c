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
// Constants for UART configuration
#define TX_PIN (UART_PIN_NO_CHANGE)    // Update with actual GPIO number
#define RX_PIN (UART_PIN_NO_CHANGE)    // Update with actual GPIO number
#define BT_TX_PIN (UART_PIN_NO_CHANGE) // Update with actual GPIO number
#define BT_RX_PIN (UART_PIN_NO_CHANGE) // Update with actual GPIO number
#define RX_BUF_SIZE (1024)
#define TX_BUF_SIZE (1024)

// Global variables for BLE and UART
esp_ble_adv_data_t adv_data = {};
esp_ble_adv_params_t adv_params = {};

// Task function declarations
void read_serial_task(void *pvParameters);
void read_ble_task(void *pvParameters);

// Function to initialize Bluetooth LE
void initialize_bluetooth()
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    /*esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
*/
    uint8_t mac[6];
    esp_read_mac(mac, ESP_IF_WIFI_STA); // Get Wi-Fi MAC address

    // Set the Bluetooth base MAC address (Wi-Fi MAC address + 2)
    mac[5] += 2; // Increment the last byte of MAC address

    esp_base_mac_addr_set(mac);

    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Set BLE device name
    esp_ble_gap_set_device_name("ESP32_BLE_Device");

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
    adv_data.service_uuid_len = 16;
    adv_data.p_service_uuid = NULL;
    adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

    // Configure advertising parameters
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_start_advertising(&adv_params);
}

// Function to initialize UART communication
void initialize_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB}; // Specify the source clock};

    // Configure UART for regular serial
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);

    // Configure UART for Bluetooth SPP
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, BT_TX_PIN, BT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
}

void app_main()
{
    // Initialize NVS
    nvs_flash_init();

    // Initialize Bluetooth
    initialize_bluetooth();

    // Initialize UART
    initialize_uart();

    // Create tasks for serial communication
    xTaskCreate(read_serial_task, "read_serial_task", 4096, NULL, 10, NULL);
    xTaskCreate(read_ble_task, "read_ble_task", 4096, NULL, 10, NULL);
}

// Task to read from regular serial and forward to BLE
void read_serial_task(void *pvParameters)
{
    uint8_t data;
    while (true)
    {
        if (uart_read_bytes(UART_NUM_0, &data, 1, portMAX_DELAY) > 0)
        {
            uart_write_bytes(UART_NUM_1, (const char *)&data, 1);
        }
    }
}

// Task to read from BLE and forward to regular serial
void read_ble_task(void *pvParameters)
{
    uint8_t data;
    while (true)
    {
        if (uart_read_bytes(UART_NUM_1, &data, 1, portMAX_DELAY) > 0)
        {
            uart_write_bytes(UART_NUM_0, (const char *)&data, 1);
        }
    }
}
