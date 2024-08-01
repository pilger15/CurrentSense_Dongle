// #TODO: https://github.com/aliengreen/esp32_uart_bridge/blob/main/main/esp32_uart_bridge.c
// #TODO dublebuffer/ringbuffer

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_private/esp_wifi_types_private.h"
#include "esp_private/wifi.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"

// Define the structure for the data to be received over ESP-NOW
#define ADC_BUFFER_LEN (ESP_NOW_MAX_DATA_LEN)
#define BUFFER_SCALE 8
uint16_t buffer_idx = 0;
#define NUM_BUFFER 8
bool buffer_sel = false;
bool buffer_sel_prev = false;
uint8_t buffer_espnow_rx[NUM_BUFFER][ADC_BUFFER_LEN + ESP_NOW_ETH_ALEN];
#define USB_MESSAGE_LEN 16
uint8_t buffer_usb_rx[USB_MESSAGE_LEN];
// a0:76:4e:44:51:38

// static uint8_t sensor_mac2[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x44, 0x51, 0x38};
//  a0:76:4e:41:bd:dc
// static uint8_t sensor_mac1[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x41, 0xBD, 0xDC};
//  a0:76:4e:41:e6:70
// static uint8_t sensor_mac[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x41, 0xE6, 0x70};

#define CONFIG_ESPNOW_CHANNEL 1
uint8_t mac_address[ESP_NOW_ETH_ALEN];
esp_now_peer_info_t *peer_broadcast;
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static const uint8_t espcommand_channel[8] = {0x74, 0x79, 0x73, 0x80, 0xec, 0x3c, 0xce, 0x13};

#define UART_PORT UART_NUM_0
#define UART_TX_GPIO_NUM UART_PIN_NO_CHANGE
#define UART_RX_GPIO_NUM UART_PIN_NO_CHANGE

uint8_t send = 0;
char header[] = {'D', 'A', 'T', 'A'};
void usb_task(void *pvParameters) {
    /* Configure USB-CDC */
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = ADC_BUFFER_LEN + ESP_NOW_ETH_ALEN,
        .rx_buffer_size = ADC_BUFFER_LEN + ESP_NOW_ETH_ALEN};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    /* Configure a bridge buffer for the incoming data */

    memset(buffer_espnow_rx, 0x55, sizeof(buffer_espnow_rx) * sizeof(uint8_t));
    ESP_LOGI("USB", "USB initialised");

    while (true) {
        if (send) {
            send--;

            usb_serial_jtag_write_bytes(header, 4, portMAX_DELAY);
            usb_serial_jtag_write_bytes(&buffer_espnow_rx[send][0], ADC_BUFFER_LEN + ESP_NOW_ETH_ALEN, portMAX_DELAY);
            usb_serial_jtag_ll_txfifo_flush();
        } else if (usb_serial_jtag_read_bytes(buffer_usb_rx, USB_MESSAGE_LEN, pdMS_TO_TICKS(1))) {
            // len = usb_serial_jtag_ll_read_rxfifo(buffer1, ADC_BUFFER_LEN * sizeof(uint8_t));
            ESP_LOGI("ESP-NOW", "Received USB-command. Starting measurement");
            REG_WRITE(GPIO_OUT_W1TS_REG, BIT2);  // LOW
            esp_err_t ret = esp_now_send(peer_broadcast->peer_addr, buffer_usb_rx, 1);
            if (ret != ESP_OK) {
                ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(ret));
            }
            REG_WRITE(GPIO_OUT_W1TC_REG, BIT2);  // LOW
        }
        taskYIELD();
    }
}

// Define the callback function to handle received data
void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    // Cast the received data to the defined structure
    if (data_len <= ESP_NOW_MAX_DATA_LEN) {
        memcpy(&buffer_espnow_rx[send][0], esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
        memcpy(&buffer_espnow_rx[send][ESP_NOW_ETH_ALEN], data, data_len);
        // usb_serial_jtag_write_bytes(&buffer_espnow_rx[send][0], 2, 1 / portTICK_PERIOD_MS);
        // ESP_LOGE("WARNING", "\nDATA %d - %d Buffer", ((uint16_t)data[3] << 8 | data[2]), ((uint16_t)buffer_espnow_rx[send][3] << 8) | buffer_espnow_rx[send][2]);

        send++;
        if (send >= NUM_BUFFER)  // overflow
        {
        }
    } else {
        // Handle the case when the received data size is larger than the buffer size
        // You may choose to truncate the data or handle it in a different way based on your application's requirements.
        // Here, we are printing an error message.
        ESP_LOGE("ESP-NOW", "Received data size is larger than the buffer size");
    }
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void esp_now_channel_broadcast(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (esp_now_is_peer_exist(peer_broadcast->peer_addr)) {  // double-check
            ESP_LOGI("ESP-NOW", "Broadcasting channel for registration\nChannel %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     espcommand_channel[0],
                     espcommand_channel[1],
                     espcommand_channel[2],
                     espcommand_channel[3],
                     espcommand_channel[4],
                     espcommand_channel[5],
                     espcommand_channel[6],
                     espcommand_channel[7]);
            ESP_ERROR_CHECK(esp_now_send(peer_broadcast->peer_addr, espcommand_channel, sizeof(espcommand_channel)));
        }
    }
}

void app_main() {
    esp_err_t ret;
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Initialize NVS
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT2);  // LOW
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("NVS", "NVS Initialised");

    // INIT WIFI
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_2M_S));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Initialize ESPNOW and register sending and receiving callback function.
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_MODE_STA, WIFI_PHY_RATE_MCS7_SGI));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add Dongle information to peer list. */
    peer_broadcast = malloc(sizeof(esp_now_peer_info_t));

    memset(peer_broadcast, 0, sizeof(esp_now_peer_info_t));
    peer_broadcast->channel = CONFIG_ESPNOW_CHANNEL;
    peer_broadcast->ifidx = ESP_IF_WIFI_STA;
    peer_broadcast->encrypt = false;
    memcpy(peer_broadcast->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer_broadcast));

    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_address);
    ESP_LOGI("ESP-NOW", "ESP-Now initialised. MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
             mac_address[0], mac_address[1], mac_address[2],
             mac_address[3], mac_address[4], mac_address[5]);
    ESP_LOGI("USB", "USB-task started");
    xTaskCreate(usb_task, "usb_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY, NULL);

    xTaskCreate(esp_now_channel_broadcast, "esp_now_channel_broadcast", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY, NULL);

    while (0) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI("IDLE", "%02x:%02x:%02x:%02x:%02x:%02x\n",
                 mac_address[0], mac_address[1], mac_address[2],
                 mac_address[3], mac_address[4], mac_address[5]);
    }
}
/*
// a0:76:4e:44:51:38
//a0:76:4e:43:9b:88
a0:76:4e:41:e6:70
    // Cleanup Wi-Fi
    esp_wifi_stop();
    esp_wifi_deinit();
*/