#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "hid_dev.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "sdkconfig.h"

#define DEVICE_NAME "HID Serial Bridge"

#define BLE_TAG "BLE"
#define USB_TAG "USB"
#define CTRL_TAG "CTRL"

#define BRIDGE_TASK_PRIO 1
#define CTRL_COMMAND_TIMEOUT = 500000

#define CDC_FLUSH_TICKS pdMS_TO_TICKS(10)

void usb_device_event_handler(tinyusb_event_t *event, void *arg)
{
    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        // Device has been attached to the USB Host and configured
        break;
    case TINYUSB_EVENT_DETACHED:
        // Device has been detached from the USB Host
        break;
    default:
        break;
    }
}

static void init_usb_cdc(void)
{
    ESP_LOGI(USB_TAG, "USB initialization");

    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(usb_device_event_handler);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_console_init(TINYUSB_CDC_ACM_0));
    ESP_LOGI(USB_TAG, "USB CDC initialized");
}


static void cdc_write_string(const char* data)
{
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t*) data, strlen(data));
}

static void cdc_write_newline()
{
    const char *nl = "\r\n";
    cdc_write_string(nl);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, CDC_FLUSH_TICKS);
}


static char cdc_get_char()
{
    while(1)
    {
        // read & process a single byte
        char c;
        size_t read_len;
        tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, (uint8_t*) &c, 1, &read_len);
        if( read_len > 0 ) {
            return c;
        }
        vTaskDelay(1);
    }
}

static void cdc_read_string(char *buf, size_t expected_chars, uint64_t timeout, size_t *received_chars)
{
    uint64_t end_time = esp_timer_get_time() + timeout;
    size_t received = 0;
    while( received < expected_chars && esp_timer_get_time() < end_time ) {
        char c;
        size_t read_len;
        tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, (uint8_t*) &c, 1, &read_len);
        if( read_len > 0 ) {
            buf[received] = c;
            received++;
        }
        else {
            vTaskDelay(1);
        }
    }
    *received_chars = received;
}


int hex_str_to_bytes(const char *input, size_t input_len,
                     uint8_t *output, size_t *output_len) {
  if (input_len % 2 != 0) {
    return -1;
  }

  size_t out_idx = 0;
  for (size_t in_idx = 0; in_idx < input_len; in_idx++) {
      char c = input[in_idx];
      uint8_t v = 0;
      if (c >= '0' && c <= '9') {
          v = (uint8_t)(c - '0');
      }
      else if (c >= 'A' && c <= 'F') {
          v = (uint8_t)(c - 'A' + 10);
      }
      else if (c >= 'a' && c <= 'f') {
          v = (uint8_t)(c - 'a' + 10);
      }
      else {
          return -1;
      }

      if ( in_idx % 2 == 0 ) {
          output[out_idx] = v * 16;
      }
      else {
          output[out_idx] += v;
          out_idx++;
      }
  }
  *output_len = out_idx;
  return 0;
}


static int cdc_read_hex_bytes(uint8_t *buf, size_t expected_bytes, uint64_t timeout, size_t *received_bytes)
{
    char textbuf[expected_bytes*2];
    size_t received_chars;
    cdc_read_string(textbuf, expected_bytes*2, timeout, &received_chars);
    return hex_str_to_bytes(textbuf, received_chars, buf, received_bytes);
}




static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};


static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x000A, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    /* HID generic appearance does not work on some iOS/Amazon FireTV Sticks, see
       https://github.com/asterics/esp32_mouse_keyboard/issues/52 */
    .appearance = 0x03c1,       //HID keyboard
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};


static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


    


/*
  Commands: (xx is a two-character hex number)
  Pxx   -- start pairing and accepting a new connection for connection slot xx
  L     -- list active connections
  Cxx   -- switch to active connection number xx
  N     -- switch to the next connection slot
  Z     -- erase all pairings
  Kxxxx -- send a keyboard report
  Mxxxx -- send a mouse report
  Jxxxx -- send a joystick report
*/

void cdc_bridge_task(void *pvParameters)
{
    ESP_LOGI(CTRL_TAG,"CDC Bridge task started");
    while(1)
    {
        char c = cdc_get_char();
        switch (c) {
        case 'A':
        {
            int8_t slot_n;
            size_t received_bytes;
            if( cdc_read_hex_bytes(&slot_n, 1, CTRL_COMMAND_TIMEOUT, &received_bytes) == 0 &&
                received_bytes == 1 ) {
                if( slot_n < CONFIG_BT_ACL_CONNECTIONS ) {
                    hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
                    esp_ble_gap_start_advertising(&hidd_adv_params);
                    current_hid_conn_id = slot_n;
                    ESP_LOGI(BLE_TAG, "Started advertising for slot %x", current_hid_conn_id);
                    esp_timer_start_once(adv_timer, BLE_ADV_TIMEOUT);
                }
                else {
                    ESP_LOGE(CTRL_TAG, "Slot number too big, cannot be %d or higher", CONFIG_BT_ACL_CONNECTIONS);
                }
            }
            else {
                ESP_LOGE(CTRL_TAG, "Error in input, expected Axx");
            }
        }
        break;
    }
}





void app_main(void)
{
    esp_err_t ret;

    init_usb_cdc();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
    }


    //start the bridge job
    xTaskCreate(&cdc_bridge_task, "bridge", 4096, NULL, BRIDGE_TASK_PRIO, NULL);
}
