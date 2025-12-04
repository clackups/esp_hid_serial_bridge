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

#define HIDD_DEVICE_NAME "HID Serial Bridge"

#define BLE_TAG "BLE"
#define USB_TAG "USB"
#define CTRL_TAG "CTRL"

#define BRIDGE_TASK_PRIO 1
#define CTRL_COMMAND_TIMEOUT 500000

#define CDC_FLUSH_TICKS pdMS_TO_TICKS(10)

static void start_ble();


void usb_device_event_handler(tinyusb_event_t *event, void *arg)
{
    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        // Device has been attached to the USB Host and configured
        start_ble();
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

static void cdc_write_string_nl(const char* data)
{
    cdc_write_string(data);
    cdc_write_newline();
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




static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static esp_bd_addr_t connected_addr;

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


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG: {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
        ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
        sec_conn = false;
        ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    default:
        break;
    }
    return;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGD(BLE_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "remote BD_ADDR: %08x%04x",\
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(BLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(BLE_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (param->ble_security.auth_cmpl.success) {
            sec_conn = true;
            memcpy(connected_addr, bd_addr, sizeof(esp_bd_addr_t));
            ESP_LOGI(BLE_TAG, "secure connection established.");
        } else {
            ESP_LOGE(BLE_TAG, "pairing failed, reason = 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}




/*
  Commands: (xx is a two-character hex number)
  S     -- retrieve current BLE connection status
  Z     -- erase all pairings
  Kxxxxxx -- send a keyboard report: [modifier][numkeys][key1]...[key6]
  Mxxxxxx -- send a mouse report: [buttons][xmov][ymov]
  Jxxxxxxxxxxxxxxxxxxxxxx -- send a joystick report (11 bytes)
*/

void cdc_bridge_task(void *pvParameters)
{
    ESP_LOGI(CTRL_TAG,"CDC Bridge task started");
    while(1)
    {
        char c = cdc_get_char();
        switch (c) {
        case 'S':
        {
            cdc_write_string("STATUS:");
            if( !sec_conn ) {
                cdc_write_string_nl("NOTCONNECTED");
            }
            else {
                cdc_write_string("CONNECTED:");
                char buf[13];
                sprintf(buf,"%08x%04x", (connected_addr[0] << 24) + (connected_addr[1] << 16) +
                        (connected_addr[2] << 8) + connected_addr[3],
                        (connected_addr[4] << 8) + connected_addr[5]);
                cdc_write_string_nl(buf);
            }
        }
        break;
        case 'Z':
        {
            int dev_num = esp_ble_get_bond_device_num();
            if (dev_num == 0) {
                ESP_LOGI(BLE_TAG, "No bonded devices\n");
            }
            else {
                esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
                if (!dev_list) {
                    ESP_LOGE(BLE_TAG, "malloc failed\n");
                }
                else {
                    esp_ble_get_bond_device_list(&dev_num, dev_list);
                    for (int i = 0; i < dev_num; i++) {
                        esp_ble_remove_bond_device(dev_list[i].bd_addr);
                    }
                    free(dev_list);
                }
            }
        }
        break;
        case 'K':
        {
            uint8_t special_key_mask;
            uint8_t num_keys;
            uint8_t keys[6] = {0};
            size_t received_bytes;
            uint8_t ok = 1;
            if( cdc_read_hex_bytes(&special_key_mask, 1, CTRL_COMMAND_TIMEOUT, &received_bytes) != 0 || received_bytes != 1 ) {
                ok = 0;
            }
            if( !ok || cdc_read_hex_bytes(&num_keys, 1, CTRL_COMMAND_TIMEOUT, &received_bytes) != 0 || received_bytes != 1 ) {
                ok = 0;
            }
            if( !ok || cdc_read_hex_bytes(keys, num_keys, CTRL_COMMAND_TIMEOUT, &received_bytes) != 0 ||
                received_bytes != num_keys ) {
                ok = 0;
            }
            if( ok ) {
                if( sec_conn ) {
                    esp_hidd_send_keyboard_value(hid_conn_id, special_key_mask, keys, num_keys);
                }
                else {
                    cdc_write_string_nl("ERR:NOTCONNECTED");
                }
            }
            else {
                cdc_write_string_nl("ERR:SYNTAX");
            }
        }
        break;
        case 'M':
        {
            uint8_t data[5];
            size_t received_bytes;
            uint8_t ok = 1;
            if( cdc_read_hex_bytes(data, 5, CTRL_COMMAND_TIMEOUT, &received_bytes) != 0 || received_bytes != 5 ) {
                ok = 0;
            }
            if( ok ) {
                if( sec_conn ) {
                    esp_hidd_send_mouse_value(hid_conn_id, data[0], data[1], data[2], data[3], data[4]);
                }
                else {
                    cdc_write_string_nl("ERR:NOTCONNECTED");
                }
            }
            else {
                cdc_write_string_nl("ERR:SYNTAX");
            }
        }
        case 'J':
        {
            uint8_t data[11];
            size_t received_bytes;
            uint8_t ok = 1;
            if( cdc_read_hex_bytes(data, 1, CTRL_COMMAND_TIMEOUT, &received_bytes) != 0 || received_bytes != 11 ) {
                ok = 0;
            }
            if( ok ) {
                if( sec_conn ) {
                    uint32_t buttons = (uint32_t)data[7] + ((uint32_t)data[8] << 8) +
                        ((uint32_t)data[9] << 16) + ((uint32_t)data[10] << 24);
                    esp_hidd_send_joy_value(hid_conn_id, data[0], data[1], data[2],
                                            data[3], data[4], data[5], data[6], buttons);
                }
                else {
                    cdc_write_string_nl("ERR:NOTCONNECTED");
                }
            }
            else {
                cdc_write_string_nl("ERR:SYNTAX");
            }
        }
        break;
        }
    }
}



static void start_ble()
{
    esp_err_t ret;
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

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
       and the response key means which key you can distribute to the Master;
       If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
       and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
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

    //start the bridge job
    xTaskCreate(&cdc_bridge_task, "bridge", 4096, NULL, BRIDGE_TASK_PRIO, NULL);
}
