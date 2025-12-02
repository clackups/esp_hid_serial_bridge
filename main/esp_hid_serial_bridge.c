/*
  Code inherited from: https://github.com/asterics/esp32_mouse_keyboard
  by Benjamin Aigner <beni@asterics-foundation.org>,<aignerb@technikum-wien.at>
*/


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

#define HID_TAG "HID"
#define USB_TAG "USB"
#define CTRL_TAG "CTRL"

#define BRIDGE_TASK_PRIO 0

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


/* NVS handle to resolve BT addr to name. In NVS, we store the BT addr as key and the name as value. */
nvs_handle nvs_bt_name_h;

/* Timestamp of last sent HID packet, used for idle sending timer callback */
uint64_t timestampLastSent;

/* Last mouse button state.
   Due to the absolute values for mouse buttons, we need to keep track.
   Because when the mouse button is pressed and hold (without steady movement),
   the idle callback will send an empty report (X/Y/wheel are 0), but
   the mousebuttons must be the same
*/
uint8_t mouseButtons = 0;

/* "Keepalive" rate when in idle (no HID commands) */
#define HID_IDLE_UPDATE_RATE 200000

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define MOUSE_SPEED 30
#define MAX_CMDLEN  100


//a list of active HID connections.
//conn_id array stores the connection ID, if unused it is -1
//active_connections stores the BT mac address
int16_t        active_hid_conn_ids[CONFIG_BT_ACL_CONNECTIONS];
esp_bd_addr_t  active_connections[CONFIG_BT_ACL_CONNECTIONS] = {0};

//only this HID connection should receive the HID data.
int16_t hid_conn_id = -1;


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
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

uint8_t uppercase(uint8_t c)
{
    if ((c>='a') && (c<='z')) return (c-'a'+'A');
    return(c);
}

int get_int(const char * input, int index, int * value)
{
    int sign=1, result=0, valid=0;

    while (input[index]==' ') index++;   // skip leading spaces
    if (input[index]=='-') {
        sign=-1;
        index++;
    }
    while ((input[index]>='0') && (input[index]<='9')) {
        result= result*10+input[index]-'0';
        valid=1;
        index++;
    }
    while (input[index]==' ')
        index++;  // skip trailing spaces
    if (input[index]==',')
        index++;     // or a comma

    if (valid) {
        *value = result*sign;
        return (index);
    }
    return(0);
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



/** Periodic sending of empty HID reports if no updates are sent via API */
static void periodicHIDCallback(void* arg)
{
    if((esp_timer_get_time()-timestampLastSent) > HID_IDLE_UPDATE_RATE)
    {
        //send empty report (but with last known button state)
        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
        {
            if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],mouseButtons,0,0,0);
        }
        //save timestamp for next call
        timestampLastSent = esp_timer_get_time();
    }
}

/**
 * Determine if there is at least one device connected.
 */
bool isConnected()
{
    //determine if there is at least one device connected.
    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
    {
        if(active_hid_conn_ids[i] != -1) return true;
    }
    return false;
}

void printConnectedDevicesTable()
{
    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
    {
        ESP_LOGI(HID_TAG, "%d: HID connection ID: %d",i,active_hid_conn_ids[i]);
        ESP_LOGI(HID_TAG, "%d: remote BD_ADDR: %08x%04x",i,
		 (active_connections[i][0] << 24) + (active_connections[i][1] << 16) +
                 (active_connections[i][2] << 8) + active_connections[i][3],
		 (active_connections[i][4] << 8) + active_connections[i][5]);
    }
}




static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            esp_ble_gap_set_device_name(DEVICE_NAME);
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
        ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");

        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS;i++)
        {
            if(active_hid_conn_ids[i] == -1) //search for the first unused slot
            {
                memcpy(active_connections[i], param->connect.remote_bda, sizeof(esp_bd_addr_t));
                active_hid_conn_ids[i] = param->connect.conn_id;
                ESP_LOGI(HID_TAG, "Added connection: %d @ %d",active_hid_conn_ids[i],i);
                break;
            }
        }

        //because some devices do connect with a quite high connection
        //interval, we might have a congested channel...
        //to overcome this issue, we update the connection parameters here
        //to use a very low connection interval.
        esp_ble_conn_update_params_t new;
        memcpy(new.bda,param->connect.remote_bda,sizeof(esp_bd_addr_t));
        new.min_int = 6;
        new.max_int = 6;
        new.latency = 0;
        new.timeout = 500;
        esp_ble_gap_update_conn_params(&new);

        //to allow more connections, we simply restart the adv process.
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
        {
            //check if this addr is in the array
            if(memcmp(active_connections[i],param->disconnect.remote_bda,sizeof(esp_bd_addr_t)) == 0)
            {
                //clear element
                ESP_LOGI(HID_TAG, "Removed connection: %d @ %d",active_hid_conn_ids[i],i);
                memset(active_connections[i],0,sizeof(esp_bd_addr_t));
                active_hid_conn_ids[i] = -1;
                break;
            }
        }

        ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_CONGEST: {
        if(param->congest.congested)
        {
            ESP_LOGI(HID_TAG, "Congest: %d, conn: %d",param->congest.congested,param->congest.conn_id);
            esp_gap_conn_params_t current;
            esp_ble_get_current_conn_params(active_connections[param->congest.conn_id],&current);
            ESP_LOGI(HID_TAG, "Interval: %d, latency: %d, timeout: %d",current.interval, current.latency, current.timeout);
        }
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
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if(esp_ble_gap_start_scanning(3600) != ESP_OK) ESP_LOGW(HID_TAG,"Cannot start scan");
        else ESP_LOGI(HID_TAG,"Start scan");
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGD(HID_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
    {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_TAG, "remote BD_ADDR: %08x%04x",\
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGW(HID_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        if(esp_ble_gap_update_whitelist(true,bd_addr,BLE_WL_ADDR_TYPE_PUBLIC) != ESP_OK)
        {
            ESP_LOGW(HID_TAG,"cannot add device to whitelist, with public address");
        } else {
            ESP_LOGI(HID_TAG,"added device to whitelist");
        }
        if(esp_ble_gap_update_whitelist(true,bd_addr,BLE_WL_ADDR_TYPE_RANDOM) != ESP_OK)
        {
            ESP_LOGW(HID_TAG,"cannot add device to whitelist, with random address");
        }
    }
    break;
    //handle scan responses here...
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //esp_log_buffer_hex(HID_TAG, scan_result->scan_rst.bda, 6);
            //ESP_LOGI(HID_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if(adv_name_len == 0) {
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
            }
            //ESP_LOGI(HID_TAG, "Searched Device Name Len %d", adv_name_len);
            //esp_log_buffer_char(HID_TAG, adv_name, adv_name_len);
            //ESP_LOGI(HID_TAG, "\n");
            if (adv_name != NULL) {
                //store name to BT addr...
                esp_log_buffer_hex(HID_TAG, scan_result->scan_rst.bda, 6);
                esp_log_buffer_char(HID_TAG, adv_name, adv_name_len);
                adv_name[adv_name_len] = '\0';
                char key[13];
                sprintf(key,"%02X%02X%02X%02X%02X%02X",
                        scan_result->scan_rst.bda[0],scan_result->scan_rst.bda[1],
                        scan_result->scan_rst.bda[2],scan_result->scan_rst.bda[3],
                        scan_result->scan_rst.bda[4],scan_result->scan_rst.bda[5]);
                if(nvs_set_str(nvs_bt_name_h,key,(char*)adv_name) == ESP_OK)
                {
                    ESP_LOGI(HID_TAG,"Saved %s to %s",adv_name, key);
                }
                else {
                    ESP_LOGW(HID_TAG,"Error saving %s for %s",adv_name,key);
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
    }
    break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(HID_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(HID_TAG, "Scan start success");
        break;

    default:
        break;
    }
}




/*
  Commands:
  A     -- start advertising and accepting new connections
  L     -- list active connections
  Cn    -- switch to active connection number n
  Z     -- erase all pairings
  Kxxxx -- send a keyboard report
  Mxxxx -- send a mouse report
  Jxxxx -- send a joystick report
*/

void cdc_bridge_task(void *pvParameters)
{
    ESP_LOGI(CTRL_TAG,"CDC Bridge task started");
    char character;
    while(1)
    {
        // read & process a single byte
        size_t read_len;
        tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, (uint8_t*) &character, 1, &read_len);
        if( read_len > 0 ) {
            ESP_LOGI(HID_TAG, "Got: %c", character);
            // parse the command
        }
        vTaskDelay(1);
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
        ESP_LOGE(HID_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed\n", __func__);
    }

    //open NVS handle for storing BT device names
    ESP_LOGI("MAIN","opening NVS handle for BT names");
    ret = nvs_open("btnames", NVS_READWRITE, &nvs_bt_name_h);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS for bt names");

    ///clear the HID connection IDs&MACs
    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS;i++)
    {
        active_hid_conn_ids[i] = -1;
        memset(active_connections[i],0,sizeof(esp_bd_addr_t));
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback, 1);

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

    //start the bridge job
    xTaskCreate(&cdc_bridge_task, "bridge", 4096, NULL, BRIDGE_TASK_PRIO, NULL);

    //start periodic timer to send HID reports
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodicHIDCallback,
        .name = "HIDidle"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 100000); //call every 100ms
}
