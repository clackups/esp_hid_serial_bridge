/*
  Code inherited from: https://github.com/asterics/esp32_mouse_keyboard
  by Benjamin Aigner <beni@asterics-foundation.org>,<aignerb@technikum-wien.at>
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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
#include "config.h"
#include "esp_ota_ops.h"

#include "tusb.h"
#include "usb_defs.h"
#include "hal/usb_phy_types.h"
#include "esp_mac.h"
#include "esp_private/usb_phy.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"


#define HID_TAG "HID"
#define USB_TAG "USB"

#define SERIAL_HANDLER_TASK_PRI 5

#define MS_OS_20_DESC_LEN  0xB2
#define VENDOR_REQUEST_MICROSOFT 0x20  // Can be any value between 0x20 and 0xFF

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_MSC_DESC_LEN)

static const tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210, // at least 2.1 or 3.x for BOS
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#ifdef CFG_TUD_ENDPOINT0_SIZE
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
#else  // earlier versions have a typo in the name
    .bMaxPacketSize0 = CFG_TUD_ENDOINT0_SIZE,
#endif
    .idVendor = CONFIG_BRIDGE_USB_VID,
    .idProduct = CONFIG_BRIDGE_USB_PID,
    .bcdDevice = BCDDEVICE,     // defined in CMakeLists.txt
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static uint8_t const desc_configuration[] = {
    // config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, EPNUM_CDC, 0x80 | EPNUM_CDC, CFG_TUD_CDC_EP_BUFSIZE),
};

#define MAC_BYTES       6

static char serial_descriptor[MAC_BYTES * 2 + 1] = {'\0'}; // 2 chars per hexnumber + '\0'

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
    CONFIG_BRIDGE_MANUFACTURER,    // 1: Manufacturer
    CONFIG_BRIDGE_PRODUCT_NAME,    // 2: Product
    serial_descriptor,             // 3: Serials
    "CDC",
};

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

// BOS Descriptor with Microsoft OS 2.0 support
static uint8_t const desc_bos[] = {
    // total length, number of device caps
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

    // Microsoft OS 2.0 descriptor
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    return desc_configuration;
}

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &descriptor_config;
}


void tud_mount_cb(void)
{
    ESP_LOGI(USB_TAG, "Mounted");
}

static void init_serial_no(void)
{
    uint8_t m[MAC_BYTES] = {0};
    esp_err_t ret = esp_efuse_mac_get_default(m);

    if (ret != ESP_OK) {
        ESP_LOGD(USB_TAG, "Cannot read MAC address and set the device serial number");
    }

    snprintf(serial_descriptor, sizeof(serial_descriptor),
             "%02X%02X%02X%02X%02X%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void init_usb_phy(void)
{
    usb_phy_config_t phy_config = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_FULL,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };
    usb_phy_handle_t phy_handle;
    usb_new_phy(&phy_config, &phy_handle);
}


uint16_t const *tud_descriptor_string_cb(const uint8_t index, const uint16_t langid)
{
    static uint16_t _desc_str[32];  // Static, because it must exist long enough for transfer to complete
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}


static void tusb_device_task(void *pvParameters)
{
    while (1) {
        tud_task();
    }
    vTaskDelete(NULL);
}


#define USB_SEND_RINGBUFFER_SIZE (2 * 1024)
#define USB_RECV_RINGBUFFER_SIZE (2 * 1024)

static RingbufHandle_t usb_sendbuf;
static RingbufHandle_t usb_recvbuf;
static SemaphoreHandle_t usb_tx_requested = NULL;
static SemaphoreHandle_t usb_tx_done = NULL;


esp_err_t cdc_write(const char *data, size_t len)
{
    ESP_LOGD(USB_TAG, "To USB ringbuffer (%zu bytes)", len);
    ESP_LOG_BUFFER_HEXDUMP("Data: ", data, len, ESP_LOG_DEBUG);

    // Send the data to USB CDC
    if (xRingbufferSend(usb_sendbuf, data, len, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGV(USB_TAG, "Cannot write to ringbuffer (free %zu of %zu)!",
                 xRingbufferGetCurFreeSize(usb_sendbuf),
                 (size_t)USB_SEND_RINGBUFFER_SIZE);
        vTaskDelay(pdMS_TO_TICKS(10));
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

int cdc_read(void *databuf, uint32_t length, TickType_t ticks_to_wait)
{
    size_t ringbuf_received = 0;
    uint8_t *buf = xRingbufferReceiveUpTo(usb_recvbuf, &ringbuf_received, ticks_to_wait, length);
    if (buf) {
        memcpy(databuf, buf, ringbuf_received);
        vRingbufferReturnItem(usb_recvbuf, (void *) buf);
    }
    return ringbuf_received;
}



static esp_err_t usb_wait_for_tx(const uint32_t block_time_ms)
{
    if (xSemaphoreTake(usb_tx_done, pdMS_TO_TICKS(block_time_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}


static void usb_sender_task(void *pvParameters)
{
    while (1) {
        size_t ringbuf_received;
        uint8_t *buf = xRingbufferReceiveUpTo(usb_sendbuf, &ringbuf_received, pdMS_TO_TICKS(100),
                                              CFG_TUD_CDC_TX_BUFSIZE);

        if (buf) {
            uint8_t int_buf[CFG_TUD_CDC_TX_BUFSIZE];
            memcpy(int_buf, buf, ringbuf_received);
            vRingbufferReturnItem(usb_sendbuf, (void *) buf);

            for (int transferred = 0, to_send = ringbuf_received; transferred < ringbuf_received;) {
                xSemaphoreGive(usb_tx_requested);
                const int wr_len = tud_cdc_write(int_buf + transferred, to_send);
                /* tinyusb might have been flushed the data. In case not flushed, we are flushing here.
                    2nd attempt might return zero, meaning there is no data to transfer. So it is safe to call it again.
                */
                tud_cdc_write_flush();
                if (usb_wait_for_tx(50) != ESP_OK) {
                    xSemaphoreTake(usb_tx_requested, 0);
                    tud_cdc_write_clear(); /* host might be disconnected. drop the buffer */
                    ESP_LOGV(USB_TAG, "usb tx timeout");
                    break;
                }
                ESP_LOGD(USB_TAG, "USB ringbuffer -> USB CDC (%d bytes)", wr_len);
                transferred += wr_len;
                to_send -= wr_len;
            }
        } else {
            ESP_LOGD(USB_TAG, "usb_sender_task: nothing to send");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
    }
    vTaskDelete(NULL);
}


void tud_cdc_tx_complete_cb(const uint8_t itf)
{
    if (xSemaphoreTake(usb_tx_requested, 0) != pdTRUE) {
        /* Semaphore should have been given before write attempt.
            Sometimes tinyusb can send one more cb even xfer_complete len is zero
        */
        return;
    }

    xSemaphoreGive(usb_tx_done);
}


void tud_cdc_rx_cb(const uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    const uint32_t rx_size = tud_cdc_n_read(itf, buf, CFG_TUD_CDC_RX_BUFSIZE);
    if (rx_size > 0) {
        ESP_LOGD(USB_TAG, "USB CDC -> (%" PRIu32 " bytes)", rx_size);
        ESP_LOG_BUFFER_HEXDUMP("Data: ", buf, rx_size, ESP_LOG_DEBUG);

        // save to usb_recvbuf
        if (xRingbufferSend(usb_recvbuf, buf, rx_size, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGV(USB_TAG, "Cannot write to ringbuffer (free %zu of %zu)!",
                     xRingbufferGetCurFreeSize(usb_recvbuf),
                     (size_t)USB_SEND_RINGBUFFER_SIZE);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        ESP_LOGW(USB_TAG, "tud_cdc_rx_cb receive error");
    }
}


static esp_err_t init_usb_rxtx(void)
{
    // Create ring buffer for USB sending
    usb_sendbuf = xRingbufferCreate(USB_SEND_RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sendbuf) {
        ESP_LOGE(USB_TAG, "Cannot create ringbuffer for USB sender");
        return ESP_ERR_NO_MEM;
    }

    // Create ring buffer for USB receiving
    usb_recvbuf = xRingbufferCreate(USB_RECV_RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_recvbuf) {
        ESP_LOGE(USB_TAG, "Cannot create ringbuffer for USB receiver");
        return ESP_ERR_NO_MEM;
    }

    // Create semaphores for USB TX synchronization
    usb_tx_done = xSemaphoreCreateBinary();
    usb_tx_requested = xSemaphoreCreateBinary();
    if (!usb_tx_done || !usb_tx_requested) {
        ESP_LOGE(USB_TAG, "Cannot create USB TX semaphores");
        return ESP_ERR_NO_MEM;
    }

    // Start USB sender task
    xTaskCreate(usb_sender_task, "usb_sender_task", 4 * 1024, NULL, SERIAL_HANDLER_TASK_PRI, NULL);

    ESP_LOGI(USB_TAG, "USB rx/tx initialized");
    return ESP_OK;
}



/** @brief NVS handle to resolve BT addr to name
 * 
 * In NVS, we store the BT addr as key and the name as value. */
nvs_handle nvs_bt_name_h;

/** @brief NVS handle to store key/value pairs via UART
 * 
 * In NVS, we store arbitrary values, which are sent via UART.
 * This can(will) be used for storing different values from the
 * GUI on the ESP32. */
nvs_handle nvs_storage_h;

/** Timestamp of last sent HID packet, used for idle sending timer callback 
 * @see periodicHIDCallback */
uint64_t timestampLastSent;

/** Last mouse button state.
 * Due to the absolute values for mouse buttons, we need to keep track.
 * Because when the mouse button is pressed and hold (without steady movement),
 * the idle callback will send an empty report (X/Y/wheel are 0), but
 * the mousebuttons must be the same
 * @see periodicHIDCallback */
uint8_t mouseButtons = 0;


/** "Keepalive" rate when in idle (no HID commands)
 * @note Microseconds!
 * @see timestampLastSent
 * @see periodicHIDCallback */
#define HID_IDLE_UPDATE_RATE 200000



static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define MOUSE_SPEED 30
#define MAX_CMDLEN  100

#define EXT_UART_TAG "EXT_UART"
#define CONSOLE_UART_TAG "CONSOLE_UART"

static config_data_t config;

#define CMDSTATE_IDLE 0
#define CMDSTATE_GET_RAW 1
#define CMDSTATE_GET_ASCII 2

//a list of active HID connections.
//conn_id array stores the connection ID, if unused it is -1
//active_connections stores the BT mac address
int16_t active_hid_conn_ids[CONFIG_BT_ACL_CONNECTIONS];
esp_bd_addr_t active_connections[CONFIG_BT_ACL_CONNECTIONS] = {0};
//this HID connection is used, if only ONE device should receive the HID data.
//currently, the $SW command to select the HID device to be controlled it is
//set to -1 (send to all devices). If it is != -1, we will send it to only this device.
///@note This works only for the UART interface (uart_parse_command).
int16_t hid_conn_id = -1;

struct cmdBuf {
    //current state of the parser, CMD_STATE*
    int state;
    //if a fixed length command is issued, we store expected length here
    int expectedBytes;
    int bufferLength;
    uint8_t buf[MAX_CMDLEN];
};

static uint8_t manufacturer[19]= {'c', 'l', 'a', 'c', 'k', 'u', 'p', 's'};


static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

/** @brief Event bit, set if pairing is enabled
 * @note If MODULE_BT_PAIRING ist set in menuconfig, this bit is disable by default
 * and can be enabled via $PM1 , disabled via $PM0.
 * If MODULE_BT_PAIRING is not set, this bit will be set on boot.*/
#define SYSTEM_PAIRING_ENABLED (1<<0)

/** @brief Event bit, set if the ESP32 is currently advertising.
 *
 * Used for determining if we need to set advertising params again,
 * when the pairing mode is changed. */
#define SYSTEM_CURRENTLY_ADVERTISING (1<<1)

/** @brief Event group for system status */
EventGroupHandle_t eventgroup_system;

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x000A, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    /** @note: HID generic appearance does not work on some iOS/Amazon FireTV Sticks, see https://github.com/asterics/esp32_mouse_keyboard/issues/52 */
    .appearance = 0x03c2,       //HID Mouse (keyboard: 0x03c1)
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_scan_params_t scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x00A0, /* 100ms (n*0.625ms)*/
    .scan_window = 0x0090, /* 90ms */
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

// config scan response data
///@todo Scan response is currently not used. If used, add state handling (adv start) according to ble/gatt_security_server example of Espressif
static esp_ble_adv_data_t hidd_adv_resp = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(manufacturer),
    .p_manufacturer_data = manufacturer,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
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
    while ((input[index]>='0') && (input[index]<='9'))
    {
        result= result*10+input[index]-'0';
        valid=1;
        index++;
    }
    while (input[index]==' ') index++;  // skip trailing spaces
    if (input[index]==',') index++;     // or a comma

    if (valid) {
        *value = result*sign;
        return (index);
    }
    return(0);
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
        ESP_LOGD(HID_TAG,"Idle...");
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
		 (active_connections[i][0] << 24) + (active_connections[i][1] << 16) + (active_connections[i][2] << 8) + active_connections[i][3],
		 (active_connections[i][4] << 8) + active_connections[i][5]);
    }
}


void update_config()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(err != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    err = nvs_set_str(my_handle, "btname", config.bt_device_name);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - bt name");
    err = nvs_set_u8(my_handle, "locale", config.locale);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - locale");
    err = nvs_set_u8(my_handle, "joyactive", config.joystick_active);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - joystick state");
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    nvs_close(my_handle);
}


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(config.bt_device_name);
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
        //xEventGroupClearBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
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
        xEventGroupSetBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
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
        xEventGroupSetBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
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
        } else {
            xEventGroupClearBits(eventgroup_system,SYSTEM_CURRENTLY_ADVERTISING);
        }
#if CONFIG_MODULE_BT_PAIRING
        //add connected device to whitelist (necessary if whitelist connections only).
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
#endif
    }
    break;
    //handle scan responses here...    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //esp_log_buffer_hex(HID_TAG, scan_result->scan_rst.bda, 6);
            //ESP_LOGI(HID_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if(adv_name_len == 0) adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
            //ESP_LOGI(HID_TAG, "Searched Device Name Len %d", adv_name_len);
            //esp_log_buffer_char(HID_TAG, adv_name, adv_name_len);
            //ESP_LOGI(HID_TAG, "\n");
            if (adv_name != NULL) {
                //store name to BT addr...
                esp_log_buffer_hex(HID_TAG, scan_result->scan_rst.bda, 6);
                esp_log_buffer_char(HID_TAG, adv_name, adv_name_len);
                adv_name[adv_name_len] = '\0';
                char key[13];
                sprintf(key,"%02X%02X%02X%02X%02X%02X",scan_result->scan_rst.bda[0],scan_result->scan_rst.bda[1], \
                        scan_result->scan_rst.bda[2],scan_result->scan_rst.bda[3],scan_result->scan_rst.bda[4],scan_result->scan_rst.bda[5]);
                if(nvs_set_str(nvs_bt_name_h,key,(char*)adv_name) == ESP_OK)
                {
                    ESP_LOGI(HID_TAG,"Saved %s to %s",adv_name, key);
                } else ESP_LOGW(HID_TAG,"Error saving %s for %s",adv_name,key);
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


void processCommand(struct cmdBuf *cmdBuffer)
{
    //commands:
    // $ID
    // $PMx (0 or 1)
    // $GP
    // $DPx (number of paired device, starting with 0)
    // $SW aabbccddeeff (select a BT addr to send the HID commands to)
    // $GC get connected devices
    // $NAME set name of bluetooth device
    // $JPx (0,1) en- / disable the joystick interface (for iOS compatibility) [available if compiled with Joystick support]
    // $APx (0-4) Set the appearance value for advertising (0x03C0 - 0x03C4; default is mouse). See https://specificationrefs.bluetooth.com/assigned-values/Appearance%20Values.pdf page 8
    // $GV <key>  get the value of the given key from NVS. Note: no spaces in <key>! max. key length: 15
    // $SV <key> <value> set the value of the given key & store to NVS. Note: no spaces in <key>!
    // $CV clear all key/value pairs set with $SV
    // $UG start flash update by searching for factory partition and rebooting there. Warning: not possible to boot back without flashing!
    // $LGx (0,1,2): enable / disable logging system of ESP32.0 is level error, 1 is level info, 2 is level debug

    if(cmdBuffer->bufferLength < 2) return;
    //easier this way than typecast in each str* function
    const char *input = (const char *) cmdBuffer->buf;
    int len = cmdBuffer->bufferLength;
    const char *nl = "\r\n";
    esp_ble_bond_dev_t * btdevlist;
    int counter;
    esp_err_t ret;
  
#if CONFIG_MODULE_USEJOYSTICK
    /**++++ (de-)activate joystick ++++*/
    if(strncmp(input,"JP",2) == 0)
    {
        uint8_t joystate = input[2] - '0';
        if(joystate) config.joystick_active = 1;
        else config.joystick_active = 0;
        update_config();
        ESP_LOGI(EXT_UART_TAG,"new joystick state: %d, will show on next reboot", config.joystick_active);
        cdc_write("JS:",strlen("JS:"));
        if(joystate) cdc_write("1",1);
        else cdc_write("0",1);
        cdc_write(nl,sizeof(nl)); //newline
        return;
    }
#endif
    
    /**++++ set BLE appearance ++++*/
    if(strncmp(input,"AP", 2) == 0)
    {
        uint8_t appv = input[2] - '0';
        if(appv <= 4)
        {
            ESP_LOGI(EXT_UART_TAG,"setting appearance to NVS, will show on next reboot");
            nvs_set_u8(nvs_storage_h,"BLEAPPEAR",appv);
            nvs_commit(nvs_storage_h);
            cdc_write("AP:",strlen("AP:"));
            cdc_write(&input[2],1);
            cdc_write(nl,sizeof(nl)); //newline
        } else {
            ESP_LOGE(EXT_UART_TAG,"Cannot set appearance, value not correct. Use AP0 - AP4");
            cdc_write("AP:invalid number, AP0-AP4",strlen("AP:invalid number, AP0-AP4"));
            cdc_write(nl,sizeof(nl)); //newline
        }
        return;
    }
	
    /**++++en-/disable logging++++*/
    if(strcmp(input,"LG0") == 0)
    {
        esp_log_level_set("*",ESP_LOG_ERROR);
        cdc_write("LOG:0",strlen("LOG:0"));
        cdc_write(nl,sizeof(nl)); //newline
        return;
    }
    if(strcmp(input,"LG1") == 0)
    {
        esp_log_level_set("*",ESP_LOG_INFO);
        cdc_write("LOG:1",strlen("LOG:1"));
        cdc_write(nl,sizeof(nl)); //newline
        return;
    }
    if(strcmp(input,"LG2") == 0)
    {
        esp_log_level_set("*",ESP_LOG_DEBUG);
        cdc_write("LOG:2",strlen("LOG:2"));
        cdc_write(nl,sizeof(nl)); //newline
        return;
    }
	
    /**++++ key/value storing ++++*/
    if(strncmp(input,"CV ", 2) == 0)
    {
        //no error checks here, because all errors
        //are related to the NVS part, which cannot be fixed via
        //the UART console
        nvs_erase_all(nvs_storage_h);
        //commit NVS storage
        ret = nvs_commit(nvs_storage_h);
        ESP_LOGI(EXT_UART_TAG,"cleared all NVS key/value pairs");
        cdc_write("NVS:OK",strlen("NVS:OK"));
        cdc_write(nl,sizeof(nl)); //newline
        return;
    }
    if(strncmp(input,"GV ", 3) == 0)
    {
        char* work = (char*)cmdBuffer->buf;
        //remove GV command name
        strsep(&work, " ");
        //get key
        char *key = strsep(&work, " ");
		
        //get data size from NVS, check if key is set
        size_t sizeData;
        char* nvspayload = NULL;
        ret = nvs_get_str(nvs_storage_h, key,NULL,&sizeData);
		
        //if we have a data length, load string
        if(ret == ESP_OK)
        {
            //load str data
            nvspayload = malloc(sizeData);
            ret = nvs_get_str(nvs_storage_h, key, nvspayload, &sizeData);
        }
		
        //OK or error?
        if(ret != ESP_OK)
        {
            //send back error message
            ESP_LOGI(EXT_UART_TAG,"error reading value: %s",esp_err_to_name(ret));
            cdc_write("NVS:",strlen("NVS:"));
            cdc_write(esp_err_to_name(ret), strlen(esp_err_to_name(ret)));
            cdc_write(nl,sizeof(nl)); //newline
        } else {
            ESP_LOGI(EXT_UART_TAG,"loaded - %s:%s",key,nvspayload);
            cdc_write("NVS:",strlen("NVS:"));
            cdc_write(nvspayload, strlen(nvspayload));
            cdc_write(nl,sizeof(nl)); //newline
        }
		
        //done with the payload
        if(nvspayload) free(nvspayload);
        return;
    }
	
    if(strncmp(input,"SV ", 3) == 0)
    {
        char* work = (char*)cmdBuffer->buf;
        //remove SV command name
        strsep(&work, " ");
        //get key
        char* key = strsep(&work, " ");
        //get payload
        char* nvspayload = work;
		
        if(work == NULL)
        {
            ESP_LOGI(EXT_UART_TAG,"error setting string: no value provided");
            cdc_write("NVS:ESP_ERR_NVS_NO_VALUE",strlen("NVS:ESP_ERR_NVS_NO_VALUE"));
            cdc_write(nl,sizeof(nl)); //newline
            return;
        }
		
        //try to set string data to nvs
        ret = nvs_set_str(nvs_storage_h, key,nvspayload);
		
        if(ret == ESP_OK)
        {
            //commit NVS storage
            ret = nvs_commit(nvs_storage_h);
        }
		
        if(ret != ESP_OK)
        {
            //send back error message
            ESP_LOGI(EXT_UART_TAG,"error setting string: %s",esp_err_to_name(ret));
            cdc_write("NVS:",strlen("NVS:"));
            cdc_write(esp_err_to_name(ret), strlen(esp_err_to_name(ret)));
            cdc_write(nl,sizeof(nl)); //newline
        } else {
            //send back OK & used/free entries
            nvs_stats_t nvs_stats;
            nvs_get_stats(NULL, &nvs_stats);
            ESP_LOGI(EXT_UART_TAG,"set - %s:%s - used:%d,free:%d",key,nvspayload,nvs_stats.used_entries, nvs_stats.free_entries);
            cdc_write("NVS:OK ", strlen("NVS:OK "));
            char stats[64];
            sprintf(stats,"%d/%d - used/free",nvs_stats.used_entries, nvs_stats.free_entries);
            cdc_write(stats,strnlen(stats,64));
            cdc_write(nl,sizeof(nl)); //newline
        }
        return;
    }

    /**++++ commands without parameters ++++*/
    //get connected devices
    if(strcmp(input,"GC") == 0)
    {
        char hexnum[5];
        ESP_LOGI(EXT_UART_TAG,"connected devices (starting with index 0):");
        ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
        {
            esp_bd_addr_t empty = {0,0,0,0,0,0};
			
            //only select active connections
            if(memcmp(active_connections[i],empty,sizeof(esp_bd_addr_t)) != 0)
            {
                //print on monitor & external uart
                cdc_write("CONNECTED:",strlen("CONNECTED:"));
                esp_log_buffer_hex(EXT_UART_TAG, active_connections[i], sizeof(esp_bd_addr_t));
                for (int t=0; t<sizeof(esp_bd_addr_t); t++) {
                    sprintf(hexnum,"%02X ",active_connections[i][t]);
                    cdc_write(hexnum, 3);
                }
                cdc_write(nl,sizeof(nl)); //newline
            }
        }
        ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
        return;
    }
    //switch between BT devices which are connected... 
    if(input[0] == 'S' && input[1] == 'W')
    {
        if(len >= 15)
        {
            esp_bd_addr_t newaddr;
            for(uint8_t i = 0; i<6; i++) {
                if(input[i*2+3] >= '0' && input[i*2+3] <= '9') newaddr[i] = (input[i*2+3] - '0')<<4;
                if(input[i*2+3] >= 'a' && input[i*2+3] <= 'f') newaddr[i] = (input[i*2+3] + 10 - 'a')<<4;
                if(input[i*2+3] >= 'A' && input[i*2+3] <= 'F') newaddr[i] = (input[i*2+3] + 10 - 'A')<<4;
				
                if(input[i*2+1+3] >= '0' && input[i*2+1+3] <= '9') newaddr[i] |= input[i*2+1+3] - '0';
                if(input[i*2+1+3] >= 'a' && input[i*2+1+3] <= 'f') newaddr[i] |= input[i*2+1+3] + 10 - 'a';
                if(input[i*2+1+3] >= 'A' && input[i*2+1+3] <= 'F') newaddr[i] |= input[i*2+1+3] + 10 - 'A';
            }
            esp_log_buffer_hex(HID_TAG, newaddr, 6);
			
            for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
            {
                //check if this addr is in the array
                if(memcmp(active_connections[i],newaddr,sizeof(esp_bd_addr_t)) == 0)
                {
                    ESP_LOGI(EXT_UART_TAG, "New hid_conn_id: %d",i);
                    hid_conn_id = i;
                    return;
                }
            }
            ESP_LOGW(EXT_UART_TAG,"Cannot find BT MAC in connections");
        } else {
            ESP_LOGW(EXT_UART_TAG,"Command to short (need full BT MAC addr): %d",len);
        }
        return;
    }
    
    
    //get module ID
    if(strcmp(input,"ID") == 0)
    {
        cdc_write(MODULE_ID, sizeof(MODULE_ID));
        cdc_write(nl, sizeof(nl));
        ESP_LOGI(EXT_UART_TAG,"ID: %s",MODULE_ID);
        return;
    }
    //disable pairing
    if(strcmp(input,"PM0") == 0)
    {
#if CONFIG_MODULE_BT_PAIRING
        ESP_LOGI(EXT_UART_TAG,"$PM0 - disabling pairing");
        xEventGroupClearBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
        if(hidd_adv_params.adv_filter_policy == ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY)
        {
            //restart advertising with whitelisted connection only (no connection is possible if not
            // on whitelist)
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
            esp_ble_gap_start_advertising(&hidd_adv_params);
        }
#else
        ESP_LOGW(EXT_UART_TAG,"Not available, cannot change pairing state (bug in esp-idf)");
#endif
        return;
    }
    //enable pairing
    if(strcmp(input,"PM1") == 0)
    {
#if CONFIG_MODULE_BT_PAIRING
        ESP_LOGI(EXT_UART_TAG,"$PM1 - enabling pairing");
        xEventGroupSetBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
        if(hidd_adv_params.adv_filter_policy == ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST)
        {
            //restart advertising with open connection
            hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
            esp_ble_gap_start_advertising(&hidd_adv_params);
        }
#else
        ESP_LOGW(EXT_UART_TAG,"Not available, cannot change pairing state (bug in esp-idf)");
#endif
        return;
    }

    //get all BT pairings
    if(strcmp(input,"GP") == 0)
    {
        counter = esp_ble_get_bond_device_num();
        char hexnum[5];

        if(counter > 0)
        {
            btdevlist = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t)*counter);
            if(btdevlist != NULL)
            {
                if(esp_ble_get_bond_device_list(&counter,btdevlist) == ESP_OK)
                {
                    ESP_LOGI(EXT_UART_TAG,"bonded devices (starting with index 0):");
                    ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
                    for(uint8_t i = 0; i<counter; i++)
                    {
                        //print on monitor & external uart
                        cdc_write("PAIRING:",strlen("PAIRING:"));
                        esp_log_buffer_hex(EXT_UART_TAG, btdevlist[i].bd_addr, sizeof(esp_bd_addr_t));
                        for (int t=0; t<sizeof(esp_bd_addr_t); t++) {
                            sprintf(hexnum,"%02X ",btdevlist[i].bd_addr[t]);
                            cdc_write(hexnum, 3);
                        }
                        //print out name
                        char btname[64];
                        size_t name_len = 0;
                        char key[13];
                        sprintf(key,"%02X%02X%02X%02X%02X%02X",btdevlist[i].bd_addr[0],btdevlist[i].bd_addr[1], \
                                btdevlist[i].bd_addr[2],btdevlist[i].bd_addr[3],btdevlist[i].bd_addr[4],btdevlist[i].bd_addr[5]);
                        
                        if(nvs_get_str(nvs_bt_name_h,key,btname,&name_len) == ESP_OK)
                        {
                            sprintf(hexnum," - ");
                            cdc_write(hexnum, 3);
                            cdc_write(btname, name_len);
                            ESP_LOGI(EXT_UART_TAG,"%s",btname);
                        }
                        else ESP_LOGW(EXT_UART_TAG,"cannot find name for addr.");
                        cdc_write(nl,sizeof(nl)); //newline
                    }
                    ESP_LOGI(EXT_UART_TAG,"---------------------------------------");
                } else ESP_LOGW(EXT_UART_TAG,"error getting device list");
            } else ESP_LOGE(EXT_UART_TAG,"error allocating memory for device list");
        } else {
            ESP_LOGI(EXT_UART_TAG,"error getting bonded devices count or no devices bonded");
            cdc_write("END\r\n", 5);
        }
        return;
    }


    //DP: delete one pairing
    if(input[0] == 'D' && input[1] == 'P')
    {
        int index_to_remove;
        if (!(get_int(input,2,&index_to_remove))) {
            ESP_LOGI(EXT_UART_TAG,"DP: no integer, deleting all bonded devices");
            index_to_remove = -1;
        }

        counter = esp_ble_get_bond_device_num();
        if(counter == 0)
        {
            ESP_LOGI(EXT_UART_TAG,"error deleting device, no paired devices");
            return;
        }

        if(index_to_remove >= counter)
        {
            ESP_LOGW(EXT_UART_TAG,"error deleting device, number out of range");
            return;
        }
        if(counter >= 0)
        {
            btdevlist = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t)*counter);
            if(btdevlist != NULL)
            {
                if(esp_ble_get_bond_device_list(&counter,btdevlist) == ESP_OK)
                {
                    //deleting only one pairing (-> -1 == delete all)
                    if(index_to_remove >= 0)
                    {
                        esp_ble_remove_bond_device(btdevlist[index_to_remove].bd_addr);
                        esp_ble_gap_update_whitelist(false,btdevlist[index_to_remove].bd_addr,BLE_WL_ADDR_TYPE_PUBLIC);
                        esp_ble_gap_update_whitelist(false,btdevlist[index_to_remove].bd_addr,BLE_WL_ADDR_TYPE_RANDOM);
                    } else {
                        for(int i = 0; i<counter; i++)
                        {
                            esp_ble_remove_bond_device(btdevlist[i].bd_addr);
                            esp_ble_gap_update_whitelist(false,btdevlist[i].bd_addr,BLE_WL_ADDR_TYPE_PUBLIC);
                            esp_ble_gap_update_whitelist(false,btdevlist[i].bd_addr,BLE_WL_ADDR_TYPE_RANDOM); 
                        }
                    }
                } else ESP_LOGI(EXT_UART_TAG,"error getting device list");
                free (btdevlist);
                //wait 20 ticks for everything to settle (write commits to NVS)
                vTaskDelay(20);
                //then restart to avoid re-bonding of the device(s).
                esp_restart();
            } else ESP_LOGW(EXT_UART_TAG,"error allocating memory for device list");
        } else ESP_LOGW(EXT_UART_TAG,"error getting bonded devices count");
        return;
    }

    //set BT GATT advertising name
    if(strncmp(input,"NAME ", 5) == 0)
    {
        if ((strlen(input)>6) && (strlen(input)-4<MAX_BT_DEVICENAME_LENGTH))
        {
            strcpy (config.bt_device_name, input+5);
            update_config();
            ESP_LOGI(EXT_UART_TAG,"NAME: new bt device name was stored");
        }
        else ESP_LOGI(EXT_UART_TAG,"NAME: given bt name is too long or too short");
        return;
    }

    //UG: triggering update mode of ESP by restarting into "factory partition"
    if (strcmp(input, "UG") == 0)
    {
        esp_partition_iterator_t pi;

        pi = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

        if (pi != NULL) {
            const esp_partition_t* factory = esp_partition_get(pi);
            esp_partition_iterator_release(pi);
            if (esp_ota_set_boot_partition(factory) == ESP_OK) {
                cdc_write("OTA:start", strlen("OTA:start"));
                cdc_write(nl, sizeof(nl));
                ESP_LOGI(EXT_UART_TAG, "Addon board in upgrade mode");
                esp_restart();
            }else {
                ESP_LOGI(EXT_UART_TAG, "Booting factory partition not possible");
                cdc_write("OTA:not possible", strlen("OTA:not possible"));
                cdc_write(nl, sizeof(nl));
            }
        } else {
            ESP_LOGI(EXT_UART_TAG, "Factory partition not found");
            cdc_write("OTA:not possible", strlen("OTA:not possible"));
            cdc_write(nl, sizeof(nl));
        }
        return;
    }
    ESP_LOGW(EXT_UART_TAG,"No command executed with: %s ; len= %d\n",input,len);
}


void uart_parse_command (uint8_t character, struct cmdBuf * cmdBuffer)
{
    switch (cmdBuffer->state) {

    case CMDSTATE_IDLE:
        if (character==0xfd) {
            cmdBuffer->bufferLength=0;
            cmdBuffer->expectedBytes=8;   // 8 bytes for raw report size
            cmdBuffer->state=CMDSTATE_GET_RAW;
        }
        else if (character == '$') {
            cmdBuffer->bufferLength=0;   // we will read an ASCII-command until CR or LF
            cmdBuffer->state=CMDSTATE_GET_ASCII;
        }
        break;

    case CMDSTATE_GET_RAW:
        cmdBuffer->buf[cmdBuffer->bufferLength]=character;
        if ((cmdBuffer->bufferLength == 1) && (character==0x01)) { // we have a joystick report: increase by 5 bytes
            cmdBuffer->expectedBytes += 5;
            //ESP_LOGI(EXT_UART_TAG,"expecting 5 more bytes for joystick");
        }

        cmdBuffer->bufferLength++;
        cmdBuffer->expectedBytes--;
        if (!cmdBuffer->expectedBytes) {
            if(!isConnected()) {
                ESP_LOGI(EXT_UART_TAG,"not connected, cannot send report");
            } else {
                if (cmdBuffer->buf[1] == 0x00) {   // keyboard report
                    //if hid_conn_id is set (!= -1) we send to one device only. Send to all otherwise
                    if(hid_conn_id == -1)
                    {
                        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                        {
                            if(active_hid_conn_ids[i] != -1) esp_hidd_send_keyboard_value(active_hid_conn_ids[i],cmdBuffer->buf[0],&cmdBuffer->buf[2],6);
                        }
                    } else {
                        esp_hidd_send_keyboard_value(hid_conn_id,cmdBuffer->buf[0],&cmdBuffer->buf[2],6);
                    }
                    
                    //update timestamp
                    timestampLastSent = esp_timer_get_time();
                } else if (cmdBuffer->buf[1] == 0x01) {  // joystick report
                    ESP_LOGI(EXT_UART_TAG,"joystick: axis: 0x%X:0x%X:0x%X:0x%X, hat: %d",cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5],cmdBuffer->buf[8]);
                    ESP_LOGI(EXT_UART_TAG,"joystick: buttons: 0x%X:0x%X:0x%X:0x%X",cmdBuffer->buf[9],cmdBuffer->buf[10],cmdBuffer->buf[11],cmdBuffer->buf[12]);
                    //@todo should be HID_JOYSTICK_IN_RPT_LEN, but not available here.
                    uint8_t joy[11];
                    memcpy(joy,&cmdBuffer->buf[2],11);
                    //send joystick report
#if CONFIG_MODULE_USEJOYSTICK
                    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                    {
                        if(active_hid_conn_ids[i] != -1) esp_hidd_send_joy_report(active_hid_conn_ids[i],joy);
                    }
#else
                    ESP_LOGE(EXT_UART_TAG,"built without joystick support, cannot fix that!");
#endif
                } else if (cmdBuffer->buf[1] == 0x03) {  // mouse report
                    if(hid_conn_id == -1)
                    {
                        for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS; i++)
                        {
                            if(active_hid_conn_ids[i] != -1) esp_hidd_send_mouse_value(active_hid_conn_ids[i],cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5]);
                        }
                    } else {
                        esp_hidd_send_mouse_value(hid_conn_id,cmdBuffer->buf[2],cmdBuffer->buf[3],cmdBuffer->buf[4],cmdBuffer->buf[5]);
                    }
                    //update timestamp
                    timestampLastSent = esp_timer_get_time();
                    //and save mouse button state
                    mouseButtons = cmdBuffer->buf[2];
                    //ESP_LOGI(EXT_UART_TAG,"m: %d/%d",cmdBuffer->buf[3],cmdBuffer->buf[4]);
                }
                else ESP_LOGW(EXT_UART_TAG,"Unknown RAW HID packet");
            }
            cmdBuffer->state=CMDSTATE_IDLE;
        }
        break;

    case CMDSTATE_GET_ASCII:
        // collect a command string until CR or LF are received
        if ((character==0x0d) || (character==0x0a))  {
            cmdBuffer->buf[cmdBuffer->bufferLength]=0;
            ESP_LOGI(EXT_UART_TAG,"sending command to parser: %s",cmdBuffer->buf);

            processCommand(cmdBuffer);
            cmdBuffer->state=CMDSTATE_IDLE;
        } else {
            if (cmdBuffer->bufferLength < MAX_CMDLEN-1)
                cmdBuffer->buf[cmdBuffer->bufferLength++]=character;
        }
        break;
    default:
        cmdBuffer->state=CMDSTATE_IDLE;
    }
}


void cdc_bridge_task(void *pvParameters)
{
    ESP_LOGI(EXT_UART_TAG,"CDC Bridge task started");
    char character;
    struct cmdBuf cmdBuffer;
    cmdBuffer.state=CMDSTATE_IDLE;
    while(1)
    {
        // read & process a single byte
        cdc_read((uint8_t*) &character, 1, portMAX_DELAY);
        uart_parse_command(character, &cmdBuffer);
    }
}





void app_main(void)
{
    esp_err_t ret;

    init_serial_no();
    init_usb_phy();

    tusb_init();
    xTaskCreate(tusb_device_task, "tusb_device_task", 4 * 1024, NULL, 5, NULL);

    if( init_usb_rxtx() != ESP_OK ) {
        return;
    }

    
    // Initialize FreeRTOS elements
    eventgroup_system = xEventGroupCreate();
    if(eventgroup_system == NULL) ESP_LOGE(HID_TAG, "Cannot initialize event group");
    //if set in KConfig, pairing is disable by default.
    //User has to enable pairing with $PM1
#if CONFIG_MODULE_BT_PAIRING
    ESP_LOGI(HID_TAG,"pairing disabled by default");
    xEventGroupClearBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
    hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
#else
    ESP_LOGI(HID_TAG,"pairing enabled by default");
    xEventGroupSetBits(eventgroup_system,SYSTEM_PAIRING_ENABLED);
    hidd_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
#endif

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
    
    //open NVS handle for key/value storage via UART
    ESP_LOGI("MAIN","opening NVS handle for key/value storage");
    ret = nvs_open("kvstorage", NVS_READWRITE, &nvs_storage_h);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS for key/value storage");
    
    //read the appearance value for advertising
    uint8_t advapp;
    ret = nvs_get_u8(nvs_storage_h,"BLEAPPEAR",&advapp);
    if(ret == ESP_OK)
    {
        ESP_LOGI("MAIN","Setting appearance to 0x03C%d",advapp);
        hidd_adv_data.appearance = 0x03C0 + advapp;
    }
    
    // Read config
    nvs_handle my_handle;
    ESP_LOGI("MAIN","loading configuration from NVS");
    ret = nvs_open("config_c", NVS_READWRITE, &my_handle);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    size_t available_size = MAX_BT_DEVICENAME_LENGTH;
    strcpy(config.bt_device_name, GATTS_TAG);
    nvs_get_str (my_handle, "btname", config.bt_device_name, &available_size);
    if(ret != ESP_OK)
    {
        ESP_LOGI("MAIN","error reading NVS - bt name, setting to default");
        strcpy(config.bt_device_name, GATTS_TAG);
    } else ESP_LOGI("MAIN","bt device name is: %s",config.bt_device_name);

    //get from NVS if the joystick should be registered
#if CONFIG_MODULE_USEJOYSTICK
    config.joystick_active = 0;
    nvs_get_u8(my_handle, "joyactive", &config.joystick_active);
    ESP_LOGI("MAIN","Joystick: %d",config.joystick_active);
#endif
    
    //get locale
    ret = nvs_get_u8(my_handle, "locale", &config.locale);
    //if(ret != ESP_OK || config.locale >= LAYOUT_MAX)
    ///@todo implement keyboard layouts.
    if(ret != ESP_OK)
    {
        ESP_LOGI("MAIN","error reading NVS - locale, setting to US_INTERNATIONAL");
        //config.locale = LAYOUT_US_INTERNATIONAL;
    } else ESP_LOGI("MAIN","locale code is : %d",config.locale);
    nvs_close(my_handle);
    ///@todo How to handle the locale here? We have the memory for full lookups on the ESP32, but how to communicate this with the Teensy?
    
    ///clear the HID connection IDs&MACs
    for(uint8_t i = 0; i<CONFIG_BT_ACL_CONNECTIONS;i++)
    {
        active_hid_conn_ids[i] = -1;
        memset(active_connections[i],0,sizeof(esp_bd_addr_t));
    }
    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback,config.joystick_active);

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
    
    //start active scan
    xTaskCreate(&cdc_bridge_task, "bridge", 4096, NULL, configMAX_PRIORITIES, NULL);
    
    //start periodic timer to send HID reports
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodicHIDCallback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "HIDidle"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    //call every 100ms
    esp_timer_start_periodic(periodic_timer, 100000);
    
    //avoid unused variable warnings here:
    (void)hidd_adv_resp;
    (void)scan_params;
}
