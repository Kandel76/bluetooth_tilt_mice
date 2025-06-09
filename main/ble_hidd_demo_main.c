/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "esp_bt.h"
#include <math.h>
#include "string.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"

/* this is for the IMU*/
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SCL_IO     8
#define I2C_MASTER_SDA_IO     10
#define I2C_MASTER_FREQ_HZ    1160
#define ICM42670_ADDR         0x68

#define REG_PWR_MGMT0         0x1F
#define REG_ACCEL_CONFIG0     0x50
#define REG_ACCEL_DATA_X1     0x0B
#define REG_WHO_AM_I          0x75

#define ACCEL_SENSITIVITY_LSB_PER_G 2048.0f  // For ±16g
#define TILT_THRESHOLD_G      0.3f           // Tilt threshold in g
static const char *TAG = "TILT";
/*IMU Definition Over*/

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};



/*IMU and I2C Init*/
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void icm42670_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, ICM42670_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
}

void icm42670_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_master_write_read_device(I2C_MASTER_NUM, ICM42670_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

void icm42670_init(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    icm42670_write(REG_PWR_MGMT0, 0x1F);      // Low noise mode: accel + gyro
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t pwr_check = 0;
    icm42670_read(REG_PWR_MGMT0, &pwr_check, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    printf("PWR_MGMT0 = 0x%02X\n", pwr_check);  // Should print 0x0F
    icm42670_write(REG_ACCEL_CONFIG0, 0x05);  // ±16g, 1.125kHz ODR
    vTaskDelay(pdMS_TO_TICKS(50));
}

bool icm42670_read_accel_g(float *x, float *y, float *z) {
    uint8_t raw[6];
    icm42670_read(REG_ACCEL_DATA_X1, raw, 6);

    int16_t ax_raw = (raw[0] << 8) | raw[1];
    int16_t ay_raw = (raw[2] << 8) | raw[3];
    int16_t az_raw = (raw[4] << 8) | raw[5];

    *x = ax_raw / ACCEL_SENSITIVITY_LSB_PER_G;
    *y = ay_raw / ACCEL_SENSITIVITY_LSB_PER_G;
    *z = az_raw / ACCEL_SENSITIVITY_LSB_PER_G;

    return true;
}

void detect_orientation(float x, float y) {
    char dir[32] = "";

    if (y > TILT_THRESHOLD_G) strcat(dir, "UP ");
    else if (y < -TILT_THRESHOLD_G) strcat(dir, "DOWN ");

    if (x > TILT_THRESHOLD_G) strcat(dir, "RIGHT");
    else if (x < -TILT_THRESHOLD_G) strcat(dir, "LEFT");

   // if (dir[0]) printf("Tilt: %s\n", dir);

    if (dir[0]){
    // ESP_LOGI(TAG,"TILT: %s", dir);
    }
}

/*Init of I2C and IMU Over*/


static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
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
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
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
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
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
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    // const float sensitivity = 10.0f; // Adjust sensitivity for mouse movement
    // const int step_delay_ms = 50;    // Delay between mouse updates
    // printf("in demo task function\n");

    // while (1) {
    //     if (sec_conn) { // Check if BLE connection is established
    //         float ax, ay, az;
    //         if (icm42670_read_accel_g(&ax, &ay, &az)) {
    //             // Map accelerometer values to mouse movement
    //             int16_t mouse_x = (int16_t)(ax * sensitivity); // Scale X-axis movement
    //             int16_t mouse_y = (int16_t)(-ay * sensitivity); // Scale Y-axis movement (invert for natural movement)

    //             // Send mouse movement report
    //             esp_hidd_send_mouse_value(hid_conn_id, 0, mouse_x, mouse_y);
    //             // void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y)

    //             ESP_LOGI(HID_DEMO_TAG, "Mouse Move: X=%d, Y=%d", mouse_x, mouse_y);
            
    //         }
    //     }

    //     // Delay between updates
    //     vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    // }
    const float sensitivity = 50.0f; // Base sensitivity for mouse movement
    const int step_delay_ms = 50;    // Delay between mouse updates
    printf("in demo task function\n");

    while (1) {
        if (sec_conn) { // Check if BLE connection is established
            float ax, ay, az;
            if (icm42670_read_accel_g(&ax, &ay, &az)) {
                // Apply non-linear scaling to accelerometer values
                float scaled_ax = ax * fabs(ax) * sensitivity; // Square the value for acceleration
                float scaled_ay = ay * fabs(ay) * sensitivity; // Square the value for acceleration

                // Map scaled accelerometer values to mouse movement
                int16_t mouse_x = (int16_t)scaled_ax; // Scale X-axis movement
                int16_t mouse_y = (int16_t)(-scaled_ay); // Scale Y-axis movement (invert for natural movement)

                // Send mouse movement report
                esp_hidd_send_mouse_value(hid_conn_id, 0, mouse_x, mouse_y);

                ESP_LOGI(HID_DEMO_TAG, "Mouse Move: X=%d, Y=%d", mouse_x, mouse_y);
            }
        }

        // Delay between updates
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}



void app_main(void)
{
    esp_err_t ret;
    i2c_master_init();
    
    //check if c
    uint8_t id;
    icm42670_read(REG_WHO_AM_I, &id, 1);
    //printf("WHO_AM_I = 0x%02X\n", id);  // Expect 0x67
    if (id != 0x67) {
        printf("ICM-42670-P not detected.\n");
        return;
    }

    icm42670_init();

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
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
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

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
