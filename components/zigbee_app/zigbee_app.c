#include "zigbee_app.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include <string.h>
#include "tlc59108.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "events.h"
#include "status_led.h"
#include "wakeup_light.h"

static const char *TAG = "ZIGBEE_APP";

// Color temp range (mireds): 2200K=455 -> warm, 5000K=200 -> cool
const uint16_t MIN_TEMP = 200;
const uint16_t MAX_TEMP = 455;
const uint16_t MID_TEMP = MIN_TEMP + (MAX_TEMP - MIN_TEMP)/2;

int32_t new_mired = 0;
int32_t mired = MID_TEMP;
int32_t current_brightness = 128;

#define ZCL_WAKEUP_CLUSTER_ID          0xFF10   // manufacturer-specific
#define ZCL_WAKEUP_MANUFACTURER_CODE   0x1234   // pick your own

#define ATTR_WAKEUP_START_BRI          0x0001   // U8
#define ATTR_WAKEUP_END_BRI            0x0002   // U8
#define ATTR_WAKEUP_START_CT           0x0003   // U16
#define ATTR_WAKEUP_END_CT             0x0004   // U16
#define ATTR_WAKEUP_FADE_TIME_MS       0x0005   // U32
#define ATTR_WAKEUP_START            0x0006   // BOOL

#define CMD_WAKEUP_START               0x00
#define CMD_WAKEUP_STOP                0x01

#define CMD_WAKEUP_SET_CONFIG  0x10

// ---- Wakeup cluster backing variables (ZCL attribute storage) ----
static uint8_t  s_wakeup_start_bri    = 1;
static uint8_t  s_wakeup_end_bri      = 128;
static uint16_t s_wakeup_start_ct     = 455;
static uint16_t s_wakeup_end_ct       = 200;
static uint32_t s_wakeup_fade_time_ms = 15 * 60 * 1000UL;
static uint8_t  s_wakeup_enabled      = 1; // use uint8_t for ZCL bool storage


static uint16_t rd_u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t rd_u32_le(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }


void SaveToNVS()
{
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW("SAVE", "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(my_handle, "saved_color", (int32_t)mired);
    if (err != ESP_OK) {
        ESP_LOGW("SAVE", "Failed to save color: %s", esp_err_to_name(err));
        goto out;
    }

    //int32_t brightness_save = (current_brightness >= 100) ? (int32_t)current_brightness : 100;
    int32_t brightness_save = current_brightness;
    err = nvs_set_i32(my_handle, "brightness", brightness_save);
    if (err != ESP_OK) {
        ESP_LOGW("SAVE", "Failed to save brightness: %s", esp_err_to_name(err));
        goto out;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGW("SAVE", "nvs_commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("SAVE", "NVS commit OK");
    }
    

out:
    nvs_close(my_handle);
}

void LoadFromNVS(){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    
    int32_t saved_color = MID_TEMP; 
    err = nvs_get_i32(my_handle, "saved_color", &saved_color);
    ESP_LOGI(TAG, "Loaded %i mired from NVS", (int)saved_color);
    switch (err) {
        case ESP_OK:                    ESP_LOGI("LOAD", "Found color temperature value: %i", (int)saved_color); break;
        case ESP_ERR_NVS_NOT_FOUND:     ESP_LOGW("LOAD", "Color temperature value is not found. Default color temperature value (%i) used", (int)MID_TEMP); break;
        default :                       ESP_LOGW("LOAD", "Reading error!!! Default color temperature value (%i) used", (int)MID_TEMP);
    }

    mired = saved_color;

    int32_t saved_brightness = current_brightness; 
    err = nvs_get_i32(my_handle, "brightness", &saved_brightness);
    ESP_LOGI(TAG, "Loaded %i brightness from NVS", (int)saved_brightness);
    switch (err) {
        case ESP_OK:                    ESP_LOGI("LOAD", "Found brightness value: %i", (int)saved_brightness); break;
        case ESP_ERR_NVS_NOT_FOUND:     ESP_LOGW("LOAD", "Brightness value is not found. Default brightness value (%i) used", (int)current_brightness); break;
        default :                       ESP_LOGW("LOAD", "Reading error!!! Default brightness value (%i) used", (int)current_brightness);
    }

    current_brightness = saved_brightness;
    ESP_LOGI(TAG, "Loaded %i brightness and %i mired from NVS", (int)current_brightness, (int)mired);
    
    //return saved_color, saved_brightness;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_COLOR_DIMMABLE_LIGHT_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                light_set_on(light_state, false);
                //light_driver_set_power(light_state);
            } else {
                ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
                {
                    uint16_t new_mired = *(uint16_t *)message->attribute.data.value;
                    //led_color_temperature_control(current_brightness, new_mired);
                    ESP_LOGI(TAG, "Color sets to %i", (int)new_mired);
                    if (new_mired != mired) {
                        mired = new_mired;
                        tlc_set_ct_mired_smooth(mired, 400);
                        SaveToNVS(mired, current_brightness);
                    }
                    
                }
            break;
            
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : light_level;
                ESP_LOGI(TAG, "Light level changes to %d", light_level);
                //current_brightness = light_level;
                //led_apply_brightness_and_ct(current_brightness, mired);
                if (light_level > 0) {
                    current_brightness = light_level;
                    light_remember_brightness(light_level);
                }
                //current_brightness = light_level;
                tlc_set_logical_brightness_smooth((uint8_t)current_brightness, (uint16_t)mired);
                
                SaveToNVS(mired, current_brightness);
            } else {
                ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        
        case ZCL_WAKEUP_CLUSTER_ID: {
            wakeup_cfg_t cfg = wakeup_get();
            ESP_LOGW(TAG, "FC10 write attr=0x%04x type=0x%02x", message->attribute.id, message->attribute.data.type);

            switch (message->attribute.id) {
                case ATTR_WAKEUP_START_BRI:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                        cfg.start_bri = *(uint8_t*)message->attribute.data.value;
                        s_wakeup_start_bri = cfg.start_bri;
                        ESP_LOGI(TAG, "Wakeup start brightness set to %d", cfg.start_bri);
                    }
                    break;

                case ATTR_WAKEUP_FADE_TIME_MS:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U32) {
                        cfg.fade_time_ms = *(uint32_t*)message->attribute.data.value;
                        s_wakeup_fade_time_ms = cfg.fade_time_ms;
                        ESP_LOGI(TAG, "Wakeup fade time set to %lu ms", (unsigned long)cfg.fade_time_ms);
                    }
                    break;
                case ATTR_WAKEUP_END_BRI:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                        cfg.end_bri = *(uint8_t*)message->attribute.data.value;
                        s_wakeup_end_bri = cfg.end_bri;
                        ESP_LOGI(TAG, "Wakeup end brightness set to %d", cfg.end_bri);
                    }
                    break;
                case ATTR_WAKEUP_START_CT:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        cfg.start_ct_mired = *(uint16_t*)message->attribute.data.value;
                        s_wakeup_start_ct = cfg.start_ct_mired;
                        ESP_LOGI(TAG, "Wakeup start color temp set to %d mired", cfg.start_ct_mired);
                    }
                    break;
                case ATTR_WAKEUP_END_CT:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                        cfg.end_ct_mired = *(uint16_t*)message->attribute.data.value;
                        s_wakeup_end_ct = cfg.end_ct_mired;
                        ESP_LOGI(TAG, "Wakeup end color temp set to %d mired", cfg.end_ct_mired);
                    }
                    break;
                case ATTR_WAKEUP_START:
                    if (message->attribute.data.value && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                        cfg.enabled = *(bool*)message->attribute.data.value;
                        s_wakeup_enabled = cfg.enabled ? 1 : 0;
                        ESP_LOGI(TAG, "Wakeup enabled set to %s", cfg.enabled ? "true" : "false");
                    }
                    break;
                //case CMD_WAKEUP_START:
                //   ESP_LOGI(TAG, "Wakeup command: START");
                //    break;

                // ... rest: update cfg + s_wakeup_* each time ...
            }

            wakeup_set(&cfg);
            wakeup_save_to_nvs();

            // Optional: report attributes so zigbee2mqtt updates instantly
            // wakeup_report_all();  (you can add helper later)

            break;
        }
        
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}


static esp_err_t zb_default_resp_handler(const esp_zb_zcl_cmd_default_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty default resp message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Default response: error status(%d)", message->info.status);

    ESP_LOGI(TAG,
             "Default response: cluster(0x%x), cmd(0x%x), status(0x%x)",
             message->info.cluster,
             message->resp_to_cmd,
             message->status_code);
    return ESP_OK;
}

void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0001,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void zigbee_update_temp_rh(float temperature_c, float rh_percent)
{
    if (!esp_zb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "Not joined yet, skipping temp/RH update");
        return;
    }

    // Convert temperature: degC -> centi-degC
    int16_t temp_measured = (int16_t)(temperature_c * 100.0f);

    // Clamp + convert RH: % -> 0.01% units
    if (rh_percent < 0.0f)   rh_percent = 0.0f;
    if (rh_percent > 100.0f) rh_percent = 100.0f;
    uint16_t rh_measured = (uint16_t)(rh_percent * 100.0f);

    ESP_LOGI(TAG, "Updating Zigbee: T=%d (0.01C), RH=%u (0.01%%)",
             temp_measured, rh_measured);

    esp_zb_lock_acquire(portMAX_DELAY);

    // ---- Temperature attribute update ----
    esp_zb_zcl_attr_t *temp_attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID
    );

    if (temp_attr && temp_attr->data_p) {
        memcpy(temp_attr->data_p, &temp_measured, sizeof(temp_measured));
    } else {
        ESP_LOGW(TAG, "Temp attribute not found");
    }

    // ---- Humidity attribute update ----
    esp_zb_zcl_attr_t *rh_attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID
    );

    if (rh_attr && rh_attr->data_p) {
        memcpy(rh_attr->data_p, &rh_measured, sizeof(rh_measured));
    } else {
        ESP_LOGW(TAG, "RH attribute not found");
    }

    // One-shot report: Temperature
    if (temp_attr && temp_attr->data_p) {
        esp_zb_zcl_report_attr_cmd_t cmd_t = {0};
        cmd_t.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // coordinator
        cmd_t.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
        cmd_t.zcl_basic_cmd.dst_endpoint = 1;               // coordinator endpoint
        cmd_t.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd_t.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd_t.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        cmd_t.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        esp_zb_zcl_report_attr_cmd_req(&cmd_t);
    }

    // One-shot report: Humidity
    if (rh_attr && rh_attr->data_p) {
        esp_zb_zcl_report_attr_cmd_t cmd_rh = {0};
        cmd_rh.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // coordinator
        cmd_rh.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
        cmd_rh.zcl_basic_cmd.dst_endpoint = 1;               // coordinator endpoint
        cmd_rh.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd_rh.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd_rh.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        cmd_rh.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        esp_zb_zcl_report_attr_cmd_req(&cmd_rh);
    }

    esp_zb_lock_release();
}


void zigbee_update_temperature(float temperature)
{
    int16_t measured_value = (int16_t)(temperature * 100); // centi-deg

    if (!esp_zb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "Not joined yet, skipping temp update");
        return;
    }

    ESP_LOGI(TAG, "Updating temperature to %d (centi-deg)", measured_value);

    esp_zb_lock_acquire(portMAX_DELAY);

    // Update the local ZCL attribute value in the stack
    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID
    );

    if (attr && attr->data_p) {
        memcpy(attr->data_p, &measured_value, sizeof(measured_value));
    } else {
        ESP_LOGW(TAG, "Temp attribute not found");
        esp_zb_lock_release();
        return;
    }

    // One-shot report
    esp_zb_zcl_report_attr_cmd_t cmd = {0};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // coordinator
    cmd.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
    cmd.zcl_basic_cmd.dst_endpoint = 1;               // coordinator endpoint (often 1)
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;

    esp_zb_zcl_report_attr_cmd_req(&cmd);

    esp_zb_lock_release();
}


static esp_err_t zb_action_handler_testing(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    switch (callback_id) {
        // This is often where Manufacturer-Specific writes land in older SDKs
        case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
            esp_zb_zcl_custom_cluster_command_message_t *cmd_msg = (esp_zb_zcl_custom_cluster_command_message_t *)message;
            ESP_LOGW(TAG, "Custom Cluster Received: 0x%04x", cmd_msg->info.cluster);
            break;
        }

        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
            esp_zb_zcl_set_attr_value_message_t *set_attr_msg = (esp_zb_zcl_set_attr_value_message_t *)message;
            ESP_LOGI(TAG, "Standard Attr Set: Cluster 0x%04x, Attr 0x%04x", 
                     set_attr_msg->info.cluster, 
                     set_attr_msg->attribute.id);
            break;
        }
        
        default:
            ESP_LOGD(TAG, "Other Zigbee action: 0x%x", callback_id);
            break;
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;

    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        ret = zb_default_resp_handler((const esp_zb_zcl_cmd_default_resp_message_t *)message);
        break;
    
    // might add later
    // case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
    // case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
    // case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:

    default:
        ESP_LOGD(TAG, "Ignored Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}



void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // ---------------- Basic cluster ----------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };

    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion       = 0x0002;
    uint32_t HWVersion          = 0x0002;
    uint8_t ManufacturerName[]  = {7, 'C', 'K', '-', 'H', 'o', 'm', 'e'};
    uint8_t ModelIdentifier[]   = {14, 'C', 'C', 'T', '-', 'S', 'm', 'a', 'r', 't', 'L', 'a', 'm', 'p'};
    uint8_t DateCode[]          = {8, '2', '0', '2', '5', '1', '2', '2', '6'};

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    esp_zb_attribute_list_t *wakeup_cluster = esp_zb_zcl_attr_list_create(ZCL_WAKEUP_CLUSTER_ID);
    
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_START_BRI,
    ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_start_bri));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_END_BRI,
        ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_end_bri));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_START_CT,
        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_start_ct));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_END_CT,
        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_end_ct));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_FADE_TIME_MS,
        ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_fade_time_ms));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(wakeup_cluster, ZCL_WAKEUP_CLUSTER_ID, ATTR_WAKEUP_START,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_wakeup_enabled));

    // ---------------- Identify cluster ----------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // ---------------- On/Off cluster ----------------
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // ---------------- Level cluster ----------------
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    uint8_t level = 50;
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level);

    // ---------------- Color Control cluster ----------------
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = {
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,
        .color_mode = 0x0002,
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,
        .color_capabilities = 0x0010, // CT capable
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);

    uint16_t color_attr = MID_TEMP;
    uint16_t min_temp   = MIN_TEMP;
    uint16_t max_temp   = MAX_TEMP;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);

    // ---------------- Temperature Measurement cluster ----------------
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0x8000,
        .min_value = -4000,
        .max_value = 12500,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    // ---------------- Humidity Measurement cluster ----------------
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value      = 0,
        .max_value      = 10000,
    };
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg);

    // ---------------- Cluster list ----------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

    ESP_LOGW(TAG, "REGISTERING WAKEUP CLUSTER 0x%04X on endpoint %d", ZCL_WAKEUP_CLUSTER_ID, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT);

    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_update_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Add wakeup cluster to endpoint
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(
        esp_zb_cluster_list, wakeup_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // ---------------- Endpoint list ----------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t zb_endpoint_config = {
        .endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);

    // ---------------- Reporting configuration ----------------
    esp_zb_zcl_reporting_info_t temperature_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 2,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&temperature_report);

    esp_zb_zcl_reporting_info_t humidity_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 2,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&humidity_report);
    ESP_LOGW(TAG, "Reporting configured");
    
    // ---------------- Start stack ----------------
    
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    esp_zb_zcl_attr_t *t1 = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ZCL_WAKEUP_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_WAKEUP_START_BRI
    );
    ESP_LOGW(TAG, "Lookup FC10/0001 => %p data_p=%p", t1, t1 ? t1->data_p : NULL);

    esp_zb_zcl_attr_t *t5 = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ZCL_WAKEUP_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_WAKEUP_FADE_TIME_MS
    );
    ESP_LOGW(TAG, "Lookup FC10/0005 => %p data_p=%p", t5, t5 ? t5->data_p : NULL);

    esp_zb_zcl_attr_t *t4 = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ZCL_WAKEUP_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ATTR_WAKEUP_END_CT  // 0x0004
    );
    ESP_LOGW(TAG, "Lookup FC10/0004 => %p data_p=%p", t4, t4 ? t4->data_p : NULL);

    esp_zb_set_node_descriptor_manufacturer_code(0x1234);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}


void esp_zb_task_old(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Set up basic cluster configuration
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    

    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {7, 'C', 'K', '-', 'H', 'o', 'm', 'e'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {14, 'C', 'C', 'T', '-', 'S', 'm', 'a', 'r', 't', 'L', 'a', 'm', 'p'};
    uint8_t DateCode[] = {8, '2', '0', '2', '5', '1', '2', '2', '6'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    // Set up identify cluster configuration
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    
    // Set up on/off cluster configuration
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);


    // Set up color control cluster configuration
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = 0x0002,                                                               /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                          /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,  /*!<  The enhanced-mode which attribute determines the color of the device */ 
        .color_capabilities = 0x0010,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);
    // Add color control attributes
    uint16_t color_attr = MID_TEMP;
    uint16_t min_temp = MIN_TEMP;
    uint16_t max_temp = MAX_TEMP;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    
    // Set up level control cluster configuration
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    uint8_t level = 50;
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level);

    // Set up temperature measurement cluster configuration
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0x8000,
        .min_value = -4000,
        .max_value = 12500,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    // Set up humidity measurement cluster configuration
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = 0xFFFF,   // "invalid/unknown" start value is typical for MeasuredValue
        .min_value      = 0,
        .max_value      = 10000,    // 100.00% (units are 0.01%)
    };
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg);

    
    // Create cluster list and add clusters
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_update_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // Create endpoint configuration
    esp_zb_endpoint_config_t zb_endpoint_config = {
        .endpoint =  HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,                       /*!< Endpoint */
        .app_profile_id =  ESP_ZB_AF_HA_PROFILE_ID,               /*!< Application profile identifier */
        .app_device_id =  ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,       /*!< Application device identifier */
        .app_device_version = 4,                                  /*!< Application device version */
    };
    // Add endpoint to endpoint list
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);
    // Register device endpoint list
    esp_zb_device_register(esp_zb_ep_list);

    // Configure temperature reporting
    esp_zb_zcl_reporting_info_t temperature_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 2,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 0, 
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&temperature_report);

    // Configure humidity reporting
    esp_zb_zcl_reporting_info_t humidity_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, // cluster 0x0405 :contentReference[oaicite:4]{index=4}
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 2,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 0, // report on any change (or set e.g. 50 == 0.50%)
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&humidity_report);


    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        status_led_set_state(STATUS_LED_STATE_JOINING_NETWORK);
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                status_led_set_state(STATUS_LED_STATE_JOINING_NETWORK);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                ESP_LOGI(TAG, "Applying saved LED state after reboot");
                ESP_LOGI(TAG, "Vals: %i brightness and %i mired", (int)current_brightness, (int)mired);
                /*
                zigbee_connection_confirmed_sequence(current_brightness);
                tlc_set_ct_mired_smooth((uint16_t)mired, 400);
                tlc_set_logical_brightness_smooth((uint8_t)current_brightness, (uint16_t)mired);
                status_led_set_state(STATUS_LED_STATE_NORMAL_OPERATION);
                light_set_on(true, true);
                */
                
                
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        //status_led_set_state(STATUS_LED_STATE_JOINING_NETWORK);
        if (err_status == ESP_OK) {
            //status_led_set_state(STATUS_LED_STATE_JOINED_SUCCESSFULLY);
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            /*
            ESP_LOGI(TAG, "Applying saved LED state after join");
            ESP_LOGI(TAG, "Vals: %i brightness and %i mired", (int)current_brightness, (int)mired);
            zigbee_connection_confirmed_sequence(current_brightness);
            tlc_set_ct_mired_smooth((uint16_t)mired, 400);
            tlc_set_logical_brightness_smooth((uint8_t)current_brightness, (uint16_t)mired);
            status_led_set_state(STATUS_LED_STATE_NORMAL_OPERATION);
            light_set_on(true, true);
            */
            
                
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            
            
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

bool zigbee_is_connected(void)
{
    return esp_zb_bdb_dev_joined();
}

void zigbee_event_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "zigbee_event_task started");
    while (1) {
        app_event_t ev;
        if (events_wait(&ev, portMAX_DELAY)) {
            if (ev == APP_EVENT_ZIGBEE_FACTORY_RESET) {
                ESP_LOGI(TAG, "Performing Zigbee factory reset");
                esp_zb_bdb_reset_via_local_action();
            }
        }
    }
}

void zigbee_set_onoff_and_report(bool on)
{
    if (!esp_zb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "Not joined yet, skipping on/off report");
        return;
    }

    esp_zb_lock_acquire(portMAX_DELAY);

    // Update local ZCL attribute
    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID
    );

    if (attr && attr->data_p) {
        uint8_t v = on ? 1 : 0; // bool is usually fine too, but u8 is safe
        memcpy(attr->data_p, &v, sizeof(v));
    } else {
        ESP_LOGW(TAG, "On/Off attribute not found");
        esp_zb_lock_release();
        return;
    }

    // Report to coordinator
    esp_zb_zcl_report_attr_cmd_t cmd = {0};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // coordinator
    cmd.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
    cmd.zcl_basic_cmd.dst_endpoint = 1;               // coordinator endpoint
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
    cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;

    esp_zb_zcl_report_attr_cmd_req(&cmd);

    esp_zb_lock_release();
}

void zigbee_set_level_and_report(uint8_t level)
{
    if (!esp_zb_bdb_dev_joined()) return;

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID
    );

    if (attr && attr->data_p) {
        memcpy(attr->data_p, &level, sizeof(level));
    } else {
        ESP_LOGW(TAG, "Level attribute not found");
        esp_zb_lock_release();
        return;
    }

    esp_zb_zcl_report_attr_cmd_t cmd = {0};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
    cmd.attributeID = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID;

    esp_zb_zcl_report_attr_cmd_req(&cmd);

    esp_zb_lock_release();
}

void zigbee_set_ct_and_report(uint16_t mired_value)
{
    if (!esp_zb_bdb_dev_joined()) return;

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID
    );

    if (attr && attr->data_p) {
        memcpy(attr->data_p, &mired_value, sizeof(mired_value));
    } else {
        ESP_LOGW(TAG, "CT attribute not found");
        esp_zb_lock_release();
        return;
    }

    esp_zb_zcl_report_attr_cmd_t cmd = {0};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
    cmd.attributeID = ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID;

    esp_zb_zcl_report_attr_cmd_req(&cmd);

    esp_zb_lock_release();
}





