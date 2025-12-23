#include "zigbee_app.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "tlc59108.h"
#include "int_temp_sensor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "ZIGBEE_APP";

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------


const uint16_t MIN_TEMP = 200;
const uint16_t MAX_TEMP = 455;
const uint16_t MID_TEMP = MIN_TEMP + (MAX_TEMP - MIN_TEMP)/2;


static bool zigbee_connected = false;

// Color temp range (mireds): 2200K=455 -> warm, 5000K=200 -> cool
#define CT_MIN_MIREDS 200
#define CT_MAX_MIREDS 455
int32_t new_mired = 0;
int32_t mired = MID_TEMP;
int32_t current_brightness = 128;


static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}


// ---------------------------------------------------------------------------
// Zigbee task main loop
// ---------------------------------------------------------------------------


static void esp_app_temp_sensor_handler_old(float temperature)
{
    int16_t measured_value = zb_temperature_to_s16(temperature);
    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_lock_release();
}

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
    

}


static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(temperature_sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(temperature_sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(temperature_sensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
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
                //light_driver_set_power(light_state);
            } else {
                ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
                {
                    uint16_t new_mired = *(uint16_t *)message->attribute.data.value;
                    led_color_temperature_control(current_brightness, new_mired);
                    ESP_LOGI(TAG, "Color sets to %i", (int)new_mired);
                    if (new_mired != mired) {
                        mired = new_mired;
                        SaveToNVS(mired, current_brightness);
                    }
                    
                }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : light_level;
                current_brightness = light_level;
                led_apply_brightness_and_ct(current_brightness, mired);
                ESP_LOGI(TAG, "Light level changes to %d", light_level);
                SaveToNVS(mired, current_brightness);
            } else {
                ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
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

    // later, if you want:
    // case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
    // case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
    // case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
    //    ...

    default:
        ESP_LOGD(TAG, "Ignored Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static esp_zb_ep_list_t *custom_temperature_sensor_ep_create(uint8_t endpoint_id, esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_temperature_sensor_clusters_create(temperature_sensor), endpoint_config);
    return ep_list;
}

void esp_zb_task_old_funke(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);


    // Handle ZCL attribute writes (you already had this)
    esp_zb_core_action_handler_register(zb_action_handler);
    ESP_LOGI(TAG, "ZCL action handler registered");
    
    
    // Create an empty endpoint list
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    
    // 1) Create HA Color Dimmable Light endpoint (10)
    esp_zb_color_dimmable_light_cfg_t light_cfg =
            ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();

    // Create cluster list for the light endpoint 
    esp_zb_cluster_list_t *light_clusters =
            esp_zb_color_dimmable_light_clusters_create(&light_cfg);

    // Add light endpoint 
    esp_zb_endpoint_config_t light_ep_cfg = {
        .endpoint           = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(ep_list, light_clusters, light_ep_cfg);
    
    /* Cluster for light on/off*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    /* Ane ikke, men mekke color cluster i guess*/
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = 0x0002,                                                               /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                          /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,  /*!<  The enhanced-mode which attribute determines the color of the device */ 
        .color_capabilities = 0x0010,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);

    uint16_t color_attr = MID_TEMP;
    uint16_t min_temp = MIN_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_DEFAULT_VALUE;
    uint16_t max_temp = MAX_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_DEFAULT_VALUE;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    

    /*----------------------------------------------------------
    * 2) Create HA Temperature Sensor endpoint (11)
    *---------------------------------------------------------*/
    esp_zb_temperature_sensor_cfg_t sensor_cfg =
            ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();

    sensor_cfg.temp_meas_cfg.min_value =
            zb_temperature_to_s16(TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value =
            zb_temperature_to_s16(TEMP_SENSOR_MAX_VALUE);

    esp_zb_cluster_list_t *temp_clusters =
            custom_temperature_sensor_clusters_create(&sensor_cfg);

    esp_zb_endpoint_config_t temp_ep_cfg = {
        .endpoint           = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(ep_list, temp_clusters, temp_ep_cfg);

    /*----------------------------------------------------------
     * 3) Register the device (both endpoints)
     *---------------------------------------------------------*/
    esp_zb_device_register(ep_list);
    
    /* Limit Color Control to “color temperature only” and set min/max */
    uint16_t caps = 0x0010;
    esp_zb_zcl_set_attribute_val(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_CAPABILITIES_ID,
        &caps, 
        false
    );
    ESP_LOGI(TAG, "asdads");
    uint16_t ct_min = CT_MIN_MIREDS;   // note: smaller K = larger mired 
    uint16_t ct_max = CT_MAX_MIREDS;

    esp_zb_zcl_set_attribute_val(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID,
        &ct_min, false);

    esp_zb_zcl_set_attribute_val(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID,
        &ct_max, false);

    /* Optionally: set manufacturer / model on the light endpoint too */
    esp_zb_zcl_set_attribute_val(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        MANUFACTURER_NAME, false);

    esp_zb_zcl_set_attribute_val(
        HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        MODEL_IDENTIFIER, false);

    

    /* You can also override manufacturer/model on endpoint 11 if you like */
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        MANUFACTURER_NAME, false);

    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        MODEL_IDENTIFIER, false);

    
    // ------------------------------ Cluster Temperature ------------------------------
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -50,
        .max_value = 100,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    /*----------------------------------------------------------
     * 4) Configure reporting
     *---------------------------------------------------------*/
    /*
    // 4.1 Temperature reporting on endpoint 11
    esp_zb_zcl_reporting_info_t temperature_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2,      // seconds
        .u.send_info.max_interval = 2,      // force every 2s
        .u.send_info.def_min_interval = 2,
        .u.send_info.def_max_interval = 2,
        .u.send_info.delta.u16 = 0,         // always report when polled
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_LOGI(TAG, "Configuring temperature reporting");
    esp_zb_zcl_update_reporting_info(&temperature_report);
    
    // 4.2 Brightness reporting on endpoint 10 (Level Control)
    esp_zb_zcl_reporting_info_t level_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 60,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 60,
        .attr_id = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    level_report.u.send_info.delta.u8 = 1;   // report when level changes by ≥1
    ESP_LOGI(TAG, "Configuring level reporting");
    esp_zb_zcl_update_reporting_info(&level_report);

    // 4.3 Color temperature reporting on endpoint 10 (Color Control)
    esp_zb_zcl_reporting_info_t ct_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 60,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 60,
        .attr_id = ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ct_report.u.send_info.delta.u16 = 1;    // report when CT changes ≥1 mired
    ESP_LOGI(TAG, "Configuring color temperature reporting");
    esp_zb_zcl_update_reporting_info(&ct_report);
    */
    /*----------------------------------------------------------
     * 5) Start network
     *---------------------------------------------------------*/
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_LOGI(TAG, "primary network channel set");
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started");
    esp_zb_stack_main_loop();
}

        
void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);


    // Handle ZCL attribute writes (you already had this)
    //esp_zb_core_action_handler_register(zb_action_handler);
    //ESP_LOGI(TAG, "ZCL action handler registered");

    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    
    // ------------------------------ Cluster LIGHT ------------------------------
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    /* Ane ikke, men mekke color cluster i guess*/
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = { 
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                          /*!<  The current value of the normalized chromaticity value y */ 
        .color_mode = 0x0002,                                                               /*!<  The mode which attribute determines the color of the device */ 
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                          /*!<  The bitmap determines behavior of some cluster commands */ 
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,  /*!<  The enhanced-mode which attribute determines the color of the device */ 
        .color_capabilities = 0x0010,                                                       /*!<  Specifying the color capabilities of the device support the color control cluster */ 
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);

    uint16_t color_attr = MID_TEMP;
    uint16_t min_temp = MIN_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_DEFAULT_VALUE;
    uint16_t max_temp = MAX_TEMP;//ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_DEFAULT_VALUE;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);
    
    // ============================= Level Cluster for test =============================
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    uint8_t level = 50;
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level);

    /*----------------------------------------------------------
    * 2) Create HA Temperature Sensor endpoint (11)
    *---------------------------------------------------------*/
    /*
    esp_zb_endpoint_config_t temp_ep_cfg = {
        .endpoint           = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(ep_list, temp_clusters, temp_ep_cfg);
    */

    // ------------------------------ Cluster Temperature ------------------------------
    
    
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -50,
        .max_value = 100,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);
    
    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list,esp_zb_temperature_meas_cluster,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    
    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
   // esp_zb_cluster_list_update_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_update_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  
    // ------------------------------ Create endpoint list ------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    //Create struct for esp_zb_ep_list_add_ep function from newest library
    //Warning! BitFields?!
    esp_zb_endpoint_config_t zb_endpoint_config = {
        .endpoint =  HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,                       /*!< Endpoint */
        .app_profile_id =  ESP_ZB_AF_HA_PROFILE_ID,               /*!< Application profile identifier */
        .app_device_id =  ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,       /*!< Application device identifier */
        .app_device_version = 4,                                  /*!< Application device version */
    };

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);
    /*----------------------------------------------------------
     * 3) Register the device (both endpoints)
     *---------------------------------------------------------*/
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_LOGI(TAG, "primary network channel set");
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
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            //ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                ESP_LOGI(TAG, "Applying saved LED state after reboot");
                ESP_LOGI(TAG, "Vals: %i brightness and %i mired", (int)current_brightness, (int)mired);
                led_color_temperature_control(current_brightness, mired);
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            ESP_LOGI(TAG, "Applying saved LED state after join");
            ESP_LOGI(TAG, "Vals: %i brightness and %i mired", (int)current_brightness, (int)mired);
            led_color_temperature_control(current_brightness, mired);
            
            
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


