#include "zigbee_app.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "tlc59108.h"
#include "int_temp_sensor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//static const char *TAG = "zigbee_cct";
// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

// TLC59108 channels: warm = 0–3, cool = 4–7
//static const uint8_t warm_channels[] = {0, 1, 2, 3};
//static const uint8_t cool_channels[] = {4, 5, 6, 7};

const uint16_t MIN_TEMP = 200;
const uint16_t MAX_TEMP = 455;
const uint16_t MID_TEMP = MIN_TEMP + (MAX_TEMP - MIN_TEMP)/2;

//static uint16_t COLOR_TEMP = MID_TEMP;

static bool zigbee_connected = false;

// Color temp range (mireds): 2200K=455 -> warm, 5000K=200 -> cool
#define CT_MIN_MIREDS 200
#define CT_MAX_MIREDS 455
int32_t mired = MID_TEMP;

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}


// ---------------------------------------------------------------------------
// Zigbee task main loop
// ---------------------------------------------------------------------------
static const char *TAG = "ZIGBEE_APP";

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
/*
void SaveToNVS(){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, "saved_color", (int32_t) mired);
    switch (err) {
        case ESP_OK: ESP_LOGI("SAVE", "Saved color temperature value: %i", (int)mired); break;       
        default :    ESP_LOGW("SAVE", "Saving error!!!");
    }
    brightness_save = current_brightness;
    if (brightness_save >= 100) {
        brightness_save = (int32_t)brightness_save;
    }
    err = nvs_set_i32(my_handle, "saved_brightness", brightness_save);
    switch (err) {
        case ESP_OK: ESP_LOGI("SAVE", "Saved brightness value: %i", (int)brightness_save); break;       
        default :    ESP_LOGW("SAVE", "Saving error!!!");
    }
}
*/
static void esp_app_temp_sensor_handler(float temperature)
{
    int16_t measured_value = zb_temperature_to_s16(temperature);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &measured_value,
        false);
    esp_zb_lock_release();
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


static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
    if (!is_inited) {
        ESP_RETURN_ON_ERROR(
            temp_sensor_driver_init(&temp_sensor_config, ESP_TEMP_SENSOR_UPDATE_INTERVAL, esp_app_temp_sensor_handler),
            TAG, "Failed to initialize temperature sensor");
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}


static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
    //uint16_t light_color_x = 0;
    //uint16_t light_color_y = 0;
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
                    uint16_t mired = *(uint16_t *)message->attribute.data.value;
                    led_color_temperature_control(mired);
                    ESP_LOGI(TAG, "Color sets to %i", (int)mired);
                    //SaveToNVS(mired, current_brightness);
                }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : light_level;
                current_brightness = light_level;
                led_apply_brightness_and_ct();
                ESP_LOGI(TAG, "Light level changes to %d", light_level);
                //SaveToNVS(mired, current_brightness);
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

void esp_zb_task_old(void *pvParameters)
{
    /* Initialize Zigbee stack */
    ESP_LOGI(TAG, "esp_zb_init starting");
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    ESP_LOGI(TAG, "esp_zb_init completed");

    esp_zb_core_action_handler_register(zb_action_handler);
    ESP_LOGI(TAG, "ZCL action handler registered");

    /* Create customized temperature sensor endpoint */
    esp_zb_temperature_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    /* Set (Min|Max)MeasuredValure */
    sensor_cfg.temp_meas_cfg.min_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);
    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_temperature_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t temperature_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2, // var 1
        .u.send_info.max_interval = 2, // var 0
        .u.send_info.def_min_interval = 2, //var 1
        .u.send_info.def_max_interval = 2, // var 0
        .u.send_info.delta.u16 = 0, //var 100
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_LOGI(TAG, "reporting info set");
    esp_zb_zcl_update_reporting_info(&temperature_report);
    ESP_LOGI(TAG, "reporting info updated for temperature and setting primary network channel");

    esp_zb_zcl_reporting_info_t led_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,

        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 60,   // or 5 or 10, whatever you want
        .attr_id = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        //.u.send_info.delta.u8 = 1,                    // report when brightness changes by > 1
    };
    ESP_LOGI(TAG, "led report info");
    esp_zb_zcl_update_reporting_info(&led_report);
    ESP_LOGI(TAG, "led report set");

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_LOGI(TAG, "primary network channel set");
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "error check");
    esp_zb_stack_main_loop();
    ESP_LOGI(TAG, "zb main loop started");
    
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
            zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    sensor_cfg.temp_meas_cfg.max_value =
            zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);

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
    esp_zb_core_action_handler_register(zb_action_handler);
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
    // ------------------------------ Cluster Temperature ------------------------------
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -50,
        .max_value = 100,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);



    /*
    esp_zb_endpoint_config_t temp_ep_cfg = {
        .endpoint           = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(ep_list, temp_clusters, temp_ep_cfg);
    */

    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    //esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    
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

    /*----------------------------------------------------------
     * 5) Start network
     *---------------------------------------------------------*/
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_LOGI(TAG, "primary network channel set");
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started");
    esp_zb_stack_main_loop();
}

void esp_zb_task_fuck(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {8, 'N', 'i', 'k', 'a', 'H', 'o', 'm', 'e'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {5, 'A', 'L', 'a', 'm', 'p'};
    uint8_t DateCode[] = {8, '2', '0', '2', '4', '0', '5', '0', '3'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

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

    // ===============================Color Cluster for test==============================
    // It works with HA!!!
    // Use esp_zb_color_control_cluster_add_attr for this
    //esp_zb_color_control_cluster_create
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
    //For old library
    //esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_ESP_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);

    // ------------------------------ Register Device ------------------------------
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
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


