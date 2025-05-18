/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Sleepy end device Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_check.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_window_covering.h"
#include "esp_zb_sleepy_end_device.h"
#include "switch_driver.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "zboss_api.h"
#include "servo_control.h"
#include "servo_calibration.h"

/**
 * @note Make sure set idf.py menuconfig in zigbee component as zigbee end device!
 */
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

// Хэндл для задачи управления сервоприводом
static TaskHandle_t servo_task_handle = NULL;

static switch_func_pair_t button_func_pair[] = {
    {CONFIG_GPIO_INPUT_IO_WAKEUP, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL)
    {
        // Выполняем сброс Zigbee через локальное действие
        ESP_LOGI(TAG, "Выполняем сброс Zigbee...");
        esp_zb_bdb_reset_via_local_action();

        // Запускаем процесс подключения к сети
        ESP_LOGI(TAG, "Запускаем процесс подключения к сети...");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited)
    {
        ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler),
                            ESP_FAIL, TAG, "Failed to initialize switch driver");
        /* Configure RTC IO wake up:
        The configuration mode depends on your hardware design.
        Since the BOOT button is connected to a pull-up resistor, the wake-up mode is configured as LOW.
        */
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(1ULL << CONFIG_GPIO_INPUT_IO_WAKEUP, ESP_EXT1_WAKEUP_ANY_LOW));

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
        rtc_gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#else
        gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
        gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#endif
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // ESP_LOGI(TAG, "Zigbee can sleep");
        // esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = false
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

static esp_err_t nvs_write_servo_position(bool position)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    uint8_t value_to_write = position ? 1 : 0;
    err = nvs_set_u8(nvs_handle, NVS_KEY, value_to_write);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) writing NVS key '%s'!", esp_err_to_name(err), NVS_KEY);
    }
    else
    {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) committing NVS changes!", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Servo position (%s) saved to NVS", position ? "Open" : "Close");
        }
    }

    nvs_close(nvs_handle);
    return err;
}

static esp_err_t nvs_read_servo_position(bool *position)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for reading!", esp_err_to_name(err));
        *position = false;
        return err;
    }

    uint8_t value_read = 0;
    err = nvs_get_u8(nvs_handle, NVS_KEY, &value_read);
    if (err == ESP_OK)
    {
        *position = (value_read == 1);
        ESP_LOGI(TAG, "Servo position (%s) read from NVS", *position ? "Open" : "Close");
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Servo position key '%s' not found in NVS. Using default (Close).", NVS_KEY);
        *position = false;
        err = ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Error (%s) reading NVS key '%s'! Using default (Close).", esp_err_to_name(err), NVS_KEY);
        *position = false;
    }

    nvs_close(nvs_handle);
    return err;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool servo_position = false; // false = Close/Off, true = Open/On

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) // Используем тот же эндпоинт
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                if (message->attribute.data.value)
                {
                    servo_position = *(bool *)message->attribute.data.value;

                    // --- Логика управления сервоприводом и NVS теперь здесь ---
                    ESP_LOGI(TAG, "Servo position command: %s", servo_position ? "Open" : "Close");

                    // Сохраняем новое состояние в NVS
                    nvs_write_servo_position(servo_position);

                    // Проверяем, что хэндл задачи был получен при создании
                    if (servo_task_handle != NULL)
                    {
                        // Отправляем уведомление задаче сервопривода
                        BaseType_t notify_result = xTaskNotify(servo_task_handle,
                                                               (uint32_t)servo_position,
                                                               eSetValueWithOverwrite);
                        if (notify_result == pdPASS)
                        {
                            ESP_LOGI(TAG, "Servo task notified.");
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Failed to notify servo task (pdFAIL)");
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Cannot notify servo task: handle is NULL!");
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Received On/Off attribute update with NULL data value.");
                }
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    /* Enable zigbee light sleep */
    esp_zb_sleep_enable(true); // Включаем возможность сна сразу
    esp_zb_init(&zb_nwk_cfg);

    /* Create Basic cluster (Server) */
    esp_zb_attribute_list_t *basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint8_t power_source = ZB_POWER_SOURCE_BATTERY;
    uint8_t manufacturer_name[] = ESP_MANUFACTURER_NAME;
    uint8_t model_identifier[] = ESP_MODEL_IDENTIFIER;
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_identifier);

    /* Create Identify cluster (Server) */
    esp_zb_attribute_list_t *identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);

    /* Create On/Off cluster (Client - for switch) */
    // esp_zb_attribute_list_t *on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    /* No mandatory attributes for On/Off client according to ZCL spec */

    /* Create On/Off cluster (Server - for state/control) */
    esp_zb_attribute_list_t *on_off_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    bool on_off_state = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_on_off_cluster_add_attr(on_off_server_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off_state);

    /* Create cluster list and add clusters */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    // ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, on_off_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Create endpoint list and add endpoint */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, // Still an On/Off Switch device type
        .app_device_version = 0};
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config));

    /* Register the device */
    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    /* load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    // // Инициализируем сервопривод перед использованием
    // ESP_ERROR_CHECK(servo_init());
    
    // Инициализируем модуль калибровки сервопривода
    ESP_ERROR_CHECK(servo_calibration_init());

    // Читаем сохраненное положение сервопривода из NVS в ЛОКАЛЬНУЮ переменную
    bool local_initial_servo_pos = false;              // Локальная переменная
    nvs_read_servo_position(&local_initial_servo_pos); // Читаем в локальную переменную

    /* Не инициализируем сервопривод при старте - он будет инициализирован при использовании */
    /* Создаем задачу управления сервоприводом */
    BaseType_t task_created = xTaskCreate(servo_control_task,
                                          "servo_control_task",
                                          4096,                // Stack size
                                          NULL,                // pvParameters
                                          5,                   // Priority
                                          &servo_task_handle); // Task handle output

    if (task_created != pdPASS || servo_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create servo control task");
        servo_task_handle = NULL;  // Убедимся, что он NULL в случае ошибки
        ESP_ERROR_CHECK(ESP_FAIL); // Остановит выполнение
    }
    ESP_LOGI(TAG, "Servo control task created.");

    // Отправляем начальное положение задаче сервопривода через уведомление
    ESP_LOGI(TAG, "Sending initial position notification to servo task: %s", local_initial_servo_pos ? "Open" : "Close");
    BaseType_t notify_result = xTaskNotify(servo_task_handle,
                                           (uint32_t)local_initial_servo_pos,
                                           eSetValueWithOverwrite);
    if (notify_result != pdPASS)
    {
        ESP_LOGW(TAG, "Failed to notify servo task for initial position (pdFAIL)");
        // Не фатальная ошибка, но стоит залогировать   
    }

    /* Создаем основную задачу Zigbee, передавая начальное положение как параметр */
    task_created = xTaskCreate(esp_zb_task,
                               "Zigbee_main",
                               4096,
                               (void *)(uintptr_t)local_initial_servo_pos, // Передаем локальное значение
                               5,
                               NULL);
    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Zigbee main task");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(TAG, "Zigbee main task created.");
}
