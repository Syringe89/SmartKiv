#include "servo_calibration.h"
#include "servo_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "SERVO_CALIB";

// Структура для хранения конфигурации модуля
typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t adc_cali_handle;
    bool adc_calibrated;
    float shunt_resistance;   // Сопротивление шунта в Омах
    QueueHandle_t button_evt_queue;
} servo_calibration_config_t;

// Глобальные переменные
static servo_calibration_config_t config;
static bool is_initialized = false;
static servo_calibration_data_t calib_data = {
    .is_calibrated = false,
    .min_angle = SERVO_MIN_ANGLE,
    .max_angle = SERVO_MAX_ANGLE,
    .normal_current = 50.0f,  // начальное значение тока в мА
    .stall_current = 200.0f   // начальное значение тока при блокировке в мА
};

// Пересчитать скважность PWM для заданного угла
static uint32_t angle_to_duty(float angle) {
    float duty_float = (((angle - SERVO_MIN_ANGLE) * (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US)) / 
                        (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) + SERVO_MIN_WIDTH_US;
    
    // Преобразование микросекунд в значение duty
    uint32_t duty = (uint32_t)((duty_float * LEDC_MAX_DUTY) / (1000000.0f / SERVO_FREQ));
    return duty;
}

// Функция для сохранения калибровочных данных
esp_err_t servo_calibration_save_data(const servo_calibration_data_t *calibration_data) {
    if (calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Сохраняем данные калибровки
    err = nvs_set_float(nvs_handle, "min_angle", calibration_data->min_angle);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_float(nvs_handle, "max_angle", calibration_data->max_angle);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_u32(nvs_handle, "min_duty", calibration_data->min_duty);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_u32(nvs_handle, "max_duty", calibration_data->max_duty);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_float(nvs_handle, "norm_current", calibration_data->normal_current);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_float(nvs_handle, "stall_current", calibration_data->stall_current);
    if (err != ESP_OK) goto cleanup;
    
    err = nvs_set_u8(nvs_handle, "is_calib", calibration_data->is_calibrated ? 1 : 0);
    if (err != ESP_OK) goto cleanup;
    
    // Записываем данные в NVS
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка сохранения данных в NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Данные калибровки успешно сохранены");
    }

cleanup:
    nvs_close(nvs_handle);
    return err;
}

// Функция для загрузки калибровочных данных
esp_err_t servo_calibration_load_data(servo_calibration_data_t *calibration_data) {
    if (calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    // Проверяем наличие данных калибровки
    uint8_t is_calib;
    err = nvs_get_u8(nvs_handle, "is_calib", &is_calib);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Данные калибровки не найдены");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    calibration_data->is_calibrated = (is_calib == 1);
    
    // Загружаем данные если есть калибровка
    if (calibration_data->is_calibrated) {
        err = nvs_get_float(nvs_handle, "min_angle", &calibration_data->min_angle);
        if (err != ESP_OK) goto cleanup;
        
        err = nvs_get_float(nvs_handle, "max_angle", &calibration_data->max_angle);
        if (err != ESP_OK) goto cleanup;
        
        err = nvs_get_u32(nvs_handle, "min_duty", &calibration_data->min_duty);
        if (err != ESP_OK) goto cleanup;
        
        err = nvs_get_u32(nvs_handle, "max_duty", &calibration_data->max_duty);
        if (err != ESP_OK) goto cleanup;
        
        err = nvs_get_float(nvs_handle, "norm_current", &calibration_data->normal_current);
        if (err != ESP_OK) goto cleanup;
        
        err = nvs_get_float(nvs_handle, "stall_current", &calibration_data->stall_current);
        if (err != ESP_OK) goto cleanup;
        
        ESP_LOGI(TAG, "Данные калибровки загружены: min=%0.1f°, max=%0.1f°, ток=%0.1f/%0.1f мА", 
                 calibration_data->min_angle, calibration_data->max_angle, 
                 calibration_data->normal_current, calibration_data->stall_current);
    }
    
cleanup:
    nvs_close(nvs_handle);
    return err;
}

// Инициализация модуля калибровки
esp_err_t servo_calibration_init(void) {
    if (is_initialized) {
        ESP_LOGW(TAG, "Модуль уже инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Инициализация модуля калибровки сервопривода");
    
    // Инициализируем структуру конфигурации
    config.adc_calibrated = false;
    config.shunt_resistance = 2.2f; // Шунт 2.2 Ома, можно изменить
    
    // Загружаем сохраненные данные калибровки
    esp_err_t ret = servo_calibration_load_data(&calib_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Данные калибровки не найдены, будут использованы значения по умолчанию");
        calib_data.is_calibrated = false;
    }
    
    // Создаем очередь сообщений для кнопки
    config.button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (config.button_evt_queue == NULL) {
        ESP_LOGE(TAG, "Не удалось создать очередь для кнопки");
        return ESP_ERR_NO_MEM;
    }
    
    // Запускаем задачу мониторинга кнопки
    BaseType_t xReturned = xTaskCreate(
        servo_calibration_button_task,
        "servo_calib_btn",
        4096,
        NULL,
        5,
        NULL
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Не удалось создать задачу мониторинга кнопки");
        vQueueDelete(config.button_evt_queue);
        return ESP_ERR_NO_MEM;
    }
    
    is_initialized = true;
    ESP_LOGI(TAG, "Модуль калибровки сервопривода инициализирован");
    
    return ESP_OK;
}

// Инициализация АЦП для измерения тока
static esp_err_t init_adc(void) {
    ESP_LOGI(TAG, "Инициализация АЦП для измерения тока");
    
    // Инициализация АЦП для измерения тока
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = SERVO_CURRENT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &config.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Настройка канала АЦП
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = SERVO_CURRENT_ADC_ATTEN,
    };
    
    ret = adc_oneshot_config_channel(config.adc_handle, SERVO_CURRENT_ADC_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка настройки канала ADC: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(config.adc_handle);
        return ret;
    }
    
    // Инициализация калибровки АЦП
    adc_cali_handle_t adc_cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = SERVO_CURRENT_ADC_UNIT,
        .chan = SERVO_CURRENT_ADC_CHANNEL,
        .bitwidth = ADC_BITWIDTH_12,
        .atten = SERVO_CURRENT_ADC_ATTEN,
    };
    
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        config.adc_cali_handle = adc_cali_handle;
        config.adc_calibrated = true;
        ESP_LOGI(TAG, "Калибровка АЦП успешно инициализирована");
    } else {
        ESP_LOGW(TAG, "Калибровка АЦП не поддерживается, будут использованы сырые значения");
        config.adc_cali_handle = NULL;
    }
    
    return ESP_OK;
}

// Деинициализация АЦП
static esp_err_t deinit_adc(void) {
    ESP_LOGI(TAG, "Деинициализация АЦП");
    
    if (config.adc_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(config.adc_cali_handle);
        config.adc_cali_handle = NULL;
    }
    
    esp_err_t ret = adc_oneshot_del_unit(config.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка деинициализации ADC: %s", esp_err_to_name(ret));
    }
    
    config.adc_calibrated = false;
    
    return ret;
}

// Измерение тока через шунт
esp_err_t servo_calibration_get_current(float *current_ma, bool auto_deinit) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (current_ma == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Проверяем, инициализирован ли АЦП, и если нет - инициализируем его
    if (!config.adc_calibrated && config.adc_cali_handle == NULL) {
        ESP_LOGI(TAG, "АЦП не инициализирован, инициализируем его");
        esp_err_t ret = init_adc();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка инициализации АЦП: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(config.adc_handle, SERVO_CURRENT_ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Вычисляем напряжение на шунте
    float voltage_mv;
    if (config.adc_calibrated) {
        int voltage;
        ret = adc_cali_raw_to_voltage(config.adc_cali_handle, adc_raw, &voltage);
        if (ret == ESP_OK) {
            voltage_mv = (float)voltage;
        } else {
            voltage_mv = ((float)adc_raw * 3300.0f) / 4095.0f;
        }
    } else {
        voltage_mv = ((float)adc_raw * 3300.0f) / 4095.0f;
    }
    
    // Вычисляем ток через шунт (I = U/R)
    *current_ma = voltage_mv / config.shunt_resistance;
    
    ESP_LOGD(TAG, "Измерен ток: %.2f мА (ADC: %d, напряжение: %.2f мВ)", 
             *current_ma, adc_raw, voltage_mv);
    
    // Если запрошена автоматическая деинициализация АЦП
    if (auto_deinit) {
        ESP_LOGD(TAG, "Автоматическая деинициализация АЦП после измерения");
        deinit_adc();
    }
    
    return ESP_OK;
}

// Измерение среднего тока из нескольких измерений
static esp_err_t get_average_current(float *avg_current, int samples) {
    if (avg_current == NULL || samples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float sum = 0.0f;
    float current;
    esp_err_t ret;
    
    for (int i = 0; i < samples; i++) {
        // Не деинициализируем АЦП между измерениями для экономии времени
        ret = servo_calibration_get_current(&current, false);
        if (ret != ESP_OK) {
            return ret;
        }
        sum += current;
        vTaskDelay(pdMS_TO_TICKS(10)); // небольшая задержка между измерениями
    }
    
    *avg_current = sum / samples;
    return ESP_OK;
}

// Основная функция калибровки сервопривода
esp_err_t servo_calibration_start(servo_calibration_data_t *calibration_data) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Начало калибровки сервопривода");
    
    // Инициализируем АЦП для измерения тока
    esp_err_t ret = init_adc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации АЦП для калибровки: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Устанавливаем сервопривод в среднее положение
    float center_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    uint32_t center_duty = angle_to_duty(center_angle);
    
    ESP_LOGI(TAG, "Установка в среднее положение: %.1f градусов", center_angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, center_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    // Даем время на стабилизацию
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Измеряем базовый ток при нормальном движении
    float normal_current;
    ret = get_average_current(&normal_current, SERVO_CALIBRATION_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка измерения базового тока: %s", esp_err_to_name(ret));
        deinit_adc(); // Деинициализируем АЦП при ошибке
        return ret;
    }
    
    ESP_LOGI(TAG, "Нормальный ток: %.2f мА", normal_current);
    calibration_data->normal_current = normal_current;
    
    // Определяем пороговый ток для обнаружения блокировки
    float threshold_current = normal_current * SERVO_CALIBRATION_THRESHOLD_FACTOR;
    ESP_LOGI(TAG, "Пороговый ток блокировки: %.2f мА", threshold_current);
    
    // Поиск нижней границы (минимальный угол)
    float current_angle = center_angle;
    float min_angle = SERVO_MIN_ANGLE;
    float current_current;
    
    ESP_LOGI(TAG, "Поиск нижней границы...");
    
    while (current_angle > SERVO_MIN_ANGLE) {
        // Уменьшаем угол на шаг
        current_angle -= SERVO_CALIBRATION_STEP_SIZE;
        if (current_angle < SERVO_MIN_ANGLE) {
            current_angle = SERVO_MIN_ANGLE;
        }
        
        // Устанавливаем новое положение
        uint32_t duty = angle_to_duty(current_angle);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        
        // Ждем стабилизации
        vTaskDelay(pdMS_TO_TICKS(SERVO_CALIBRATION_DELAY_MS));
        
        // Измеряем ток
        ret = get_average_current(&current_current, SERVO_CALIBRATION_SAMPLES);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка измерения тока: %s", esp_err_to_name(ret));
            continue;
        }
        
        ESP_LOGD(TAG, "Угол: %.1f°, ток: %.2f мА", current_angle, current_current);
        
        // Если ток превысил пороговое значение, значит мы достигли блокировки
        if (current_current > threshold_current) {
            min_angle = current_angle + SERVO_CALIBRATION_STEP_SIZE; // Возвращаемся на шаг назад
            calibration_data->stall_current = current_current;
            ESP_LOGI(TAG, "Обнаружена нижняя граница при токе %.2f мА: %.1f градусов", 
                    current_current, min_angle);
            break;
        }
    }
    
    // Возвращаемся в центр
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, center_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Поиск верхней границы (максимальный угол)
    current_angle = center_angle;
    float max_angle = SERVO_MAX_ANGLE;
    
    ESP_LOGI(TAG, "Поиск верхней границы...");
    
    while (current_angle < SERVO_MAX_ANGLE) {
        // Увеличиваем угол на шаг
        current_angle += SERVO_CALIBRATION_STEP_SIZE;
        if (current_angle > SERVO_MAX_ANGLE) {
            current_angle = SERVO_MAX_ANGLE;
        }
        
        // Устанавливаем новое положение
        uint32_t duty = angle_to_duty(current_angle);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        
        // Ждем стабилизации
        vTaskDelay(pdMS_TO_TICKS(SERVO_CALIBRATION_DELAY_MS));
        
        // Измеряем ток
        ret = get_average_current(&current_current, SERVO_CALIBRATION_SAMPLES);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка измерения тока: %s", esp_err_to_name(ret));
            continue;
        }
        
        ESP_LOGD(TAG, "Угол: %.1f°, ток: %.2f мА", current_angle, current_current);
        
        // Если ток превысил пороговое значение, значит мы достигли блокировки
        if (current_current > threshold_current) {
            max_angle = current_angle - SERVO_CALIBRATION_STEP_SIZE; // Возвращаемся на шаг назад
            
            // Обновляем значение тока при блокировке (среднее между двумя измерениями)
            calibration_data->stall_current = (calibration_data->stall_current + current_current) / 2.0f;
            
            ESP_LOGI(TAG, "Обнаружена верхняя граница при токе %.2f мА: %.1f градусов", 
                    current_current, max_angle);
            break;
        }
    }
    
    // Сохраняем результаты калибровки
    calibration_data->min_angle = min_angle;
    calibration_data->max_angle = max_angle;
    calibration_data->min_duty = angle_to_duty(min_angle);
    calibration_data->max_duty = angle_to_duty(max_angle);
    calibration_data->is_calibrated = true;
    
    // Возвращаемся в центральное положение между калиброванными границами
    center_angle = (calibration_data->max_angle + calibration_data->min_angle) / 2.0f;
    center_duty = angle_to_duty(center_angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, center_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    ESP_LOGI(TAG, "Калибровка завершена. Диапазон: %.1f-%.1f градусов, ток: норм=%.2f мА, блок=%.2f мА",
             calibration_data->min_angle, calibration_data->max_angle,
             calibration_data->normal_current, calibration_data->stall_current);
    
    // Сохраняем данные в NVS
    ret = servo_calibration_save_data(calibration_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Не удалось сохранить данные калибровки: %s", esp_err_to_name(ret));
    }
    
    // Деинициализируем АЦП, так как он больше не нужен после калибровки
    ESP_LOGI(TAG, "Освобождаем ресурсы АЦП после калибровки");
    deinit_adc();
    
    return ESP_OK;
}

// Задача мониторинга кнопки
void servo_calibration_button_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SERVO_CALIBRATION_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка настройки пина кнопки: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Запущена задача мониторинга кнопки калибровки (GPIO%d)", SERVO_CALIBRATION_BUTTON);
    
    TickType_t button_pressed_time = 0;
    bool button_pressed = false;
    
    while (1) {
        int button_value = gpio_get_level(SERVO_CALIBRATION_BUTTON);
        
        // Инверсная логика (кнопка подтянута к VCC через подтягивающий резистор)
        bool is_pressed = (button_value == 0);
        
        if (is_pressed && !button_pressed) {
            // Кнопка только что нажата
            button_pressed = true;
            button_pressed_time = xTaskGetTickCount();
            ESP_LOGD(TAG, "Кнопка нажата");
        } else if (!is_pressed && button_pressed) {
            // Кнопка отпущена
            button_pressed = false;
            TickType_t pressed_duration = xTaskGetTickCount() - button_pressed_time;
            ESP_LOGD(TAG, "Кнопка отпущена, длительность: %d мс", 
                     pressed_duration * portTICK_PERIOD_MS);
            
            // Проверяем длительное нажатие
            if (pressed_duration * portTICK_PERIOD_MS >= SERVO_CALIBRATION_BUTTON_PRESS_DURATION) {
                ESP_LOGI(TAG, "Обнаружено длительное нажатие кнопки (%d мс), запуск калибровки",
                         pressed_duration * portTICK_PERIOD_MS);
                
                // Запускаем калибровку
                servo_calibration_start(&calib_data);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Опрос с периодом 50 мс
    }
}

// Деинициализация модуля
esp_err_t servo_calibration_deinit(void) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Деинициализируем АЦП, если он был инициализирован
    if (config.adc_cali_handle != NULL || config.adc_calibrated) {
        deinit_adc();
    }
    
    // Очищаем очередь
    if (config.button_evt_queue != NULL) {
        vQueueDelete(config.button_evt_queue);
        config.button_evt_queue = NULL;
    }
    
    is_initialized = false;
    ESP_LOGI(TAG, "Модуль калибровки деинициализирован");
    
    return ESP_OK;
} 