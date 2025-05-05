#include "servo_position_reader.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO_POS";

// Глобальные переменные для хранения настроек
static servo_position_reader_config_t reader_config;
static bool is_initialized = false;
static bool reading_enabled = false;

esp_err_t servo_position_reader_init(servo_position_reader_config_t *config) {
    ESP_LOGI(TAG, "Инициализация модуля чтения положения сервопривода");
    
    if (is_initialized) {
        ESP_LOGW(TAG, "Модуль уже инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
     
    if (config == NULL) {
        ESP_LOGE(TAG, "Конфигурация не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Инициализация АЦП
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = SERVO_POSITION_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &reader_config.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Настройка канала АЦП
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12, // 12-битное разрешение
        .atten = ADC_ATTEN_DB_12,    // Аттенюация 12dB для измерения до 3.3В
    };
    
    ret = adc_oneshot_config_channel(reader_config.adc_handle, SERVO_POSITION_ADC_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка настройки канала ADC: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(reader_config.adc_handle);
        return ret;
    }
    
    // Копируем настройки калибровки
    reader_config.angle_min = config->angle_min;
    reader_config.angle_max = config->angle_max;
    reader_config.voltage_min = config->voltage_min;
    reader_config.voltage_max = config->voltage_max;
    
    // Настройки делителя напряжения
    reader_config.use_voltage_divider = config->use_voltage_divider;
    if (reader_config.use_voltage_divider) {
        reader_config.voltage_divider_factor = config->voltage_divider_factor;
        ESP_LOGI(TAG, "Используется делитель напряжения с коэффициентом: %.2f", 
                 reader_config.voltage_divider_factor);
    } else {
        reader_config.voltage_divider_factor = 1.0f; // Без делителя
    }
    
    // Настройки пина активации
    reader_config.use_enable_pin = config->use_enable_pin;
    if (reader_config.use_enable_pin) {
        reader_config.enable_pin = (config->enable_pin != 0) ? config->enable_pin : SERVO_POSITION_ENABLE_PIN;
        reader_config.enable_level = config->enable_level;
        
        // Настраиваем GPIO для активации считывания
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << reader_config.enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка настройки пина активации: %s", esp_err_to_name(ret));
            adc_oneshot_del_unit(reader_config.adc_handle);
            return ret;
        }
        
        // Деактивируем считывание положения по умолчанию
        gpio_set_level(reader_config.enable_pin, !reader_config.enable_level);
        
        ESP_LOGI(TAG, "Настроен пин активации %d, уровень активации: %s", 
                 reader_config.enable_pin, reader_config.enable_level ? "HIGH" : "LOW");
    }
    
    ESP_LOGI(TAG, "Модуль успешно инициализирован. Рабочий диапазон: %.1f-%.1f градусов", 
             reader_config.angle_min, reader_config.angle_max);
    
    is_initialized = true;
    reading_enabled = false;
    return ESP_OK;
}

esp_err_t servo_position_set_reading_enabled(bool enabled) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Если текущее состояние уже соответствует запрашиваемому, ничего не делаем
    if (reading_enabled == enabled) {
        ESP_LOGD(TAG, "Считывание уже %s", enabled ? "активировано" : "деактивировано");
        return ESP_OK;
    }
    
    if (reader_config.use_enable_pin) {
        // Устанавливаем соответствующий уровень для включения/выключения считывания
        gpio_set_level(reader_config.enable_pin, enabled ? reader_config.enable_level : !reader_config.enable_level);
        ESP_LOGD(TAG, "%s пин считывания положения сервопривода", 
                enabled ? "Активирован" : "Деактивирован");
        
        // Даем схеме считывания время на стабилизацию при включении
        if (enabled) {
            vTaskDelay(pdMS_TO_TICKS(POSITION_READING_STABILIZATION_TIME_MS));
        }
    }
    
    reading_enabled = enabled;
    return ESP_OK;
}

esp_err_t servo_position_reader_get_raw(int *raw_value) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (raw_value == NULL) {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Если используется пин активации и считывание не активировано,
    // активируем схему перед измерением
    bool was_enabled = reading_enabled;
    if (reader_config.use_enable_pin && !was_enabled) {
        esp_err_t ret = servo_position_set_reading_enabled(true);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    esp_err_t ret = adc_oneshot_read(reader_config.adc_handle, SERVO_POSITION_ADC_CHANNEL, raw_value);
    
    // Если схема была активирована автоматически, деактивируем ее
    if (reader_config.use_enable_pin && !was_enabled) {
        servo_position_set_reading_enabled(false);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t servo_position_reader_get_voltage_mv(float *voltage_mv) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (voltage_mv == NULL) {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    int raw_value;
    esp_err_t ret = servo_position_reader_get_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Преобразуем сырое значение в милливольты (с учетом аттенюации 12dB)
    float adc_voltage = ((float)raw_value * 3300.0f) / ADC_MAX_VALUE;
    
    // Применяем коэффициент делителя, если он используется
    if (reader_config.use_voltage_divider) {
        *voltage_mv = adc_voltage * reader_config.voltage_divider_factor;
    } else {
        *voltage_mv = adc_voltage;
    }
    
    ESP_LOGD(TAG, "Напряжение: %.2f мВ (raw: %d, adc_voltage: %.2f мВ)", 
             *voltage_mv, raw_value, adc_voltage);
    
    return ESP_OK;
}

esp_err_t servo_position_reader_get_angle(float *angle) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (angle == NULL) {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    float voltage_mv;
    esp_err_t ret = servo_position_reader_get_voltage_mv(&voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Линейное преобразование напряжения в угол
    float voltage_range = reader_config.voltage_max - reader_config.voltage_min;
    float angle_range = reader_config.angle_max - reader_config.angle_min;
    
    if (voltage_range <= 0 || angle_range <= 0) {
        ESP_LOGE(TAG, "Некорректный диапазон калибровки");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Ограничение по минимальному напряжению
    if (voltage_mv < reader_config.voltage_min) {
        voltage_mv = reader_config.voltage_min;
    }
    
    // Ограничение по максимальному напряжению
    if (voltage_mv > reader_config.voltage_max) {
        voltage_mv = reader_config.voltage_max;
    }
    
    // Линейное преобразование: угол = угол_мин + (напряжение - напряжение_мин) * (угол_макс - угол_мин) / (напряжение_макс - напряжение_мин)
    *angle = reader_config.angle_min + 
             (voltage_mv - reader_config.voltage_min) * angle_range / voltage_range;
    
    ESP_LOGD(TAG, "Угол: %.2f град. (напряжение: %.2f мВ)", *angle, voltage_mv);
    
    return ESP_OK;
}

esp_err_t servo_position_reader_deinit(void) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "Модуль не был инициализирован");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Если считывание активировано, деактивируем его
    if (reading_enabled) {
        servo_position_set_reading_enabled(false);
    }
    
    // Если используется пин активации, сбрасываем его настройки
    if (reader_config.use_enable_pin) {
        gpio_reset_pin(reader_config.enable_pin);
    }
    
    esp_err_t ret = adc_oneshot_del_unit(reader_config.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка освобождения ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    is_initialized = false;
    reading_enabled = false;
    ESP_LOGI(TAG, "Модуль деинициализирован");
    
    return ESP_OK;
} 