#include "servo_calibration.h"
#include "servo_control.h"
#include "servo_position_reader.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "SERVO_CALIB";

// Вспомогательная функция для чтения среднего напряжения в текущей позиции
static esp_err_t read_average_voltage(int samples, float *avg_voltage) {
    float sum = 0.0f;
    float voltage;
    esp_err_t ret;
    
    for (int i = 0; i < samples; i++) {
        ret = servo_position_reader_get_voltage_mv(&voltage);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка чтения напряжения: %s", esp_err_to_name(ret));
            return ret;
        }
        sum += voltage;
        vTaskDelay(pdMS_TO_TICKS(10)); // Небольшая задержка между измерениями
    }
    
    *avg_voltage = sum / samples;
    return ESP_OK;
}

// Функция для определения стабильности значений напряжения
static bool is_voltage_stable(float prev_voltage, float curr_voltage, float threshold) {
    return fabsf(curr_voltage - prev_voltage) < threshold;
}

// Инициализация настроек калибровки по умолчанию
esp_err_t servo_calibration_init_config(servo_calibration_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config->step_size_deg = 2.0f;         // Шаг 2 градуса
    config->step_delay_ms = 200.0f;       // Задержка 200 мс между шагами
    config->threshold_voltage_mv = 10.0f; // Порог изменения 10 мВ
    config->samples_per_position = 5;     // 5 измерений для усреднения
    config->timeout_sec = 60.0f;          // Таймаут 60 секунд
    
    return ESP_OK;
}

// Определение границ рабочего диапазона сервопривода
esp_err_t servo_calibration_find_limits(const servo_calibration_config_t *config, 
                                       float *min_angle, float *min_voltage,
                                       float *max_angle, float *max_voltage) {
    if (config == NULL || min_angle == NULL || min_voltage == NULL || 
        max_angle == NULL || max_voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    float prev_voltage, curr_voltage;
    int stable_count = 0;
    const int STABILITY_THRESHOLD = 3; // Сколько измерений должно быть стабильным для определения края
    bool found_limit = false;
    
    // Активация считывания положения
    ret = servo_position_set_reading_enabled(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось активировать считывание положения: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 1. Поиск минимального угла (нижней границы)
    ESP_LOGI(TAG, "Калибровка - поиск минимального угла...");
    
    // Начинаем с середины диапазона
    float current_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    
    // Устанавливаем сервопривод в среднее положение и даем ему время на перемещение
    ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, servo_calculate_duty(current_angle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки положения сервопривода: %s", esp_err_to_name(ret));
        servo_position_set_reading_enabled(false);
        return ret;
    }
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка обновления положения сервопривода: %s", esp_err_to_name(ret));
        servo_position_set_reading_enabled(false);
        return ret;
    }
    
    // Задержка для перемещения сервопривода
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Получаем начальное значение напряжения
    ret = read_average_voltage(config->samples_per_position, &prev_voltage);
    if (ret != ESP_OK) {
        servo_position_set_reading_enabled(false);
        return ret;
    }
    
    // Устанавливаем таймаут калибровки
    TickType_t timeout_ticks = pdMS_TO_TICKS(config->timeout_sec * 1000);
    TickType_t start_time = xTaskGetTickCount();
    
    // Двигаемся в сторону уменьшения угла
    stable_count = 0;
    found_limit = false;
    
    while (current_angle > SERVO_MIN_ANGLE && !found_limit) {
        // Проверка таймаута
        if (xTaskGetTickCount() - start_time > timeout_ticks) {
            ESP_LOGE(TAG, "Таймаут калибровки при поиске минимального угла");
            servo_position_set_reading_enabled(false);
            return ESP_ERR_TIMEOUT;
        }
        
        // Уменьшаем угол на шаг
        current_angle -= config->step_size_deg;
        if (current_angle < SERVO_MIN_ANGLE) current_angle = SERVO_MIN_ANGLE;
        
        // Устанавливаем новое положение сервопривода
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, servo_calculate_duty(current_angle));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка установки положения сервопривода: %s", esp_err_to_name(ret));
            servo_position_set_reading_enabled(false);
            return ret;
        }
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка обновления положения сервопривода: %s", esp_err_to_name(ret));
            servo_position_set_reading_enabled(false);
            return ret;
        }
        
        // Задержка для стабилизации
        vTaskDelay(pdMS_TO_TICKS(config->step_delay_ms));
        
        // Измеряем новое напряжение
        ret = read_average_voltage(config->samples_per_position, &curr_voltage);
        if (ret != ESP_OK) {
            servo_position_set_reading_enabled(false);
            return ret;
        }
        
        ESP_LOGI(TAG, "Угол: %.2f, Напряжение: %.2f мВ", current_angle, curr_voltage);
        
        // Проверяем, стабилизировалось ли напряжение (означает достижение предела)
        if (is_voltage_stable(prev_voltage, curr_voltage, config->threshold_voltage_mv)) {
            stable_count++;
            if (stable_count >= STABILITY_THRESHOLD) {
                found_limit = true;
                *min_angle = current_angle;
                *min_voltage = curr_voltage;
                ESP_LOGI(TAG, "Найден минимальный угол: %.2f, напряжение: %.2f мВ", *min_angle, *min_voltage);
            }
        } else {
            stable_count = 0;
        }
        
        prev_voltage = curr_voltage;
    }
    
    if (!found_limit) {
        // Если не нашли ограничение, используем минимальное значение
        *min_angle = SERVO_MIN_ANGLE;
        *min_voltage = curr_voltage;
        ESP_LOGW(TAG, "Минимальный угол не найден, используем SERVO_MIN_ANGLE: %d", SERVO_MIN_ANGLE);
    }
    
    // 2. Поиск максимального угла (верхней границы)
    ESP_LOGI(TAG, "Калибровка - поиск максимального угла...");
    
    // Возвращаемся в середину диапазона
    current_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    
    // Устанавливаем сервопривод в среднее положение и даем ему время на перемещение
    ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, servo_calculate_duty(current_angle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки положения сервопривода: %s", esp_err_to_name(ret));
        servo_position_set_reading_enabled(false);
        return ret;
    }
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка обновления положения сервопривода: %s", esp_err_to_name(ret));
        servo_position_set_reading_enabled(false);
        return ret;
    }
    
    // Задержка для перемещения сервопривода
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Получаем начальное значение напряжения
    ret = read_average_voltage(config->samples_per_position, &prev_voltage);
    if (ret != ESP_OK) {
        servo_position_set_reading_enabled(false);
        return ret;
    }
    
    // Сбрасываем таймер
    start_time = xTaskGetTickCount();
    
    // Двигаемся в сторону увеличения угла
    stable_count = 0;
    found_limit = false;
    
    while (current_angle < SERVO_MAX_ANGLE && !found_limit) {
        // Проверка таймаута
        if (xTaskGetTickCount() - start_time > timeout_ticks) {
            ESP_LOGE(TAG, "Таймаут калибровки при поиске максимального угла");
            servo_position_set_reading_enabled(false);
            return ESP_ERR_TIMEOUT;
        }
        
        // Увеличиваем угол на шаг
        current_angle += config->step_size_deg;
        if (current_angle > SERVO_MAX_ANGLE) current_angle = SERVO_MAX_ANGLE;
        
        // Устанавливаем новое положение сервопривода
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, servo_calculate_duty(current_angle));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка установки положения сервопривода: %s", esp_err_to_name(ret));
            servo_position_set_reading_enabled(false);
            return ret;
        }
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка обновления положения сервопривода: %s", esp_err_to_name(ret));
            servo_position_set_reading_enabled(false);
            return ret;
        }
        
        // Задержка для стабилизации
        vTaskDelay(pdMS_TO_TICKS(config->step_delay_ms));
        
        // Измеряем новое напряжение
        ret = read_average_voltage(config->samples_per_position, &curr_voltage);
        if (ret != ESP_OK) {
            servo_position_set_reading_enabled(false);
            return ret;
        }
        
        ESP_LOGI(TAG, "Угол: %.2f, Напряжение: %.2f мВ", current_angle, curr_voltage);
        
        // Проверяем, стабилизировалось ли напряжение (означает достижение предела)
        if (is_voltage_stable(prev_voltage, curr_voltage, config->threshold_voltage_mv)) {
            stable_count++;
            if (stable_count >= STABILITY_THRESHOLD) {
                found_limit = true;
                *max_angle = current_angle;
                *max_voltage = curr_voltage;
                ESP_LOGI(TAG, "Найден максимальный угол: %.2f, напряжение: %.2f мВ", *max_angle, *max_voltage);
            }
        } else {
            stable_count = 0;
        }
        
        prev_voltage = curr_voltage;
    }
    
    if (!found_limit) {
        // Если не нашли ограничение, используем максимальное значение
        *max_angle = SERVO_MAX_ANGLE;
        *max_voltage = curr_voltage;
        ESP_LOGW(TAG, "Максимальный угол не найден, используем SERVO_MAX_ANGLE: %d", SERVO_MAX_ANGLE);
    }
    
    // Деактивация считывания положения
    servo_position_set_reading_enabled(false);
    
    // Возвращаем сервопривод в нейтральное положение
    current_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, servo_calculate_duty(current_angle));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки положения сервопривода: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка обновления положения сервопривода: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// Запуск процедуры автоматической калибровки
esp_err_t servo_calibration_run(servo_calibration_config_t *config, servo_calibration_result_t *result) {
    if (config == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Запуск автоматической калибровки сервопривода");
    
    // Инициализируем результат калибровки
    result->calibration_valid = false;
    
    esp_err_t ret = servo_calibration_find_limits(config, 
                                                &result->min_angle, &result->min_voltage_mv,
                                                &result->max_angle, &result->max_voltage_mv);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при поиске пределов сервопривода: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Проверка корректности результатов калибровки
    if (result->min_angle >= result->max_angle ||
        fabsf(result->min_voltage_mv - result->max_voltage_mv) < config->threshold_voltage_mv * 10.0f) {
        ESP_LOGE(TAG, "Некорректные результаты калибровки: min_angle=%.2f, max_angle=%.2f, min_voltage=%.2f, max_voltage=%.2f",
                result->min_angle, result->max_angle, result->min_voltage_mv, result->max_voltage_mv);
        return ESP_ERR_INVALID_STATE;
    }
    
    result->calibration_valid = true;
    ESP_LOGI(TAG, "Калибровка успешно завершена. Рабочий диапазон: %.2f-%.2f градусов, %.2f-%.2f мВ",
            result->min_angle, result->max_angle, result->min_voltage_mv, result->max_voltage_mv);
    
    return ESP_OK;
}

// Применение результатов калибровки
esp_err_t servo_calibration_apply_results(const servo_calibration_result_t *result) {
    if (result == NULL || !result->calibration_valid) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Обновляем параметры модуля чтения положения сервопривода
    esp_err_t ret = servo_position_reader_set_params(
        result->min_angle, 
        result->max_angle, 
        result->min_voltage_mv, 
        result->max_voltage_mv
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка обновления параметров модуля чтения положения: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Результаты калибровки применены");
    return ESP_OK;
} 