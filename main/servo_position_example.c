#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo_position_reader.h"

static const char *TAG = "SERVO_EXAMPLE";

void app_main(void)
{
    // Инициализация модуля чтения положения сервопривода
    servo_position_reader_config_t config = {
        .angle_min = 0.0f,        // Минимальный угол сервопривода (0 градусов)
        .angle_max = 180.0f,      // Максимальный угол сервопривода (180 градусов)
        .voltage_min = 0.0f,      // Напряжение при минимальном угле (мВ)
        .voltage_max = 4200.0f,   // Напряжение при максимальном угле (мВ)
        
        // Настройка делителя напряжения
        // Например, для R1=10 кОм, R2=15 кОм: (10 + 15) / 15 = 1.67
        .use_voltage_divider = true,
        .voltage_divider_factor = 1.67f,  // Для делителя напряжения 10кОм/15кОм
        
        // Настройка пина активации считывания
        .use_enable_pin = true,           // Использовать пин для активации схемы считывания
        .enable_pin = SERVO_POSITION_ENABLE_PIN, // GPIO4 по умолчанию
        .enable_level = true              // Активный уровень HIGH
    };
    
    esp_err_t ret = servo_position_reader_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации модуля чтения положения: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Модуль чтения положения инициализирован успешно");
    ESP_LOGI(TAG, "Делитель напряжения настроен: %s (коэффициент: %.2f)", 
             config.use_voltage_divider ? "ДА" : "НЕТ", 
             config.voltage_divider_factor);
    ESP_LOGI(TAG, "Пин активации: %s (GPIO%d, активный уровень: %s)",
             config.use_enable_pin ? "ДА" : "НЕТ",
             config.enable_pin,
             config.enable_level ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "Начинаем мониторинг положения сервопривода...");
    
    // Основной цикл чтения и отображения положения сервопривода
    while (1) {
        // Перед чтением активируем схему считывания
        ret = servo_position_set_reading_enabled(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка активации схемы считывания: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Схема считывания активирована");
        }
        
        // Чтение сырого значения ADC
        int raw_value;
        ret = servo_position_reader_get_raw(&raw_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка чтения сырого значения: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Сырое значение ADC: %d", raw_value);
        }
        
        // Чтение напряжения в милливольтах
        float voltage_mv;
        ret = servo_position_reader_get_voltage_mv(&voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка чтения напряжения: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Напряжение: %.2f мВ (%.2f В)", voltage_mv, voltage_mv/1000.0f);
        }
        
        // Чтение угла сервопривода
        float angle;
        ret = servo_position_reader_get_angle(&angle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка чтения угла: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Угол сервопривода: %.2f градусов", angle);
        }
        
        // После завершения измерений деактивируем схему считывания для экономии энергии
        ret = servo_position_set_reading_enabled(false);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка деактивации схемы считывания: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Схема считывания деактивирована");
        }
        
        // Ждем некоторое время перед следующим измерением
        ESP_LOGI(TAG, "-----------------------------------");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Обновление каждые 2 секунды
    }
    
    // Код ниже не будет достигнут в бесконечном цикле,
    // но добавлен для демонстрации правильного освобождения ресурсов
    servo_position_reader_deinit();
} 