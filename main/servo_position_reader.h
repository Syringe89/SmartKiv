#ifndef SERVO_POSITION_READER_H
#define SERVO_POSITION_READER_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

// Константы для настройки АЦП
#define SERVO_POSITION_ADC_UNIT     ADC_UNIT_1  // Используем ADC1, так как ADC2 может конфликтовать с WiFi
#define SERVO_POSITION_ADC_CHANNEL  ADC_CHANNEL_4 // Настройте на нужный канал
#define SERVO_POSITION_GPIO_PIN     5  // Настройте на нужный пин, который может работать как ADC

// GPIO для активации схемы считывания положения сервопривода
#define SERVO_POSITION_ENABLE_PIN   12  // Пин для активации считывания (при необходимости измените)

// Максимальное напряжение сервопривода (4.2В)
#define SERVO_MAX_VOLTAGE_MV        4200
// Максимальное значение АЦП при используемом разрешении (12 бит = 4095)
#define ADC_MAX_VALUE               4095

// Время стабилизации схемы считывания в миллисекундах
#define POSITION_READING_STABILIZATION_TIME_MS 10

// Структура для хранения настроек модуля чтения положения
typedef struct {
    adc_oneshot_unit_handle_t adc_handle;  // Хэндл АЦП в режиме oneshot
    float angle_min;                       // Минимальный угол сервопривода (градусы)
    float angle_max;                       // Максимальный угол сервопривода (градусы)
    float voltage_min;                     // Напряжение при минимальном угле (мВ)
    float voltage_max;                     // Напряжение при максимальном угле (мВ)
    
    // Коэффициенты делителя напряжения (если используется)
    // Формула для расчета коэффициента: voltage_divider_factor = (R1 + R2) / R2
    float voltage_divider_factor;         // Коэффициент делителя напряжения (по умолчанию 1.0 - без делителя)
    bool use_voltage_divider;             // Флаг использования делителя напряжения

    // Настройки для GPIO активации
    bool use_enable_pin;                  // Использовать пин активации считывания
    uint8_t enable_pin;                   // Номер пина для активации считывания
    bool enable_level;                    // Логический уровень активации (true = HIGH, false = LOW)
} servo_position_reader_config_t;

// Инициализация модуля чтения положения сервопривода
esp_err_t servo_position_reader_init(servo_position_reader_config_t *config);

// Включение/отключение схемы считывания положения
esp_err_t servo_position_set_reading_enabled(bool enabled);

// Чтение текущего положения сервопривода в градусах
esp_err_t servo_position_reader_get_angle(float *angle);

// Чтение текущего напряжения с сервопривода
esp_err_t servo_position_reader_get_voltage_mv(float *voltage_mv);

// Чтение сырого значения АЦП
esp_err_t servo_position_reader_get_raw(int *raw_value);

// Деинициализация модуля
esp_err_t servo_position_reader_deinit(void);

#endif // SERVO_POSITION_READER_H 