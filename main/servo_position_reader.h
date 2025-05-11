#ifndef SERVO_POSITION_READER_H
#define SERVO_POSITION_READER_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Настройки АЦП по умолчанию
#define SERVO_POSITION_ADC_UNIT ADC_UNIT_1
#define SERVO_POSITION_ADC_CHANNEL ADC_CHANNEL_2
#define SERVO_POSITION_ENABLE_PIN GPIO_NUM_12
#define ADC_RESOLUTION ADC_BITWIDTH_12          // 12-bit ADC
#define ADC_MAX_VALUE (1 << ADC_RESOLUTION) - 1 // 12-bit ADC VALUE
#define ADC_ATTEN ADC_ATTEN_DB_12               // Аттенюация 12dB для измерения до 3.3В
#define POSITION_READING_STABILIZATION_TIME_MS 10

// Структура для хранения настроек модуля чтения положения
typedef struct
{
    float angle_min;                      // Минимальный угол сервопривода (градусы)
    float angle_max;                      // Максимальный угол сервопривода (градусы)
    float voltage_min;                    // Напряжение при минимальном угле (мВ)
    float voltage_max;                    // Напряжение при максимальном угле (мВ)
    bool use_voltage_divider;             // Использовать ли делитель напряжения
    float voltage_divider_factor;         // Коэффициент делителя напряжения
    bool use_enable_pin;                  // Использовать ли пин активации
    gpio_num_t enable_pin;                // Номер пина активации
    bool enable_level;                    // Уровень активации (true = HIGH, false = LOW)
    adc_oneshot_unit_handle_t adc_handle; // Handle АЦП
    adc_cali_handle_t adc_cali_handle;    // Handle калибровки АЦП
} servo_position_reader_config_t;

// Инициализация модуля чтения положения сервопривода
esp_err_t servo_position_reader_init(void);

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