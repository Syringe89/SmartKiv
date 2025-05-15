#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H

#include "esp_err.h"
#include <stdbool.h>

// Структура для хранения результатов калибровки
typedef struct {
    float min_angle;        // Минимальный угол сервопривода (градусы)
    float max_angle;        // Максимальный угол сервопривода (градусы)
    float min_voltage_mv;   // Напряжение при минимальном угле (мВ)
    float max_voltage_mv;   // Напряжение при максимальном угле (мВ)
    bool calibration_valid; // Флаг успешной калибровки
} servo_calibration_result_t;

// Настройки для калибровки
typedef struct {
    float step_size_deg;         // Размер шага при движении сервопривода (градусы)
    float step_delay_ms;         // Задержка между шагами (мс)
    float threshold_voltage_mv;  // Пороговое изменение напряжения для определения края (мВ)
    int samples_per_position;    // Количество измерений в каждой позиции для усреднения
    float timeout_sec;           // Таймаут калибровки (сек)
} servo_calibration_config_t;

// Инициализация настроек калибровки по умолчанию
esp_err_t servo_calibration_init_config(servo_calibration_config_t *config);

// Запуск процедуры автоматической калибровки
esp_err_t servo_calibration_run(servo_calibration_config_t *config, servo_calibration_result_t *result);

// Применение результатов калибровки
esp_err_t servo_calibration_apply_results(const servo_calibration_result_t *result);

// Определение границ рабочего диапазона сервопривода
esp_err_t servo_calibration_find_limits(const servo_calibration_config_t *config, 
                                       float *min_angle, float *min_voltage,
                                       float *max_angle, float *max_voltage);

#endif // SERVO_CALIBRATION_H 