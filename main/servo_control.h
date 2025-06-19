#pragma once

#include "esp_err.h" // Для esp_err_t
#include "servo_calibration.h" // Добавляем для типа servo_calibration_data_t
#include "driver/ledc.h"
#include "driver/gpio.h"

#define SERVO_GPIO (14)       // Servo GPIO
#define SERVO_POWER_GPIO (13) // GPIO для управления питанием сервопривода


// Параметры сервопривода
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_WIDTH_US 500
#define SERVO_MAX_WIDTH_US 2500
#define SERVO_MIN_VOLTAGE_MV 669
#define SERVO_MAX_VOLTAGE_MV 2615
#define SERVO_FREQ 50
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define LEDC_MAX_DUTY ((1 << LEDC_DUTY_RESOLUTION) - 1)

// Константы для расчета времени перехода сервопривода
#define SERVO_MS_PER_DEGREE 20.0f          // Милисекунд на градус движения сервопривода
#define SERVO_MS_PER_DUTY_UNIT 4.0f        // Базовый множитель для расчета времени по duty
#define SERVO_MIN_FADE_TIME_MS 100         // Минимальное время перехода (мс)
#define SERVO_MAX_FADE_TIME_MS 6000        // Максимальное время перехода (мс)

// Структура для хранения конфигурации сервопривода
typedef struct {
    // Параметры GPIO
    gpio_num_t servo_gpio;           // GPIO пин для управления сервоприводом
    gpio_num_t power_gpio;          // GPIO пин для управления питанием сервопривода (если используется)

    // Параметры LEDC
    ledc_mode_t ledc_mode;         // Режим работы LEDC (обычно LEDC_LOW_SPEED_MODE)
    ledc_timer_t ledc_timer;       // Номер таймера LEDC
    ledc_channel_t ledc_channel;   // Номер канала LEDC
    uint32_t ledc_freq_hz;        // Частота ШИМ в Гц
    ledc_timer_bit_t duty_resolution; // Разрешение ШИМ в битах

    // Параметры сервопривода
    float min_angle;              // Минимальный угол сервопривода (в градусах)
    float max_angle;              // Максимальный угол сервопривода (в градусах)
    uint32_t min_pulse_width_us;  // Минимальная длительность импульса (в микросекундах)
    uint32_t max_pulse_width_us;  // Максимальная длительность импульса (в микросекундах)

    // Параметры плавного движения
    float ms_per_degree;          // Миллисекунд на градус при движении
    uint32_t min_fade_time_ms;    // Минимальное время перехода (в миллисекундах)
    uint32_t max_fade_time_ms;    // Максимальное время перехода (в миллисекундах)

    // Калибровочные значения
    bool is_calibrated;           // Флаг калибровки
    float calibration_min_angle;  // Калиброванный минимальный угол
    float calibration_max_angle;  // Калиброванный максимальный угол
    uint32_t calibration_min_duty; // Калиброванная минимальная скважность
    uint32_t calibration_max_duty; // Калиброванная максимальная скважность
} servo_control_config_t;

// Значения по умолчанию для конфигурации сервопривода
#define SERVO_CONTROL_DEFAULT_CONFIG() { \
    .servo_gpio = SERVO_GPIO,            \
    .power_gpio = SERVO_POWER_GPIO,      \
    .ledc_mode = LEDC_MODE,             \
    .ledc_timer = LEDC_TIMER,           \
    .ledc_channel = LEDC_CHANNEL,       \
    .ledc_freq_hz = SERVO_FREQ,         \
    .duty_resolution = LEDC_DUTY_RESOLUTION, \
    .min_angle = SERVO_MIN_ANGLE,        \
    .max_angle = SERVO_MAX_ANGLE,        \
    .min_pulse_width_us = SERVO_MIN_WIDTH_US, \
    .max_pulse_width_us = SERVO_MAX_WIDTH_US, \
    .ms_per_degree = SERVO_MS_PER_DEGREE, \
    .min_fade_time_ms = SERVO_MIN_FADE_TIME_MS, \
    .max_fade_time_ms = SERVO_MAX_FADE_TIME_MS, \
    .is_calibrated = false,              \
    .calibration_min_angle = SERVO_MIN_ANGLE, \
    .calibration_max_angle = SERVO_MAX_ANGLE, \
    .calibration_min_duty = 0,           \
    .calibration_max_duty = 0            \
}

// Объявление функции инициализации LEDC
esp_err_t ledc_init(uint32_t target_duty);

// Объявление функции деинициализации LEDC
esp_err_t ledc_deinit(void);

// Расчет скважности для заданного угла
uint32_t servo_calculate_duty(float angle);

// Задача управления сервоприводом
void servo_control_task(void *pvParameters);

// Объявление функции деинициализации сервопривода

// Функция для плавного установления угла сервопривода
esp_err_t servo_set_angle_smooth(float target_angle, float current_angle);

/**
 * @brief Обновить данные калибровки сервопривода в модуле управления
 * 
 * @param calibration_data Указатель на структуру с новыми данными калибровки
 * @return esp_err_t ESP_OK при успешном обновлении
 */
esp_err_t servo_control_update_calibration(const servo_calibration_data_t *calibration_data);

// Функции для работы с сервоприводом
esp_err_t servo_control_init(const servo_control_config_t *config);
