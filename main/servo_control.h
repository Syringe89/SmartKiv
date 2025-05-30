#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "esp_err.h" // Для esp_err_t
#include "driver/ledc.h" // Для доступа к функциям LEDC

#define SERVO_GPIO (14)       // Servo GPIO
#define SERVO_POWER_GPIO (13) // GPIO для управления питанием сервопривода


// Параметры сервопривода
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 120
#define SERVO_MIN_WIDTH_US 500
#define SERVO_MAX_WIDTH_US 2000
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
#define SERVO_MIN_FADE_TIME_MS 500         // Минимальное время перехода (мс)
#define SERVO_MAX_FADE_TIME_MS 6000        // Максимальное время перехода (мс)

// Функция для расчета скважности по углу
uint32_t servo_calculate_duty(float angle);

// Объявление задачи управления сервоприводом
void servo_control_task(void *pvParameters);

// Объявление функции инициализации LEDC
esp_err_t ledc_init(uint32_t target_duty);

// Объявление функции деинициализации сервопривода
esp_err_t ledc_deinit(void);
#endif // SERVO_CONTROL_H
