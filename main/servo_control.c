/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_zb_sleepy_end_device.h"
#include "servo_control.h"
#include "esp_sleep.h" // Добавляем для работы с режимами сна

#define SERVO_GPIO (14)       // Servo GPIO
#define SERVO_POWER_GPIO (13) // GPIO для управления питанием сервопривода

// Углы для калибровки
static uint16_t calibration_angle_close = 0;  // Угол для закрытого положения (градусы)
static uint16_t calibration_angle_open = 180; // Угол для открытого положения (градусы)

// Параметры сервопривода
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_WIDTH_US 500
#define SERVO_MAX_WIDTH_US 2500
#define SERVO_FREQ 50
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define LEDC_MAX_DUTY ((1 << LEDC_DUTY_RESOLUTION) - 1)

// Время для плавного перехода (2 секунды)
#define SERVO_FADE_TIME_MS 2000

// Функция для расчета скважности по углу
static uint32_t servo_calculate_duty(uint16_t angle)
{
    // Ограничиваем угол, чтобы избежать выхода за пределы
    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    // Рассчитываем длительность импульса для заданного угла
    float pulse_width_us = SERVO_MIN_WIDTH_US +
                           ((float)(SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US) * angle / SERVO_MAX_ANGLE);

    // Рассчитываем период ШИМ
    uint32_t period_us = 1000000 / SERVO_FREQ;

    // Рассчитываем значение скважности
    uint32_t duty = (uint32_t)(((pulse_width_us * (float)LEDC_MAX_DUTY)) / period_us);

    // Ограничиваем скважность максимальным значением
    if (duty > LEDC_MAX_DUTY)
    {
        duty = LEDC_MAX_DUTY;
    }

    return duty;
}

// Функция для включения/выключения питания сервопривода
static void servo_power_control(bool power_on)
{
    gpio_set_level(SERVO_POWER_GPIO, power_on ? 1 : 0);
    if (power_on)
    {
        ESP_LOGI(TAG, "Servo power ON");
        // Даем время на стабилизацию питания
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
        ESP_LOGI(TAG, "Servo power OFF");
    }
}

void servo_control_task(void *pvParameters)
{
    uint32_t notificationValue;
    bool open_cmd;

    ESP_LOGI(TAG, "Servo control task started, waiting for notifications.");
    ESP_LOGI(TAG, "Target angles - Close: %d, Open: %d",
             calibration_angle_close, calibration_angle_open);

    for (;;)
    {
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        open_cmd = (bool)notificationValue;

        ESP_LOGI(TAG, "Servo notification received: %s", open_cmd ? "Open" : "Close");

        // Запрещаем сон Zigbee перед началом движения
        esp_zb_sleep_enable(false);

        // --- Повторная инициализация LEDC перед использованием ---
        // Сначала конфигурируем канал (привязываем GPIO)
         ledc_channel_config_t ledc_channel_reinit = {
            .channel = LEDC_CHANNEL,
            .duty = 0,
            .gpio_num = SERVO_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD // Используем безопасный режим
        };
        esp_err_t channel_ret = ledc_channel_config(&ledc_channel_reinit);
        if (channel_ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to re-initialize LEDC channel: %s", esp_err_to_name(channel_ret));
             esp_zb_sleep_enable(true);
             continue;
        }

        // Настройка таймера LEDC (как в servo_init)
        ledc_timer_config_t ledc_timer_reinit = {
            .duty_resolution = LEDC_DUTY_RESOLUTION,
            .freq_hz = SERVO_FREQ,
            .speed_mode = LEDC_MODE,
            .timer_num = LEDC_TIMER,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false // Убедимся, что это инициализация
        };
        esp_err_t timer_ret = ledc_timer_config(&ledc_timer_reinit);
         if (timer_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to re-initialize LEDC timer: %s", esp_err_to_name(timer_ret));
            // Критическая ошибка, возможно, стоит перезагрузиться или остановить задачу
             esp_zb_sleep_enable(true); // Разрешаем сон в случае ошибки
            continue; 
        }

        // Инициализируем сервис fade (как в servo_init)
        esp_err_t fade_ret = ledc_fade_func_install(0);
        // Игнорируем ошибку ESP_ERR_INVALID_STATE, если он уже был установлен
        if (fade_ret != ESP_OK && fade_ret != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "Failed to install LEDC fade service: %s", esp_err_to_name(fade_ret));
             // Обработка ошибки...
             esp_zb_sleep_enable(true); // Разрешаем сон в случае ошибки
             continue;
        }
        // --- Конец повторной инициализации LEDC ---

        // Убедимся, что таймер запущен перед операциями с каналом
        // (ledc_timer_config уже должен был его запустить, но resume не повредит)
        esp_err_t resume_ret = ledc_timer_resume(LEDC_MODE, LEDC_TIMER);
        if (resume_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to resume LEDC timer (maybe already running): %s", esp_err_to_name(resume_ret));
            // Не критично, если таймер уже запущен после config
        }

        // Включаем питание сервопривода
        servo_power_control(true);

        // Рассчитываем целевой угол и скважность
        uint16_t target_angle = open_cmd ? calibration_angle_open : calibration_angle_close;
        uint32_t target_duty = servo_calculate_duty(target_angle);

        // Запускаем плавное изменение к target_duty
        esp_err_t ret = ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL,
                                                target_duty, // Передаем рассчитанную целевую скважность
                                                SERVO_FADE_TIME_MS);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set fade: %s", esp_err_to_name(ret));
        }
        else
        {
            ret = ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start fade: %s", esp_err_to_name(ret));
            }
            else
            {
                ESP_LOGI(TAG, "Servo fade to %s (Duty: %lu) initiated.",
                         open_cmd ? "open" : "close", target_duty);
            }
        }
        // Ждем завершения fade
        vTaskDelay(pdMS_TO_TICKS(SERVO_FADE_TIME_MS)); // Задержка = времени fade

        // Выключаем питание сервопривода после завершения движения
        servo_power_control(false);

        // --- Полная деинициализация LEDC перед сном ---
        // Сначала останавливаем вывод на канале
        esp_err_t stop_ret = ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0); // Устанавливаем idle_level в 0
        if (stop_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop LEDC channel: %s", esp_err_to_name(stop_ret));
            // Обработка ошибки...
        }

        // Останавливаем таймер LEDC 
        esp_err_t pause_ret = ledc_timer_pause(LEDC_MODE, LEDC_TIMER);
         if (pause_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to pause LEDC timer: %s", esp_err_to_name(pause_ret));
            // Возможно, стоит обработать ошибку более тщательно
        }

        // Удаляем сервис fade
        ledc_fade_func_uninstall();

         // Деконфигурируем таймер LEDC
        ledc_timer_config_t ledc_timer_deinit = {
            .speed_mode = LEDC_MODE,
            .timer_num = LEDC_TIMER,
            .deconfigure = true 
        };
        esp_err_t deconfig_ret = ledc_timer_config(&ledc_timer_deinit);
        if (deconfig_ret != ESP_OK) {
             ESP_LOGE(TAG, "Failed to deconfigure LEDC timer: %s", esp_err_to_name(deconfig_ret));
             // Обработка ошибки...
        }

        // Сбрасываем конфигурацию GPIO пина, чтобы отсоединить LEDC
        gpio_reset_pin(SERVO_GPIO);

        // --- Конец полной деинициализации LEDC ---

        // Разрешаем сон Zigbee после завершения движения
        esp_zb_sleep_enable(true);
    }
}

esp_err_t servo_init(void)
{
    ESP_LOGI(TAG, "Initializing Servo Control...");

    // Настройка GPIO для управления питанием сервопривода
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SERVO_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure servo power GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Изначально питание выключено
    gpio_set_level(SERVO_POWER_GPIO, 0);
    ESP_LOGI(TAG, "Servo power GPIO initialized");

    // // Настройка канала LEDC - УБРАНО, т.к. делается в задаче
    // ledc_channel_config_t ledc_channel = {
    //     .channel = LEDC_CHANNEL,
    //     .duty = 0,
    //     .gpio_num = SERVO_GPIO,
    //     .speed_mode = LEDC_MODE,
    //     .hpoint = 0,
    //     .timer_sel = LEDC_TIMER,
    //     .intr_type = LEDC_INTR_DISABLE,
    //     .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD};

    // ret = ledc_channel_config(&ledc_channel);

    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    ESP_LOGI(TAG, "Servo control initialized. Waiting for task notification for initial position.");

    return ESP_OK; // Возвращаем ESP_OK, так как основная инициализация периферии завершена
}

// esp_err_t servo_deinit(void)
// {
//     ESP_LOGI(TAG, "Deinitializing Servo Control...");

//     // Выключаем питание сервопривода
//     servo_power_control(false);

//     // Отключаем сервис fade
//     ledc_fade_func_uninstall();

//     // Отключаем ШИМ (останавливаем таймер LEDC)
// esp_err_t ret = ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to stop LEDC: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     // Сбрасываем настройки GPIO для управления питанием
//     gpio_reset_pin(SERVO_POWER_GPIO);

//     // Сбрасываем настройки GPIO сервопривода
//     gpio_reset_pin(SERVO_GPIO);

//     ESP_LOGI(TAG, "Servo control deinitialized successfully.");

//     return ESP_OK;
// }