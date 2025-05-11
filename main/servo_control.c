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
#include "servo_position_reader.h" // Добавляем заголовочный файл для чтения положения
#include "esp_sleep.h"             // Добавляем для работы с режимами сна

// Углы для калибровки
static float calibration_angle_close = SERVO_MIN_ANGLE; // Угол для закрытого положения (градусы)
static float calibration_angle_open = SERVO_MAX_ANGLE;  // Угол для открытого положения (градусы)

// Функция для расчета скважности по углу
static uint32_t servo_calculate_duty(float angle)
{
    ESP_LOGI(TAG, "Начало расчета duty для угла: %.2f градусов", angle);
    
    // Ограничиваем угол, чтобы избежать выхода за пределы
    if (angle > SERVO_MAX_ANGLE)
    {
        ESP_LOGW(TAG, "Угол превышает максимум (%.2f), ограничиваем до %d", angle, SERVO_MAX_ANGLE);
        angle = SERVO_MAX_ANGLE;
    }

    if (angle < SERVO_MIN_ANGLE)
    {
        ESP_LOGW(TAG, "Угол меньше минимума (%.2f), ограничиваем до %d", angle, SERVO_MIN_ANGLE);
        angle = SERVO_MIN_ANGLE;
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
        ESP_LOGW(TAG, "Скважность превышает максимум (%lu > %d), ограничиваем", duty, LEDC_MAX_DUTY);
        duty = LEDC_MAX_DUTY;
    }

    ESP_LOGI(TAG, "Итоговая скважность: %lu", duty);
    return duty;
}

// // Функция для включения/выключения питания сервопривода
// static void servo_power_control(bool power_on)
// {
//     gpio_set_level(SERVO_POWER_GPIO, power_on ? 1 : 0);
//     if (power_on)
//     {
//         ESP_LOGI(TAG, "Servo power ON");
//         // Даем время на стабилизацию питания
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     else
//     {
//         ESP_LOGI(TAG, "Servo power OFF");
//     }
// }

// Функция для инициализации LEDC
static esp_err_t ledc_init(uint32_t target_duty)
{
    ESP_LOGI(TAG, "Инициализация LEDC с target_duty=%lu", target_duty);

    // Сначала конфигурируем таймер LEDC
    ledc_timer_config_t ledc_timer_reinit = {
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .freq_hz = SERVO_FREQ,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false // Убедимся, что это инициализация
    };
    esp_err_t timer_ret = ledc_timer_config(&ledc_timer_reinit);
    if (timer_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to re-initialize LEDC timer: %s", esp_err_to_name(timer_ret));
        return timer_ret;
    }

    // Убедимся, что таймер запущен перед операциями с каналом
    esp_err_t resume_ret = ledc_timer_resume(LEDC_MODE, LEDC_TIMER);
    if (resume_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to resume LEDC timer (maybe already running): %s", esp_err_to_name(resume_ret));
        // Не критично, если таймер уже запущен после config
    }

    // Затем конфигурируем канал (привязываем GPIO) с целевым значением duty
    ledc_channel_config_t ledc_channel_reinit = {
        .channel = LEDC_CHANNEL,
        .duty = target_duty, // Устанавливаем целевое значение сразу
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD // Используем безопасный режим
    };
    esp_err_t channel_ret = ledc_channel_config(&ledc_channel_reinit);
    if (channel_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to re-initialize LEDC channel: %s", esp_err_to_name(channel_ret));
        return channel_ret;
    }

    // Инициализируем сервис fade
    esp_err_t fade_ret = ledc_fade_func_install(0);
    // Игнорируем ошибку ESP_ERR_INVALID_STATE, если он уже был установлен
    if (fade_ret != ESP_OK && fade_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install LEDC fade service: %s", esp_err_to_name(fade_ret));
        return fade_ret;
    }

    // Явно обновляем duty после конфигурации канала и таймера
    esp_err_t update_ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (update_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update LEDC duty: %s", esp_err_to_name(update_ret));
        return update_ret;
    }
    
    ESP_LOGI(TAG, "LEDC успешно инициализирован с duty=%lu", target_duty);
    
    // Добавляем небольшую задержку для применения настроек
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

// Функция для деинициализации LEDC
static esp_err_t ledc_deinit(void)
{
    // Сначала останавливаем вывод на канале
    esp_err_t stop_ret = ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0); // Устанавливаем idle_level в 0
    if (stop_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop LEDC channel: %s", esp_err_to_name(stop_ret));
        return stop_ret;
    }

    // Останавливаем таймер LEDC
    esp_err_t pause_ret = ledc_timer_pause(LEDC_MODE, LEDC_TIMER);
    if (pause_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to pause LEDC timer: %s", esp_err_to_name(pause_ret));
        return pause_ret;
    }

    // Удаляем сервис fade
    ledc_fade_func_uninstall();

    // Деконфигурируем таймер LEDC
    ledc_timer_config_t ledc_timer_deinit = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .deconfigure = true};
    esp_err_t deconfig_ret = ledc_timer_config(&ledc_timer_deinit);
    if (deconfig_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deconfigure LEDC timer: %s", esp_err_to_name(deconfig_ret));
        return deconfig_ret;
    }

    // Сбрасываем конфигурацию GPIO пина, чтобы отсоединить LEDC
    gpio_reset_pin(SERVO_GPIO);

    return ESP_OK;
}

// Функция для расчета времени перехода сервопривода
static uint32_t calculate_servo_fade_time(float current_angle, float target_angle)
{
    // Используем разницу углов для расчета времени
    float angle_diff = fabsf(target_angle - current_angle);
    
    // Рассчитываем время на основе разницы углов
    uint32_t fade_time_ms = (uint32_t)(angle_diff * SERVO_MS_PER_DEGREE);
    
    ESP_LOGI(TAG, "Рассчитанное время перехода на основе разницы углов (%.2f градусов): %lu мс", 
            angle_diff, fade_time_ms);
    
    // Ограничиваем время перехода минимальным и максимальным значениями
    if (fade_time_ms < SERVO_MIN_FADE_TIME_MS) {
        fade_time_ms = SERVO_MIN_FADE_TIME_MS;
        ESP_LOGI(TAG, "Применено минимальное время перехода: %lu мс", fade_time_ms);
    } else if (fade_time_ms > SERVO_MAX_FADE_TIME_MS) {
        fade_time_ms = SERVO_MAX_FADE_TIME_MS;
        ESP_LOGI(TAG, "Применено максимальное время перехода: %lu мс", fade_time_ms);
    }
    
    return fade_time_ms;
}

void servo_control_task(void *pvParameters)
{
    uint32_t notificationValue;
    bool open_cmd;

    ESP_LOGI(TAG, "Servo control task started, waiting for notifications.");
    ESP_LOGI(TAG, "Target angles - Close: %.2f, Open: %.2f",
             calibration_angle_close, calibration_angle_open);

    for (;;)
    {
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        open_cmd = (bool)notificationValue;

        ESP_LOGI(TAG, "Servo notification received: %s", open_cmd ? "Open" : "Close");

        // Запрещаем сон Zigbee перед началом движения
        esp_zb_sleep_enable(false);

        // Рассчитываем целевой угол и скважность сразу
        float target_angle = open_cmd ? calibration_angle_open : calibration_angle_close;
        uint32_t target_duty = servo_calculate_duty(target_angle);

        // Инициализируем ADC
        esp_err_t init_ret = servo_position_reader_init();
        if (init_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "ADC initialization failed");
            esp_zb_sleep_enable(true);
            continue;
        }

        // Перед чтением активируем схему считывания
        esp_err_t ret = servo_position_set_reading_enabled(true);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка активации схемы считывания: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "Схема считывания активирована");
        }

        // Чтение угла сервопривода
        // Определяем текущее положение сервопривода
        float angle = 0.0f;
        esp_err_t angle_ret = ESP_OK;
        angle_ret = servo_position_reader_get_angle(&angle);

        uint32_t current_duty = 0;

        if (angle_ret == ESP_OK)
        {
            // Если успешно получили угол, используем его для расчета текущей скважности
            current_duty = servo_calculate_duty(angle);
            ESP_LOGI(TAG, "Current servo angle: %.2f (duty: %lu)", angle, current_duty);
        }
        else
        {
            // В случае ошибки используем безопасное начальное значение
            ESP_LOGW(TAG, "Failed to read current servo angle: %s", esp_err_to_name(angle_ret));
            // Используем закрытое положение как безопасное начальное
            current_duty = servo_calculate_duty(calibration_angle_close);
        }

        // Инициализируем LEDC с текущим значением скважности
        init_ret = ledc_init(current_duty);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "LEDC initialization failed");
            esp_zb_sleep_enable(true);
            continue;
        }

        // Рассчитываем время перехода только на основе разницы углов
        float current_angle = (angle_ret == ESP_OK) ? angle : calibration_angle_close;
        uint32_t fade_time_ms = calculate_servo_fade_time(current_angle, target_angle);

        // Запускаем плавное изменение от текущей позиции к целевой
        ret = ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL,
                                      target_duty, // Целевая скважность
                                      fade_time_ms);
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
                ESP_LOGI(TAG, "Servo fade from %.2f° to %.2f° (Duty: %lu->%lu) initiated with time: %lu мс.",
                         current_angle, target_angle, current_duty, target_duty, fade_time_ms);
            }
        }

        // Ждем завершения fade
        vTaskDelay(pdMS_TO_TICKS(fade_time_ms + 100)); // Задержка = рассчитанному времени fade + 100мс запас

        // Деактивируем схему чтения после получения значения
        ret = servo_position_set_reading_enabled(false);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка деактивации схемы считывания: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "Схема считывания деактивирована");
        }

        // Деинициализируем LEDC перед сном
        esp_err_t deinit_ret = ledc_deinit();
        if (deinit_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "LEDC deinitialization failed");
        }

        // Деинициализируем ADC перед сном
        deinit_ret = servo_position_reader_deinit();
        if (deinit_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "ADC deinitialization failed");
        }

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

    ESP_LOGI(TAG, "Servo control initialized. Waiting for task notification for initial position.");

    return ESP_OK; // Возвращаем ESP_OK, так как основная инициализация периферии завершена
}