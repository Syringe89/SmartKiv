#include <stdio.h>
#include <math.h>
#include <string.h> // Для memcpy
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
#include "servo_calibration.h" // Добавляем заголовочный файл для калибровки
#include "esp_sleep.h"             // Добавляем для работы с режимами сна

static const char *TAG = "SERVO_CTRL"; // Определяем TAG для модуля

// Глобальная конфигурация сервопривода
static servo_control_config_t servo_config;
static bool is_initialized = false;

// Функция для расчета скважности по углу
uint32_t servo_calculate_duty(float angle)
{
    ESP_LOGI(TAG, "Начало расчета скважности для угла: %.2f градусов", angle);
    
    // Ограничиваем угол, чтобы избежать выхода за пределы
    if (angle > servo_config.max_angle)
    {
        ESP_LOGW(TAG, "Угол превышает максимум (%.2f), ограничиваем до %.2f", angle, servo_config.max_angle);
        angle = servo_config.max_angle;
    }

    if (angle < servo_config.min_angle)
    {
        ESP_LOGW(TAG, "Угол меньше минимума (%.2f), ограничиваем до %.2f", angle, servo_config.min_angle);
        angle = servo_config.min_angle;
    }

    uint32_t duty;
    
    // Проверяем, доступны ли калиброванные значения
    if (servo_config.is_calibrated) {
        // Используем линейную интерполяцию между калиброванными значениями
        if (angle <= servo_config.calibration_min_angle) {
            duty = servo_config.calibration_min_duty;
        } else if (angle >= servo_config.calibration_max_angle) {
            duty = servo_config.calibration_max_duty;
        } else {
            // Линейная интерполяция
            float ratio = (angle - servo_config.calibration_min_angle) / 
                         (servo_config.calibration_max_angle - servo_config.calibration_min_angle);
            duty = (uint32_t)(servo_config.calibration_min_duty + 
                             ratio * (servo_config.calibration_max_duty - servo_config.calibration_min_duty));
        }
        
        ESP_LOGI(TAG, "Используется калиброванное значение скважности: %u", duty);
    } else {
        // Стандартный расчет, если калибровка недоступна
        float pulse_width_us = servo_config.min_pulse_width_us +
                             ((float)(servo_config.max_pulse_width_us - servo_config.min_pulse_width_us) * 
                              (angle - servo_config.min_angle) / (servo_config.max_angle - servo_config.min_angle));

        // Рассчитываем период ШИМ
        uint32_t period_us = 1000000 / servo_config.ledc_freq_hz;

        // Рассчитываем значение скважности
        duty = (uint32_t)((pulse_width_us * (1 << servo_config.duty_resolution)) / period_us);
        
        ESP_LOGI(TAG, "Используется стандартный расчет скважности: %u", duty);
    }

    // Ограничиваем скважность максимальным значением
    uint32_t max_duty = (1 << servo_config.duty_resolution) - 1;
    if (duty > max_duty)
    {
        ESP_LOGW(TAG, "Скважность превышает максимум (%u > %u), ограничиваем", duty, max_duty);
        duty = max_duty;
    }

    ESP_LOGI(TAG, "Итоговая скважность: %u", duty);
    return duty;
}


// Функция для инициализации LEDC
esp_err_t ledc_init(uint32_t target_duty)
{
    ESP_LOGI(TAG, "Инициализация LEDC с целевой скважностью=%u", target_duty);

    // Сначала конфигурируем таймер LEDC
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = servo_config.duty_resolution,
        .freq_hz = servo_config.ledc_freq_hz,
        .speed_mode = servo_config.ledc_mode,
        .timer_num = servo_config.ledc_timer,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    
    esp_err_t timer_ret = ledc_timer_config(&ledc_timer);
    if (timer_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации таймера LEDC: %s", esp_err_to_name(timer_ret));
        return timer_ret;
    }

    // Убедимся, что таймер запущен перед операциями с каналом
    esp_err_t resume_ret = ledc_timer_resume(servo_config.ledc_mode, servo_config.ledc_timer);
    if (resume_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Не удалось возобновить таймер LEDC: %s", esp_err_to_name(resume_ret));
    }

    // Затем конфигурируем канал (привязываем GPIO) с целевым значением duty
    ledc_channel_config_t ledc_channel = {
        .channel = servo_config.ledc_channel,
        .duty = target_duty, // Устанавливаем целевое значение сразу
        .gpio_num = servo_config.servo_gpio,
        .speed_mode = servo_config.ledc_mode,
        .hpoint = 0,
        .timer_sel = servo_config.ledc_timer,
        .intr_type = LEDC_INTR_DISABLE,
        .flags.output_invert = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD // Используем безопасный режим
    };
    
    esp_err_t channel_ret = ledc_channel_config(&ledc_channel);
    if (channel_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации канала LEDC: %s", esp_err_to_name(channel_ret));
        return channel_ret;
    }

    // Инициализируем сервис fade
    esp_err_t fade_ret = ledc_fade_func_install(0);
    // Игнорируем ошибку ESP_ERR_INVALID_STATE, если он уже был установлен
    if (fade_ret != ESP_OK && fade_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Ошибка установки сервиса плавного изменения LEDC: %s", esp_err_to_name(fade_ret));
        return fade_ret;
    }

    // Явно обновляем duty после конфигурации канала и таймера
    esp_err_t update_ret = ledc_update_duty(servo_config.ledc_mode, servo_config.ledc_channel);
    if (update_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка обновления скважности LEDC: %s", esp_err_to_name(update_ret));
        return update_ret;
    }
    
    ESP_LOGI(TAG, "LEDC успешно инициализирован со скважностью=%u", target_duty);
    
    // Добавляем небольшую задержку для применения настроек
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

// Функция для деинициализации LEDC
esp_err_t ledc_deinit(void)
{
    // Сначала останавливаем вывод на канале
    esp_err_t stop_ret = ledc_stop(servo_config.ledc_mode, servo_config.ledc_channel, 0); // Устанавливаем idle_level в 0
    if (stop_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка остановки канала LEDC: %s", esp_err_to_name(stop_ret));
        return stop_ret;
    }

    // Останавливаем таймер LEDC
    esp_err_t pause_ret = ledc_timer_pause(servo_config.ledc_mode, servo_config.ledc_timer);
    if (pause_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка приостановки таймера LEDC: %s", esp_err_to_name(pause_ret));
        return pause_ret;
    }

    // Удаляем сервис fade
    ledc_fade_func_uninstall();

    // Деконфигурируем таймер LEDC
    ledc_timer_config_t ledc_timer_deinit = {
        .speed_mode = servo_config.ledc_mode,
        .timer_num = servo_config.ledc_timer,
        .deconfigure = true
    };
    
    esp_err_t deconfig_ret = ledc_timer_config(&ledc_timer_deinit);
    if (deconfig_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка деконфигурации таймера LEDC: %s", esp_err_to_name(deconfig_ret));
        return deconfig_ret;
    }

    // Сбрасываем конфигурацию GPIO пина, чтобы отсоединить LEDC
    gpio_reset_pin(servo_config.servo_gpio);
    return ESP_OK;
}

// Функция для расчета времени перехода сервопривода
static uint32_t calculate_servo_fade_time(float current_angle, float target_angle)
{
    // Используем разницу углов для расчета времени
    float angle_diff = fabsf(target_angle - current_angle);
    
    // Рассчитываем время на основе разницы углов
    uint32_t fade_time_ms = (uint32_t)(angle_diff * servo_config.ms_per_degree);
    
    ESP_LOGI(TAG, "Рассчитанное время перехода на основе разницы углов (%.2f градусов): %u мс", 
            angle_diff, fade_time_ms);
    
    // Ограничиваем время перехода минимальным и максимальным значениями
    if (fade_time_ms < servo_config.min_fade_time_ms) {
        fade_time_ms = servo_config.min_fade_time_ms;
        ESP_LOGI(TAG, "Применено минимальное время перехода: %u мс", fade_time_ms);
    } else if (fade_time_ms > servo_config.max_fade_time_ms) {
        fade_time_ms = servo_config.max_fade_time_ms;
        ESP_LOGI(TAG, "Применено максимальное время перехода: %u мс", fade_time_ms);
    }
    
    return fade_time_ms;
}

// Функция для плавного установления угла сервопривода
esp_err_t servo_set_angle_smooth(float target_angle, float current_angle)
{
    if (!is_initialized) {
        ESP_LOGE(TAG, "Сервопривод не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (target_angle > servo_config.max_angle || target_angle < servo_config.min_angle)
    {
        ESP_LOGE(TAG, "Целевой угол %.2f° вне допустимого диапазона [%.2f°-%.2f°]",
                 target_angle, servo_config.min_angle, servo_config.max_angle);
        return ESP_ERR_INVALID_ARG;
    }

    // Рассчитываем скважность для целевого угла
    uint32_t target_duty = servo_calculate_duty(target_angle);
    
    // Рассчитываем время перехода на основе разницы углов
    uint32_t fade_time_ms = calculate_servo_fade_time(current_angle, target_angle);

    // Запускаем плавное изменение от текущей позиции к целевой
    esp_err_t ret = ledc_set_fade_with_time(servo_config.ledc_mode, 
                                          servo_config.ledc_channel,
                                          target_duty,
                                          fade_time_ms);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка установки плавного изменения: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_fade_start(servo_config.ledc_mode, servo_config.ledc_channel, LEDC_FADE_NO_WAIT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка запуска плавного изменения: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Плавное изменение сервопривода с %.2f° на %.2f° (Скважность: %u) запущено со временем: %u мс.",
             current_angle, target_angle, target_duty, fade_time_ms);

    return ESP_OK;
}

// Функция инициализации сервопривода
esp_err_t servo_control_init(const servo_control_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Конфигурация сервопривода не предоставлена");
        return ESP_ERR_INVALID_ARG;
    }

    // Копируем конфигурацию
    memcpy(&servo_config, config, sizeof(servo_control_config_t));

    // Если используется управление питанием, настраиваем GPIO
    if (servo_config.power_gpio != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << servo_config.power_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка настройки GPIO питания сервопривода: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    is_initialized = true;
    ESP_LOGI(TAG, "Сервопривод успешно инициализирован");
    return ESP_OK;
}

void servo_control_task(void *pvParameters)
{
    uint32_t notificationValue;
    bool open_cmd;

    ESP_LOGI(TAG, "Задача управления сервоприводом запущена, ожидание уведомлений.");
    
    // Попытка загрузить данные калибровки
    servo_calibration_data_t calib_data;
    esp_err_t load_ret = servo_calibration_load_data(&calib_data);
    if (load_ret == ESP_OK && calib_data.is_calibrated) {
        // Обновляем калибровочные значения в конфигурации
        servo_config.is_calibrated = true;
        servo_config.calibration_min_angle = calib_data.min_angle;
        servo_config.calibration_max_angle = calib_data.max_angle;
        servo_config.calibration_min_duty = calib_data.min_duty;
        servo_config.calibration_max_duty = calib_data.max_duty;
        
        ESP_LOGI(TAG, "Данные калибровки загружены. Диапазон: %.2f-%.2f градусов.",
                 servo_config.calibration_min_angle, servo_config.calibration_max_angle);
    } else {
        ESP_LOGW(TAG, "Данные калибровки не найдены или недействительны, используем значения по умолчанию");
    }

    for (;;)
    {
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        open_cmd = (bool)notificationValue;

        ESP_LOGI(TAG, "Получено уведомление для сервопривода: %s", open_cmd ? "Открыть" : "Закрыть");

        // Запрещаем сон Zigbee перед началом движения
        esp_zb_sleep_enable(false);

        // Используем калиброванные углы, если доступны, иначе стандартные
        float target_angle;
        if (servo_config.is_calibrated) {
            target_angle = open_cmd ? servo_config.calibration_max_angle : servo_config.calibration_min_angle;
        } else {
            target_angle = open_cmd ? servo_config.max_angle : servo_config.min_angle;
        }

        // Инициализируем ADC
        esp_err_t init_ret = servo_position_reader_init();
        if (init_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка инициализации АЦП");
            esp_zb_sleep_enable(true);
            continue;
        }

        // Перед чтением активируем схему считывания
        esp_err_t ret = servo_position_set_reading_enabled(true);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка активации схемы считывания: %s", esp_err_to_name(ret));
        }

        // Чтение угла сервопривода
        // Определяем текущее положение сервопривода
        float current_angle;
        esp_err_t angle_ret = servo_position_reader_get_angle(&current_angle);

        uint32_t current_duty = 0;

        if (angle_ret == ESP_OK)
        {
            // Если успешно получили угол, используем его для расчета текущей скважности
            current_duty = servo_calculate_duty(current_angle);
            ESP_LOGI(TAG, "Текущий угол сервопривода: %.2f (скважность: %u)", current_angle, current_duty);
        }
        else
        {
            ESP_LOGW(TAG, "Не удалось прочитать текущий угол сервопривода: %s", esp_err_to_name(angle_ret));
            // Если не удалось прочитать угол, используем минимальный угол
            current_angle = servo_config.min_angle; 
            current_duty = servo_calculate_duty(current_angle);
        }

        // Инициализируем LEDC с текущим значением скважности
        init_ret = ledc_init(current_duty);
        if (init_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка инициализации LEDC");
            esp_zb_sleep_enable(true);
            continue;
        }

        // Плавно устанавливаем угол сервопривода
        ret = servo_set_angle_smooth(target_angle, current_angle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка установки угла сервопривода");
        }

        // Добавляем задержку для плавного перехода
        uint32_t fade_time_ms = calculate_servo_fade_time(current_angle, target_angle);
        vTaskDelay(pdMS_TO_TICKS(fade_time_ms + 100)); // Добавляем 100 мс для плавного перехода

        // Деактивируем схему считывания
        ret = servo_position_set_reading_enabled(false);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка деактивации схемы считывания: %s", esp_err_to_name(ret));
        }

        // Деинициализируем LEDC перед сном
        esp_err_t deinit_ret = ledc_deinit();
        if (deinit_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка деинициализации LEDC");
        }

        // Деинициализируем ADC перед сном
        deinit_ret = servo_position_reader_deinit();
        if (deinit_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка деинициализации АЦП");
        }

        // Разрешаем сон Zigbee после завершения движения
        esp_zb_sleep_enable(true);
    }
}

// esp_err_t servo_init(void)
// {
//     ESP_LOGI(TAG, "Инициализация управления сервоприводом...");

//     // Настройка GPIO для управления питанием сервопривода
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << SERVO_POWER_GPIO),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE};

//     esp_err_t ret = gpio_config(&io_conf);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Ошибка настройки GPIO питания сервопривода: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     // // Изначально питание выключено
//     // gpio_set_level(SERVO_POWER_GPIO, 0);
//     // ESP_LOGI(TAG, "GPIO питания сервопривода инициализирован");

//     ESP_LOGI(TAG, "Управление сервоприводом инициализировано. Ожидание уведомления задачи для начальной позиции.");

//     return ESP_OK; // Возвращаем ESP_OK, так как основная инициализация периферии завершена
// }

// Функция для обновления данных калибровки
esp_err_t servo_control_update_calibration(const servo_calibration_data_t *calibration_data)
{
    if (calibration_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!calibration_data->is_calibrated) {
        ESP_LOGW(TAG, "Попытка обновить калибровочные данные некалиброванными значениями");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Обновляем калибровочные значения в конфигурации
    servo_config.is_calibrated = true;
    servo_config.calibration_min_angle = calibration_data->min_angle;
    servo_config.calibration_max_angle = calibration_data->max_angle;
    servo_config.calibration_min_duty = calibration_data->min_duty;
    servo_config.calibration_max_duty = calibration_data->max_duty;
    
    ESP_LOGI(TAG, "Калибровочные данные обновлены. Диапазон: %.2f-%.2f градусов, скважность: %u-%u",
             servo_config.calibration_min_angle, servo_config.calibration_max_angle,
             servo_config.calibration_min_duty, servo_config.calibration_max_duty);
    
    return ESP_OK;
}