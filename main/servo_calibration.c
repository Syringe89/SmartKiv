#include "servo_calibration.h"
#include "servo_control.h"
#include "servo_position_reader.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_zigbee_core.h"

static const char *TAG = "SERVO_CALIB";

// Структура для хранения конфигурации модуля
typedef struct
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t adc_cali_handle;
    bool adc_calibrated;
    float shunt_resistance; // Сопротивление шунта в Омах
    QueueHandle_t button_evt_queue;
} servo_calibration_config_t;

// Глобальные переменные
static servo_calibration_config_t config;
static bool is_initialized = false;
static servo_calibration_data_t calib_data = {
    .is_calibrated = false,
    .min_angle = SERVO_MIN_ANGLE,
    .max_angle = SERVO_MAX_ANGLE,
    .normal_current = 50.0f, // начальное значение тока в мА
    .stall_current = 200.0f  // начальное значение тока при блокировке в мА
};

// Внешняя ссылка на handle ADC из модуля servo_position_reader
extern adc_oneshot_unit_handle_t g_adc_handle;
static bool using_external_adc_handle = false;

// Пересчитать скважность PWM для заданного угла
static uint32_t angle_to_duty(float angle)
{
    float duty_float = (((angle - SERVO_MIN_ANGLE) * (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US)) /
                        (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) +
                       SERVO_MIN_WIDTH_US;

    // Преобразование микросекунд в значение duty
    uint32_t duty = (uint32_t)((duty_float * LEDC_MAX_DUTY) / (1000000.0f / SERVO_FREQ));
    return duty;
}

// Функция для сохранения калибровочных данных
esp_err_t servo_calibration_save_data(const servo_calibration_data_t *calibration_data)
{
    if (calibration_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Сохраняем все данные калибровки одним блоком
    err = nvs_set_blob(nvs_handle, "calib_data", calibration_data, sizeof(servo_calibration_data_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка сохранения данных в NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Данные калибровки успешно сохранены");
    }

    nvs_close(nvs_handle);
    return err;
}

// Функция для загрузки калибровочных данных
esp_err_t servo_calibration_load_data(servo_calibration_data_t *calibration_data)
{
    if (calibration_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Загружаем все данные калибровки одним блоком
    size_t required_size = sizeof(servo_calibration_data_t);
    err = nvs_get_blob(nvs_handle, "calib_data", calibration_data, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Данные калибровки не найдены");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    if (required_size != sizeof(servo_calibration_data_t))
    {
        ESP_LOGW(TAG, "Размер загруженных данных не соответствует ожидаемому");
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Данные калибровки загружены: min=%0.1f°, max=%0.1f°, ток=%0.1f/%0.1f мА",
             calibration_data->min_angle, calibration_data->max_angle,
             calibration_data->normal_current, calibration_data->stall_current);

    nvs_close(nvs_handle);
    return ESP_OK;
}

// Инициализация модуля калибровки
esp_err_t servo_calibration_init(void)
{
    if (is_initialized)
    {
        ESP_LOGW(TAG, "Модуль уже инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Инициализация модуля калибровки сервопривода");

    // Инициализируем структуру конфигурации
    config.adc_calibrated = false;
    config.shunt_resistance = 2.2f; // Шунт 2.2 Ома, можно изменить

    // Загружаем сохраненные данные калибровки
    esp_err_t ret = servo_calibration_load_data(&calib_data);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Данные калибровки не найдены, будут использованы значения по умолчанию");
        calib_data.is_calibrated = false;
    }

    // Создаем очередь сообщений для кнопки
    config.button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (config.button_evt_queue == NULL)
    {
        ESP_LOGE(TAG, "Не удалось создать очередь для кнопки");
        return ESP_ERR_NO_MEM;
    }

    // Запускаем задачу мониторинга кнопки
    BaseType_t xReturned = xTaskCreate(
        button_handler_task,
        "button_handler",
        4096,
        NULL,
        5,
        NULL);

    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Не удалось создать задачу мониторинга кнопки");
        vQueueDelete(config.button_evt_queue);
        return ESP_ERR_NO_MEM;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "Модуль калибровки сервопривода инициализирован");

    return ESP_OK;
}

// Инициализация АЦП для измерения тока
static esp_err_t init_adc(void)
{
    ESP_LOGI(TAG, "Инициализация АЦП (UNIT %d) для измерения тока", SERVO_CURRENT_ADC_UNIT);
    
    // Проверяем, не был ли ADC уже инициализирован в servo_position_reader
    // Если g_adc_handle доступен и валиден, используем его
    if (g_adc_handle != NULL)
    {
        ESP_LOGI(TAG, "Используем уже инициализированный ADC из servo_position_reader");
        config.adc_handle = g_adc_handle;
        using_external_adc_handle = true;
        
        // Настраиваем только канал АЦП, так как сам ADC уже инициализирован
        adc_oneshot_chan_cfg_t chan_config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = SERVO_CURRENT_ADC_ATTEN,
        };
        
        esp_err_t ret = adc_oneshot_config_channel(config.adc_handle, SERVO_CURRENT_ADC_CHANNEL, &chan_config);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка настройки канала ADC (UNIT %d, CH %d): %s", 
                     SERVO_CURRENT_ADC_UNIT, SERVO_CURRENT_ADC_CHANNEL, esp_err_to_name(ret));
            // Не деинициализируем ADC, так как мы его не создавали
            config.adc_handle = NULL;
            using_external_adc_handle = false;
            return ret;
        }
    }
    else
    {
        // Инициализируем ADC самостоятельно, если g_adc_handle недоступен
        ESP_LOGI(TAG, "Инициализируем ADC самостоятельно");
        using_external_adc_handle = false;
        
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = SERVO_CURRENT_ADC_UNIT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        
        esp_err_t ret = adc_oneshot_new_unit(&init_config, &config.adc_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка инициализации ADC (UNIT %d): %s", 
                     SERVO_CURRENT_ADC_UNIT, esp_err_to_name(ret));
            return ret;
        }
        
        // Настройка канала АЦП
        adc_oneshot_chan_cfg_t chan_config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = SERVO_CURRENT_ADC_ATTEN,
        };
        
        ret = adc_oneshot_config_channel(config.adc_handle, SERVO_CURRENT_ADC_CHANNEL, &chan_config);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка настройки канала ADC (UNIT %d, CH %d): %s", 
                     SERVO_CURRENT_ADC_UNIT, SERVO_CURRENT_ADC_CHANNEL, esp_err_to_name(ret));
            adc_oneshot_del_unit(config.adc_handle);
            config.adc_handle = NULL;
            return ret;
        }
    }

    // Инициализация калибровки АЦП
    adc_cali_handle_t adc_cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = SERVO_CURRENT_ADC_UNIT,
        .chan = SERVO_CURRENT_ADC_CHANNEL,
        .bitwidth = ADC_BITWIDTH_12,
        .atten = SERVO_CURRENT_ADC_ATTEN,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK)
    {
        config.adc_cali_handle = adc_cali_handle;
        config.adc_calibrated = true;
        ESP_LOGI(TAG, "Калибровка АЦП (UNIT %d) успешно инициализирована", SERVO_CURRENT_ADC_UNIT);
    }
    else
    {
        ESP_LOGW(TAG, "Калибровка АЦП (UNIT %d) не поддерживается, будут использованы сырые значения", 
                SERVO_CURRENT_ADC_UNIT);
        config.adc_cali_handle = NULL;
    }

    return ESP_OK;
}

// Деинициализация АЦП
static esp_err_t deinit_adc(void)
{
    ESP_LOGI(TAG, "Деинициализация АЦП (UNIT %d) для измерения тока", SERVO_CURRENT_ADC_UNIT);

    if (config.adc_cali_handle != NULL)
    {
        adc_cali_delete_scheme_curve_fitting(config.adc_cali_handle);
        config.adc_cali_handle = NULL;
    }

    // Деинициализируем ADC только если мы его создали сами
    if (!using_external_adc_handle && config.adc_handle != NULL)
    {
        esp_err_t ret = adc_oneshot_del_unit(config.adc_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка деинициализации ADC (UNIT %d): %s", 
                     SERVO_CURRENT_ADC_UNIT, esp_err_to_name(ret));
            return ret;
        }
    }
    
    config.adc_handle = NULL;
    config.adc_calibrated = false;
    using_external_adc_handle = false;
    
    return ESP_OK;
}

// Измерение тока через шунт
esp_err_t servo_calibration_get_current(float *current_ma, bool auto_deinit)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (current_ma == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Проверяем, инициализирован ли АЦП, и если нет - инициализируем его
    if (!config.adc_calibrated && config.adc_cali_handle == NULL)
    {
        ESP_LOGI(TAG, "АЦП не инициализирован, инициализируем его");
        esp_err_t ret = init_adc();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка инициализации АЦП: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    int adc_raw;
    esp_err_t ret = adc_oneshot_read(config.adc_handle, SERVO_CURRENT_ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка чтения ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    // Вычисляем напряжение на шунте
    float voltage_mv;
    if (config.adc_calibrated)
    {
        int voltage;
        ret = adc_cali_raw_to_voltage(config.adc_cali_handle, adc_raw, &voltage);
        if (ret == ESP_OK)
        {
            voltage_mv = (float)voltage;
        }
        else
        {
            voltage_mv = ((float)adc_raw * 3300.0f) / 4095.0f;
        }
    }
    else
    {
        voltage_mv = ((float)adc_raw * 3300.0f) / 4095.0f;
    }

    // Вычисляем ток через шунт (I = U/R)
    *current_ma = voltage_mv / config.shunt_resistance;

    ESP_LOGD(TAG, "Измерен ток: %.2f мА (ADC: %d, напряжение: %.2f мВ)",
             *current_ma, adc_raw, voltage_mv);

    // Если запрошена автоматическая деинициализация АЦП
    if (auto_deinit)
    {
        ESP_LOGD(TAG, "Автоматическая деинициализация АЦП после измерения");
        deinit_adc();
    }

    return ESP_OK;
}

// Функция для измерения максимального тока во время движения сервопривода
static esp_err_t measure_peak_current_during_motion(float *peak_current, float from_angle, float to_angle)
{
    if (peak_current == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Устанавливаем начальный угол
    esp_err_t ret = servo_set_angle_smooth(from_angle, to_angle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Даем время стабилизироваться
    vTaskDelay(pdMS_TO_TICKS(200));

    // Устанавливаем целевой угол для начала движения
    ret = servo_set_angle_smooth(to_angle, from_angle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Небольшая задержка для начала движения
    vTaskDelay(pdMS_TO_TICKS(20));

    // Измеряем ток несколько раз с минимальными интервалами,
    // выбираем максимальное значение
    float max_current = 0.0f;
    float current = 0.0f;

    // Делаем несколько замеров с минимальными интервалами
    for (int i = 0; i < 20; i++)
    {
        ret = servo_calibration_get_current(&current, false);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка измерения тока: %s", esp_err_to_name(ret));
            continue;
        }

        // Обновляем максимальное значение
        if (current > max_current)
        {
            max_current = current;
        }

        // Минимальная задержка между измерениями
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    *peak_current = max_current;
    return ESP_OK;
}

// Основная функция калибровки сервопривода
esp_err_t servo_calibration_start(servo_calibration_data_t *calibration_data)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (calibration_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Начало калибровки сервопривода");

    // Порядок инициализации важен:
    // 1. Сначала инициализируем ADC для чтения положения (servo_position_reader)
    // 2. Затем инициализируем канал АЦП для измерения тока

    // Инициализируем ADC для считывания положения
    esp_err_t ret = servo_position_reader_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации ADC позиции: %s", esp_err_to_name(ret));
        return ret;
    }

    // Активируем схему считывания положения
    ret = servo_position_set_reading_enabled(true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка активации схемы считывания: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Схема считывания активирована");
    }

    // Инициализируем канал АЦП для измерения тока (используем тот же ADC unit)
    ret = init_adc();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации канала АЦП для токового шунта: %s", esp_err_to_name(ret));
        servo_position_reader_deinit();
        return ret;
    }

    // Чтение угла сервопривода
    float current_angle;
    esp_err_t angle_ret = servo_position_reader_get_angle(&current_angle);

    uint32_t current_duty = 0;

    if (angle_ret == ESP_OK)
    {
        // Если успешно получили угол, используем его для расчета текущей скважности
        current_duty = servo_calculate_duty(current_angle);
        ESP_LOGI(TAG, "Текущий угол сервопривода: %.2f (скважность: %lu)", current_angle, current_duty);
    }
    else
    {
        ESP_LOGW(TAG, "Не удалось прочитать текущий угол сервопривода: %s", esp_err_to_name(angle_ret));
        // При ошибке используем средний угол
        current_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    }

    // Инициализируем LEDC с текущим значением скважности
    ret = ledc_init(current_duty);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации LEDC");
        deinit_adc();
        servo_position_reader_deinit();
        return ret;
    }

    // Устанавливаем сервопривод в среднее положение для начала
    float center_angle = (SERVO_MAX_ANGLE + SERVO_MIN_ANGLE) / 2.0f;
    esp_err_t ret_set = servo_set_angle_smooth(center_angle, current_angle);
    if (ret_set != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка установки центрального положения: %s", esp_err_to_name(ret_set));
        deinit_adc();
        return ret_set;
    }

    ESP_LOGI(TAG, "Установка в среднее положение: %.1f градусов", center_angle);

    // Даем время на стабилизацию
    vTaskDelay(pdMS_TO_TICKS(200));

    // Измеряем базовый ток при движении сервопривода с большим углом
    float normal_current;

    // Двигаем сервопривод вперед-назад для измерения тока во время движения
    ESP_LOGI(TAG, "Измерение базового тока во время движения...");

    // Используем больший угол для гарантированного движения
    float test_angle_min = center_angle - 20.0f;
    float test_angle_max = center_angle + 20.0f;

    if (test_angle_min < SERVO_MIN_ANGLE)
    {
        test_angle_min = SERVO_MIN_ANGLE;
    }

    if (test_angle_max > SERVO_MAX_ANGLE)
    {
        test_angle_max = SERVO_MAX_ANGLE;
    }

    // Измеряем максимальный ток во время движения
    ret = measure_peak_current_during_motion(&normal_current, test_angle_min, test_angle_max);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка измерения базового тока: %s", esp_err_to_name(ret));
        deinit_adc(); // Деинициализируем АЦП при ошибке
        return ret;
    }

    // Возвращаемся в центральное положение
    ret = servo_set_angle_smooth(center_angle, test_angle_max);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка возврата в центральное положение: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Нормальный ток во время движения: %.2f мА", normal_current);
    calibration_data->normal_current = normal_current;

    // Определяем пороговый ток для обнаружения блокировки
    float threshold_current = normal_current * SERVO_CALIBRATION_THRESHOLD_FACTOR;
    ESP_LOGI(TAG, "Пороговый ток блокировки: %.2f мА", threshold_current);

    // Поиск нижней границы (минимальный угол)
    current_angle = center_angle;
    float min_angle = SERVO_MIN_ANGLE;
    float measured_current;

    ESP_LOGI(TAG, "Поиск нижней границы...");

    while (current_angle > SERVO_MIN_ANGLE)
    {
        // Уменьшаем угол на шаг
        float prev_angle = current_angle;
        current_angle -= SERVO_CALIBRATION_STEP_SIZE;
        if (current_angle < SERVO_MIN_ANGLE)
        {
            current_angle = SERVO_MIN_ANGLE;
        }

        // Измеряем ток во время движения к новому положению
        ret = measure_peak_current_during_motion(&measured_current, prev_angle, current_angle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка измерения тока: %s", esp_err_to_name(ret));
            continue;
        }

        ESP_LOGD(TAG, "Угол: %.1f°, ток: %.2f мА", current_angle, measured_current);

        // Если ток превысил пороговое значение, значит мы достигли блокировки
        if (measured_current > threshold_current)
        {
            min_angle = current_angle + SERVO_CALIBRATION_STEP_SIZE; // Возвращаемся на шаг назад
            calibration_data->stall_current = measured_current;
            ESP_LOGI(TAG, "Обнаружена нижняя граница при токе %.2f мА: %.1f градусов",
                     measured_current, min_angle);
            break;
        }
    }

    // Возвращаемся в центр
    ret = servo_set_angle_smooth(center_angle, min_angle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка возврата в центральное положение: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Поиск верхней границы (максимальный угол)
    current_angle = center_angle;
    float max_angle = SERVO_MAX_ANGLE;

    ESP_LOGI(TAG, "Поиск верхней границы...");

    while (current_angle < SERVO_MAX_ANGLE)
    {
        // Увеличиваем угол на шаг
        float prev_angle = current_angle;
        current_angle += SERVO_CALIBRATION_STEP_SIZE;
        if (current_angle > SERVO_MAX_ANGLE)
        {
            current_angle = SERVO_MAX_ANGLE;
        }

        // Измеряем ток во время движения к новому положению
        ret = measure_peak_current_during_motion(&measured_current, prev_angle, current_angle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка измерения тока: %s", esp_err_to_name(ret));
            continue;
        }

        ESP_LOGD(TAG, "Угол: %.1f°, ток: %.2f мА", current_angle, measured_current);

        // Если ток превысил пороговое значение, значит мы достигли блокировки
        if (measured_current > threshold_current)
        {
            max_angle = current_angle - SERVO_CALIBRATION_STEP_SIZE; // Возвращаемся на шаг назад

            // Обновляем значение тока при блокировке (среднее между двумя измерениями)
            calibration_data->stall_current = (calibration_data->stall_current + measured_current) / 2.0f;

            ESP_LOGI(TAG, "Обнаружена верхняя граница при токе %.2f мА: %.1f градусов",
                     measured_current, max_angle);
            break;
        }
    }

    // Сохраняем результаты калибровки
    calibration_data->min_angle = min_angle;
    calibration_data->max_angle = max_angle;
    calibration_data->min_duty = angle_to_duty(min_angle);
    calibration_data->max_duty = angle_to_duty(max_angle);
    calibration_data->is_calibrated = true;

    // Возвращаемся в положение с минимальным углом
    ret = servo_set_angle_smooth(calibration_data->min_angle, max_angle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка установки минимального угла: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Калибровка завершена. Диапазон: %.1f-%.1f градусов, ток: норм=%.2f мА, блок=%.2f мА",
             calibration_data->min_angle, calibration_data->max_angle,
             calibration_data->normal_current, calibration_data->stall_current);

    // Сохраняем данные в NVS
    ret = servo_calibration_save_data(calibration_data);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Не удалось сохранить данные калибровки: %s", esp_err_to_name(ret));
    }

    // При завершении калибровки деинициализируем модули в правильном порядке
    
    // Деинициализируем АЦП для измерения тока
    ESP_LOGI(TAG, "Освобождаем ресурсы АЦП для токового шунта");
    deinit_adc();

    // Деинициализируем LEDC
    esp_err_t deinit_ret = ledc_deinit();
    if (deinit_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка деинициализации LEDC");
    }

    // Деактивируем считывание положения, если оно было активировано
    ret = servo_position_set_reading_enabled(false);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Ошибка деактивации схемы считывания: %s", esp_err_to_name(ret));
    }

    // Деинициализируем ADC для считывания положения
    deinit_ret = servo_position_reader_deinit();
    if (deinit_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка деинициализации ADC позиции");
    }

    return ESP_OK;
}

// Задача мониторинга кнопки
void button_handler_task(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SERVO_CALIBRATION_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка настройки пина кнопки: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Запущена задача мониторинга кнопки (GPIO%d)", SERVO_CALIBRATION_BUTTON);

    TickType_t button_pressed_time = 0;
    bool button_pressed = false;
    bool action_triggered = false;

    for (;;)
    {
        int button_value = gpio_get_level(SERVO_CALIBRATION_BUTTON);

        // Инверсная логика (кнопка подтянута к VCC через подтягивающий резистор)
        bool is_pressed = (button_value == 0);

        if (is_pressed && !button_pressed)
        {
            // Кнопка только что нажата
            button_pressed = true;
            button_pressed_time = xTaskGetTickCount();
            action_triggered = false;
            ESP_LOGD(TAG, "Кнопка нажата");
        }
        else if (is_pressed && button_pressed)
        {
            // Кнопка всё ещё нажата, проверяем длительное нажатие для калибровки
            TickType_t current_time = xTaskGetTickCount();
            TickType_t pressed_duration = current_time - button_pressed_time;
            
            // Если достигнут порог длительного нажатия и действие ещё не выполнено
            if (!action_triggered && (pressed_duration * portTICK_PERIOD_MS >= SERVO_CALIBRATION_BUTTON_PRESS_DURATION))
            {
                ESP_LOGI(TAG, "Обнаружено длительное нажатие кнопки (%lu мс), запуск калибровки",
                         (unsigned long)(pressed_duration * portTICK_PERIOD_MS));

                // Запускаем калибровку
                servo_calibration_start(&calib_data);
                action_triggered = true;
            }
        }
        else if (!is_pressed && button_pressed)
        {
            // Кнопка отпущена
            button_pressed = false;
            TickType_t pressed_duration = xTaskGetTickCount() - button_pressed_time;
            ESP_LOGD(TAG, "Кнопка отпущена, длительность: %lu мс",
                     pressed_duration * portTICK_PERIOD_MS);

            // Если это было короткое нажатие (меньше порога длительного нажатия) и действие ещё не выполнено
            if (!action_triggered && (pressed_duration * portTICK_PERIOD_MS < SERVO_CALIBRATION_BUTTON_PRESS_DURATION))
            {
                ESP_LOGI(TAG, "Обнаружено короткое нажатие кнопки (%lu мс), выполняем сброс Zigbee",
                         (unsigned long)(pressed_duration * portTICK_PERIOD_MS));
                
                // Выполняем сброс Zigbee через локальное действие
                ESP_LOGI(TAG, "Выполняем сброс Zigbee...");
                esp_zb_bdb_reset_via_local_action();

                // Запускаем процесс подключения к сети
                ESP_LOGI(TAG, "Запускаем процесс подключения к сети...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                
                action_triggered = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Опрос с периодом 50 мс
    }
}

// Деинициализация модуля
esp_err_t servo_calibration_deinit(void)
{
    if (!is_initialized)
    {
        ESP_LOGW(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    // Деинициализируем АЦП, если он был инициализирован
    if (config.adc_cali_handle != NULL || config.adc_calibrated)
    {
        deinit_adc();
    }

    // Очищаем очередь
    if (config.button_evt_queue != NULL)
    {
        vQueueDelete(config.button_evt_queue);
        config.button_evt_queue = NULL;
    }

    is_initialized = false;
    ESP_LOGI(TAG, "Модуль калибровки деинициализирован");

    return ESP_OK;
}