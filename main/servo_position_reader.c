#include "servo_position_reader.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SERVO_POS";

// Глобальные переменные для хранения настроек
static servo_position_reader_config_t config;
static bool is_initialized = false;
static bool reading_enabled = false;

esp_err_t servo_position_reader_init(void)
{
    // Инициализация модуля чтения положения сервопривода
    config.angle_min = 0.0f;      // Минимальный угол сервопривода (0 градусов)
    config.angle_max = 360.0f;    // Максимальный угол сервопривода (180 градусов)
    config.voltage_min = 0.0f;    // Напряжение при минимальном угле (мВ)
    config.voltage_max = 3300.0f; // Напряжение при максимальном угле (мВ)

    // Настройка делителя напряжения
    // Например, для R1=10 кОм, R2=15 кОм: (10 + 15) / 15 = 1.67
    config.use_voltage_divider = false;
    // .voltage_divider_factor = 1.67f,  // Для делителя напряжения 10кОм/15кОм

    // Настройка пина активации считывания
    config.use_enable_pin = true;                  // Использовать пин для активации схемы считывания
    config.enable_pin = SERVO_POSITION_ENABLE_PIN; // GPIO12 по умолчанию
    config.enable_level = false;                   // Активный уровень LOW

    ESP_LOGI(TAG, "Инициализация модуля чтения положения сервопривода");

    if (is_initialized)
    {
        ESP_LOGW(TAG, "Модуль уже инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    // Инициализация АЦП
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = SERVO_POSITION_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &config.adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка инициализации ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    // Настройка канала АЦП
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12, // 12-битное разрешение
        .atten = ADC_ATTEN_DB_12,    // Аттенюация 12dB для измерения до 3.3В
    };

    ret = adc_oneshot_config_channel(config.adc_handle, SERVO_POSITION_ADC_CHANNEL, &chan_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка настройки канала ADC: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(config.adc_handle);
        return ret;
    }

    // Инициализация калибровки АЦП
    adc_cali_handle_t adc_cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = SERVO_POSITION_ADC_UNIT,
        .chan = SERVO_POSITION_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK)
    {
        config.adc_cali_handle = adc_cali_handle;
        ESP_LOGI(TAG, "Калибровка АЦП успешно инициализирована");
    }
    else
    {
        ESP_LOGW(TAG, "Калибровка АЦП не поддерживается, будут использованы сырые значения");
        config.adc_cali_handle = NULL;
    }

    // Настройки пина активации
    if (config.use_enable_pin)
    {
        // Настраиваем GPIO для активации считывания
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config.enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};

        ret = gpio_config(&io_conf);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка настройки пина активации: %s", esp_err_to_name(ret));
            adc_oneshot_del_unit(config.adc_handle);
            return ret;
        }

        // Деактивируем считывание положения по умолчанию
        gpio_set_level(config.enable_pin, !config.enable_level);

        ESP_LOGI(TAG, "Настроен пин активации %d, уровень активации: %s",
                 config.enable_pin, config.enable_level ? "HIGH" : "LOW");
    }

    ESP_LOGI(TAG, "Модуль успешно инициализирован. Рабочий диапазон: %.1f-%.1f градусов",
             config.angle_min, config.angle_max);

    is_initialized = true;
    reading_enabled = false;
    return ESP_OK;
}

esp_err_t servo_position_set_reading_enabled(bool enabled)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    // Если текущее состояние уже соответствует запрашиваемому, ничего не делаем
    if (reading_enabled == enabled)
    {
        ESP_LOGD(TAG, "Считывание уже %s", enabled ? "активировано" : "деактивировано");
        return ESP_OK;
    }

    if (config.use_enable_pin)
    {
        // Устанавливаем соответствующий уровень для включения/выключения считывания
        gpio_set_level(config.enable_pin, enabled ? config.enable_level : !config.enable_level);
        ESP_LOGD(TAG, "%s пин считывания положения сервопривода",
                 enabled ? "Активирован" : "Деактивирован");

        // Даем схеме считывания время на стабилизацию при включении
        if (enabled)
        {
            vTaskDelay(pdMS_TO_TICKS(POSITION_READING_STABILIZATION_TIME_MS));
        }
    }

    reading_enabled = enabled;
    return ESP_OK;
}

esp_err_t servo_position_reader_get_raw(int *raw_value)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (raw_value == NULL)
    {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Если используется пин активации и считывание не активировано,
    // активируем схему перед измерением
    bool was_enabled = reading_enabled;
    if (config.use_enable_pin && !was_enabled)
    {
        esp_err_t ret = servo_position_set_reading_enabled(true);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    esp_err_t ret = adc_oneshot_read(config.adc_handle, SERVO_POSITION_ADC_CHANNEL, raw_value);

    // Если схема была активирована автоматически, деактивируем ее
    if (config.use_enable_pin && !was_enabled)
    {
        servo_position_set_reading_enabled(false);
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка чтения ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t servo_position_reader_get_voltage_mv(float *voltage_mv)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (voltage_mv == NULL)
    {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }

    int raw_value;
    esp_err_t ret = servo_position_reader_get_raw(&raw_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Используем калибровку АЦП, если она доступна
    if (config.adc_cali_handle != NULL)
    {
        int voltage;
        ret = adc_cali_raw_to_voltage(config.adc_cali_handle, raw_value, &voltage);
        if (ret == ESP_OK)
        {
            *voltage_mv = (float)voltage;
        }
        else
        {
            ESP_LOGW(TAG, "Ошибка преобразования калиброванного значения: %s", esp_err_to_name(ret));
            // Fallback к некалиброванному значению
            *voltage_mv = ((float)raw_value * 3300.0f) / ADC_MAX_VALUE;
        }
    }
    else
    {
        // Если калибровка недоступна, используем линейное преобразование
        *voltage_mv = ((float)raw_value * 3300.0f) / ADC_MAX_VALUE;
    }

    // Применяем коэффициент делителя, если он используется
    if (config.use_voltage_divider)
    {
        *voltage_mv *= config.voltage_divider_factor;
    }

    ESP_LOGD(TAG, "Напряжение: %.2f мВ (raw: %d)", *voltage_mv, raw_value);

    return ESP_OK;
}

esp_err_t servo_position_reader_get_angle(float *angle)
{
    if (!is_initialized)
    {
        ESP_LOGE(TAG, "Модуль не инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    if (angle == NULL)
    {
        ESP_LOGE(TAG, "Указатель на результат не может быть NULL");
        return ESP_ERR_INVALID_ARG;
    }

    float voltage_mv;
    esp_err_t ret = servo_position_reader_get_voltage_mv(&voltage_mv);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Линейное преобразование напряжения в угол
    float voltage_range = config.voltage_max - config.voltage_min;
    float angle_range = config.angle_max - config.angle_min;

    if (voltage_range <= 0 || angle_range <= 0)
    {
        ESP_LOGE(TAG, "Некорректный диапазон калибровки");
        return ESP_ERR_INVALID_STATE;
    }

    // Ограничение по минимальному напряжению
    if (voltage_mv < config.voltage_min)
    {
        voltage_mv = config.voltage_min;
    }

    // Ограничение по максимальному напряжению
    if (voltage_mv > config.voltage_max)
    {
        voltage_mv = config.voltage_max;
    }

    // Линейное преобразование: угол = угол_мин + (напряжение - напряжение_мин) * (угол_макс - угол_мин) / (напряжение_макс - напряжение_мин)
    *angle = config.angle_min +
             (voltage_mv - config.voltage_min) * angle_range / voltage_range;

    ESP_LOGD(TAG, "Угол: %.2f град. (напряжение: %.2f мВ)", *angle, voltage_mv);

    return ESP_OK;
}

esp_err_t servo_position_reader_deinit(void)
{
    if (!is_initialized)
    {
        ESP_LOGW(TAG, "Модуль не был инициализирован");
        return ESP_ERR_INVALID_STATE;
    }

    // Если считывание активировано, деактивируем его
    if (reading_enabled)
    {
        servo_position_set_reading_enabled(false);
    }

    // Если используется пин активации, сбрасываем его настройки
    if (config.use_enable_pin)
    {
        gpio_reset_pin(config.enable_pin);
    }

    // Освобождаем ресурсы калибровки АЦП
    if (config.adc_cali_handle != NULL)
    {
        esp_err_t ret = adc_cali_delete_scheme_curve_fitting(config.adc_cali_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Ошибка при освобождении ресурсов калибровки: %s", esp_err_to_name(ret));
        }
        config.adc_cali_handle = NULL;
    }

    esp_err_t ret = adc_oneshot_del_unit(config.adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка освобождения ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    is_initialized = false;
    reading_enabled = false;
    ESP_LOGI(TAG, "Модуль деинициализирован");

    return ESP_OK;
}