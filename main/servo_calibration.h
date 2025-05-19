#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Константы для АЦП измерения тока
#define SERVO_CURRENT_ADC_UNIT ADC_UNIT_1  // ESP32-H2 имеет только ADC_UNIT_1
#define SERVO_CURRENT_ADC_CHANNEL ADC_CHANNEL_3  // Измените на фактический канал
#define SERVO_CURRENT_ADC_ATTEN ADC_ATTEN_DB_12  // Максимальный диапазон (до 3.3В)

// Настройки калибровки
#define SERVO_CALIBRATION_STEP_SIZE 2        // Шаг изменения положения сервопривода (градусы)
#define SERVO_CALIBRATION_DELAY_MS 100       // Задержка между шагами (мс)
#define SERVO_CALIBRATION_SAMPLES 5          // Количество проб тока для усреднения
#define SERVO_CALIBRATION_THRESHOLD_FACTOR 3.0f // Множитель для определения блокировки (2х от базового тока)
#define SERVO_CALIBRATION_BUTTON CONFIG_GPIO_INPUT_IO_WAKEUP // Кнопка калибровки
#define SERVO_CALIBRATION_BUTTON_PRESS_DURATION 3000 // Длительность нажатия для начала калибровки (мс)

// Структура для хранения результатов калибровки
typedef struct {
    float min_angle;          // Минимальный угол (нижняя граница)
    float max_angle;          // Максимальный угол (верхняя граница)
    uint32_t min_duty;        // Минимальное значение скважности PWM для min_angle
    uint32_t max_duty;        // Максимальное значение скважности PWM для max_angle
    float normal_current;     // Ток при обычном движении (мА)
    float stall_current;      // Ток при блокировке (мА)
    bool is_calibrated;       // Флаг успешной калибровки
} servo_calibration_data_t;

/**
 * @brief Инициализация модуля калибровки сервопривода
 * 
 * @return esp_err_t ESP_OK при успешной инициализации
 */
esp_err_t servo_calibration_init(void);

/**
 * @brief Запуск калибровки сервопривода
 * 
 * @param calibration_data Указатель на структуру для сохранения результатов калибровки
 * @return esp_err_t ESP_OK при успешной калибровке
 */
esp_err_t servo_calibration_start(servo_calibration_data_t *calibration_data);

/**
 * @brief Получение текущего значения тока сервопривода
 * 
 * @param current_ma Указатель для сохранения измеренного тока в мА
 * @param auto_deinit Флаг автоматической деинициализации АЦП после измерения
 * @return esp_err_t ESP_OK при успешном измерении
 */
esp_err_t servo_calibration_get_current(float *current_ma, bool auto_deinit);

/**
 * @brief Сохранение калибровочных данных во флеш-память
 * 
 * @param calibration_data Данные калибровки для сохранения
 * @return esp_err_t ESP_OK при успешном сохранении
 */
esp_err_t servo_calibration_save_data(const servo_calibration_data_t *calibration_data);

/**
 * @brief Загрузка калибровочных данных из флеш-памяти
 * 
 * @param calibration_data Указатель на структуру для загрузки данных
 * @return esp_err_t ESP_OK при успешной загрузке
 */
esp_err_t servo_calibration_load_data(servo_calibration_data_t *calibration_data);

/**
 * @brief Деинициализация модуля калибровки
 * 
 * @return esp_err_t ESP_OK при успешной деинициализации
 */
esp_err_t servo_calibration_deinit(void);

/**
 * @brief Функция мониторинга нажатия кнопки
 *        Обрабатывает как короткие нажатия (сброс Zigbee),
 *        так и длительные нажатия (калибровка сервопривода)
 * 
 * @param pvParameters Параметры задачи
 */
void button_handler_task(void *pvParameters);

#endif // SERVO_CALIBRATION_H 