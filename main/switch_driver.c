#include "switch_driver.h"
#include "esp_log.h"

static const char *TAG = "SWITCH_DRIVER";

// Глобальные переменные для хранения состояния кнопок
static switch_func_pair_t *g_button_func_pairs = NULL;
static uint8_t g_pair_len = 0;
static switch_event_handler_t g_button_event_handler = NULL;
static TaskHandle_t g_switch_task_handle = NULL;
static bool g_is_button_pressed = false;

// Таймаут дребезга контактов в миллисекундах
#define DEBOUNCE_TIME_MS 50

// Задача для обработки событий кнопок
static void switch_task(void *arg)
{
    // Массив для хранения предыдущего состояния кнопок
    bool *previous_state = (bool *)malloc(g_pair_len * sizeof(bool));
    if (previous_state == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for button states");
        vTaskDelete(NULL);
        return;
    }
    
    // Инициализация массива предыдущих состояний
    for (int i = 0; i < g_pair_len; i++) {
        previous_state[i] = gpio_get_level(g_button_func_pairs[i].gpio_num) == 0;
    }
    
    while (1) {
        for (int i = 0; i < g_pair_len; i++) {
            // Проверяем текущее состояние кнопки
            bool current_state = gpio_get_level(g_button_func_pairs[i].gpio_num) == 0;
            
            // Если состояние изменилось
            if (current_state != previous_state[i]) {
                // Небольшая задержка для устранения дребезга контактов
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
                
                // Повторно проверяем состояние после задержки
                current_state = gpio_get_level(g_button_func_pairs[i].gpio_num) == 0;
                
                // Если состояние действительно изменилось
                if (current_state != previous_state[i]) {
                    previous_state[i] = current_state;
                    g_is_button_pressed = current_state;
                    
                    // Вызываем обработчик событий
                    if (g_button_event_handler != NULL) {
                        g_button_event_handler(&g_button_func_pairs[i]);
                    }
                }
            }
        }
        
        // Задержка между проверками
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Освобождаем память
    free(previous_state);
    vTaskDelete(NULL);
}

// Инициализация драйвера кнопок
bool switch_driver_init(switch_func_pair_t *button_func_pairs, uint8_t pair_len, switch_event_handler_t handler)
{
    if (button_func_pairs == NULL || pair_len == 0 || handler == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for switch driver initialization");
        return false;
    }
    
    // Сохраняем параметры
    g_button_func_pairs = button_func_pairs;
    g_pair_len = pair_len;
    g_button_event_handler = handler;
    
    // Настраиваем GPIO для кнопок
    for (int i = 0; i < pair_len; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << button_func_pairs[i].gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        
        gpio_config(&io_conf);
    }
    
    // Создаем задачу для обработки событий кнопок
    BaseType_t task_created = xTaskCreate(
        switch_task,        // Функция задачи
        "switch_task",      // Имя задачи
        4096,               // Размер стека
        NULL,               // Параметры
        5,                  // Приоритет
        &g_switch_task_handle // Хэндл задачи
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create switch task");
        return false;
    }
    
    ESP_LOGI(TAG, "Switch driver initialized successfully");
    return true;
}

// Проверка, нажата ли кнопка в данный момент
bool switch_driver_is_pressed(switch_func_pair_t *button)
{
    if (button == NULL) {
        return false;
    }
    
    // Читаем текущее состояние кнопки
    // Активный низкий уровень (0 = нажата, 1 = не нажата)
    return gpio_get_level(button->gpio_num) == 0;
} 