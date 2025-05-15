#ifndef SWITCH_DRIVER_H
#define SWITCH_DRIVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Типы функций для кнопок
typedef enum {
    SWITCH_ONOFF_TOGGLE_CONTROL = 0,
    SWITCH_LEVEL_UP_CONTROL,
    SWITCH_LEVEL_DOWN_CONTROL,
    SWITCH_CALIBRATION_CONTROL,
} switch_func_t;

// Пара: кнопка и её функция
typedef struct {
    uint8_t gpio_num;
    switch_func_t func;
} switch_func_pair_t;

// Вычисление размера массива пар кнопка-функция
#define PAIR_SIZE(pairs) (sizeof(pairs) / sizeof(pairs[0]))

// Обработчик событий кнопок
typedef void (*switch_event_handler_t)(switch_func_pair_t *);

// Инициализация драйвера кнопок
bool switch_driver_init(switch_func_pair_t *button_func_pairs, uint8_t pair_len, switch_event_handler_t handler);

// Проверка, нажата ли кнопка в данный момент
bool switch_driver_is_pressed(switch_func_pair_t *button);

#endif // SWITCH_DRIVER_H 