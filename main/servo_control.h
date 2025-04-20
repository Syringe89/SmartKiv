#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "esp_err.h" // Для esp_err_t
// #include "freertos/queue.h" // <-- Убрано

// // Объявление хэндла очереди команд для серво - Убрано
// extern QueueHandle_t servo_cmd_queue;

// Объявление функции инициализации сервопривода
// Теперь не создает очередь
esp_err_t servo_init(void);

// Объявление задачи управления сервоприводом
// Теперь будет ждать уведомления задачи (Task Notification)
void servo_control_task(void *pvParameters);

// Объявление функции деинициализации сервопривода
esp_err_t servo_deinit(void);

#endif // SERVO_CONTROL_H
