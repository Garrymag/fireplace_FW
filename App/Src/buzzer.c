#include "buzzer.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "main.h"

/// Размер очереди команд buzzer
#define BUZZER_QUEUE_SIZE 4

/// Пин PC9 - Buzzer
#define BUZZER_PIN PC9

/// Дескриптор очереди FreeRTOS
static QueueHandle_t buzzer_queue = NULL;

/// Дескриптор задачи
static TaskHandle_t buzzer_task_handle = NULL;

/**
 * @brief Задача обработки buzzer
 */
static void buzzer_task(void* params) {
    BuzzerCommand cmd;
    
    (void)params;
    
    while (1) {
        // Ожидаем команду из очереди
        if (xQueueReceive(buzzer_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            // Цикл по количеству повторений
            for (uint8_t rep = 0; rep < cmd.repetitions; rep++) {
                // Включаем buzzer (HIGH - активный)
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
                
                // Ждём длительность сигнала
                vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));
                
                // Выключаем buzzer
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
                
                // Пауза между повторами (кроме после последнего)
                if (rep < cmd.repetitions - 1) {
                    vTaskDelay(pdMS_TO_TICKS(cmd.pause_ms));
                }
            }
        }
    }
}

void buzzer_init(void) {
    // Настройка пина PC9 на выход
    
    // Начальное состояние - buzzer выключен
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    
    // Создаём очередь команд
    buzzer_queue = xQueueCreate(BUZZER_QUEUE_SIZE, sizeof(BuzzerCommand));
    configASSERT(buzzer_queue != NULL);
    
    // Создаём задачу buzzer
    BaseType_t result = xTaskCreate(
        buzzer_task,
        "buzzer",
        256,           // Размер стека в словах
        NULL,
        2,             // Приоритет задачи
        &buzzer_task_handle
    );
    configASSERT(result == pdPASS);
}

int buzzer_play(const BuzzerCommand* cmd) {
    if (buzzer_queue == NULL || cmd == NULL) {
        return -1;
    }
    
    // Отправляем копию команды в очередь (неблокирующий вызов)
    return xQueueSend(buzzer_queue, cmd, 0) == pdTRUE;
}

int buzzer_play(uint16_t duration_ms, uint8_t repetitions, uint16_t pause_ms) {
    BuzzerCommand cmd = {
        .duration_ms  = duration_ms,
        .repetitions  = repetitions,
        .pause_ms     = pause_ms
    };
    return buzzer_play(&cmd);
}