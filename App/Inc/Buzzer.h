#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Структура команды для управления buzzer
typedef struct {
    uint16_t duration_ms;   /// Длительность сигнала в миллисекундах
    uint8_t  repetitions;   /// Число повторений
    uint16_t pause_ms;      /// Пауза между повторами в миллисекундах
} BuzzerCommand;

/**
 * @brief Инициализация buzzer на пине PC9
 */
void buzzer_init(void);

/**
 * @brief Отправка команды из параметров (удобная функция)
 */
int buzzer_play(uint16_t duration_ms, uint8_t repetitions, uint16_t pause_ms);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
