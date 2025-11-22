#include "plc.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * ВАЖНО:
 * Здесь мы только связываем логические каналы PLC (DI0, DO0, AI0, PWM0)
 * с конкретными ножками STM32F411 (GPIOA5 и т.д.).
 *
 * Никакой логики графа здесь нет.
 *
 * Этот файл можно править под свою разводку платы,
 * не ломая остальную систему.
 */

/* ----- Внешние HAL-хэндлы, которые уже должны быть созданы в проекте ----- */

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim1;   // добавлено для TIM1 (PA8, PA9)
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/*                            Таблицы привязки IO                            */
/* ------------------------------------------------------------------------- */

/* Дискретные входы (PLC_NODE_DIGITAL_IN)
 * paramInt узла = индекс в этом массиве.
 * inverted=true означает, что физически сигнал "активный низкий",
 * и нам надо инвертировать.
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    bool          inverted;
} HwDiscreteInMap;

/* Дискретные выходы (PLC_NODE_DIGITAL_OUT) */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    bool          inverted;
} HwDiscreteOutMap;

/* Аналоговые входы (PLC_NODE_AI_IN)
 * вместо hadc/channel оставляем индекс в g_adcRaw[]
 */
typedef struct {
    uint8_t bufIndex;   // индекс в g_adcRaw[]
    float   scale;      // масштаб (если понадобится)
} HwAnalogInMap;

/* ШИМ выходы (PLC_NODE_PWM_OUT)
 * duty задаётся в % [0..100].
 * Мы конвертируем duty в CCRx таймера.
 */
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t           channel;    // TIM_CHANNEL_1 и т.д.
} HwPwmOutMap;

/* ------------------------------------------------------------------------- */
/*                  Конкретная привязка под F411 Black Pill                  */
/* ------------------------------------------------------------------------- */

/* DI:
 * DI0 -> PA10
 * DI1 -> PB10
 * DI2 -> PB12
 * DI3 -> PB13
 * DI4 -> PB14
 * DI5 -> PB15
 *
 * Все прямые (inverted = false). Если где-то "активный низкий" — поменяешь.
 */
static const HwDiscreteInMap hw_di[] = {
        { GPIOA, GPIO_PIN_10, false },  // DI0
        { GPIOB, GPIO_PIN_10, false },  // DI1
        { GPIOB, GPIO_PIN_12, false },  // DI2
        { GPIOB, GPIO_PIN_13, false },  // DI3
        { GPIOB, GPIO_PIN_14, false },  // DI4
        { GPIOB, GPIO_PIN_15, false },  // DI5
};

/* DO:
 * DO0 -> PA4
 * DO1 -> PA15
 * DO2 -> PB2
 * DO3 -> PB3
 * DO4 -> PB4
 * DO5 -> PB5
 *
 * Все прямые. Если нужно "active-low" — выставишь inverted = true.
 */
static const HwDiscreteOutMap hw_do[] = {
        { GPIOA, GPIO_PIN_4,  false },  // DO0
        { GPIOA, GPIO_PIN_15, false },  // DO1
        { GPIOB, GPIO_PIN_2,  false },  // DO2
        { GPIOB, GPIO_PIN_3,  false },  // DO3
        { GPIOB, GPIO_PIN_4,  false },  // DO4
        { GPIOB, GPIO_PIN_5,  false },  // DO5
};

/* AI:
 * AI0 -> g_adcRaw[0] (ADC1_IN0, PA0)
 * AI1 -> g_adcRaw[1] (ADC1_IN1, PA1)
 * AI2 -> g_adcRaw[2] (ADC1_IN8, PB0)
 * AI3 -> g_adcRaw[3] (ADC1_IN9, PB1)
 */
static const HwAnalogInMap hw_ai[] = {
        { 0, 1.0f }, // AI0
        { 1, 1.0f }, // AI1
        { 2, 1.0f }, // AI2
        { 3, 1.0f }, // AI3
};

/* PWM:
 * PWM0 -> TIM1_CH1 (PA8)
 * PWM1 -> TIM4_CH1 (PB6)
 * PWM2 -> TIM4_CH2 (PB7)
 * PWM3 -> TIM1_CH2 (PA9)
 *
 * Таймеры должны быть настроены в PWM-режиме и запущены.
 */
static const HwPwmOutMap hw_pwm[] = {
        { &htim1, TIM_CHANNEL_1 }, // PWM0 -> PA8
        { &htim4, TIM_CHANNEL_1 }, // PWM1 -> PB6
        { &htim4, TIM_CHANNEL_2 }, // PWM2 -> PB7
        { &htim1, TIM_CHANNEL_2 }, // PWM3 -> PA9
};

/* Размеры таблиц */
static const uint16_t hw_di_count   = sizeof(hw_di)  / sizeof(hw_di[0]);
static const uint16_t hw_do_count   = sizeof(hw_do)  / sizeof(hw_do[0]);
static const uint16_t hw_ai_count   = sizeof(hw_ai)  / sizeof(hw_ai[0]);
static const uint16_t hw_pwm_count  = sizeof(hw_pwm) / sizeof(hw_pwm[0]);

/* ------------------------------------------------------------------------- */
/*                     Локальные вспомогательные функции                     */
/* ------------------------------------------------------------------------- */

/* Возвращает ARR таймера (AUTO-RELOAD), чтобы конвертировать duty% в CCR */
static inline uint32_t timer_get_arr(TIM_HandleTypeDef* htim)
{
    return htim->Instance->ARR;
}

/* Устанавливает CCRx таймера по каналу */
static inline void timer_set_ccr(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t value)
{
    switch (channel) {
        case TIM_CHANNEL_1: htim->Instance->CCR1 = value; break;
        case TIM_CHANNEL_2: htim->Instance->CCR2 = value; break;
        case TIM_CHANNEL_3: htim->Instance->CCR3 = value; break;
        case TIM_CHANNEL_4: htim->Instance->CCR4 = value; break;
        default: break;
    }
}

/* ------------------------------------------------------------------------- */
/*                  Реализации функций, объявленных в plc.h                  */
/* ------------------------------------------------------------------------- */

/* now_ms()
 * Возвращает аптайм в миллисекундах по тикам FreeRTOS.
 */
uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* iwdg_feed()
 * Кормим IWDG (сторожевой таймер).
 * hiwdg должен быть заранее настроен (CubeMX).
 */
void iwdg_feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* Чтение дискретного входа по логическому каналу.
 * Возвращает true/false.
 */
bool hw_read_discrete_input(int logicalCh)
{
    if (logicalCh < 0 || logicalCh >= (int)hw_di_count) {
        return false;
    }

    const HwDiscreteInMap* m = &hw_di[logicalCh];

    GPIO_PinState raw = HAL_GPIO_ReadPin(m->port, m->pin);
    bool v = (raw == GPIO_PIN_SET);

    if (m->inverted) {
        v = !v;
    }

    return v;
}

/* Запись дискретного выхода по логическому каналу.
 * Если inverted=true, то мы инвертируем перед записью,
 * чтобы внешняя логика всегда работала в терминах "true = включить".
 */
void hw_write_discrete_output(int logicalCh, bool val)
{
    if (logicalCh < 0 || logicalCh >= (int)hw_do_count) {
        return;
    }

    const HwDiscreteOutMap* m = &hw_do[logicalCh];

    bool phys = m->inverted ? !val : val;

    HAL_GPIO_WritePin(
            m->port,
            m->pin,
            phys ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

/* Чтение аналогового входа по логическому каналу.
 * Возвращаем сырое значение АЦП как int32_t.
 * Можно масштабировать (умножить на m->scale), если хочешь сразу "милливольты".
 */
int32_t hw_read_adc_input(int logicalCh)
{
    if (logicalCh < 0 || logicalCh >= (int)hw_ai_count) {
        return 0;
    }

    const HwAnalogInMap* m = &hw_ai[logicalCh];

    // чтение из DMA-буфера
    uint16_t raw = g_adcRaw[m->bufIndex];   // чтение 16 бит — атомарно на M4

    // если нужна шкала в будущем:
    // float v = (float)raw * m->scale;
    // return (int32_t)lroundf(v);

    return (int32_t)raw;
}

/* Установка ШИМ по логическому каналу.
 * dutyPercent [0..100].
 * Мы пересчитываем dutyPercent в CCRx таймера.
 */
void hw_write_pwm_output(int logicalCh, int dutyPercent)
{
    if (logicalCh < 0 || logicalCh >= (int)hw_pwm_count) {
        return;
    }

    const HwPwmOutMap* m = &hw_pwm[logicalCh];

    if (dutyPercent < 0)   dutyPercent = 0;
    if (dutyPercent > 100) dutyPercent = 100;

    uint32_t arr = timer_get_arr(m->htim);
    uint32_t ccr = (arr * (uint32_t)dutyPercent) / 100u;

    timer_set_ccr(m->htim, m->channel, ccr);
}

void plc_hw_reset_outputs(void)
{
    // --- Все дискретные выходы в 0 ---
    hw_write_discrete_output(0, false); // PA5
    hw_write_discrete_output(1, false); // PA6
    hw_write_discrete_output(2, false); // PA7
    hw_write_discrete_output(3, false); // PB0

    // --- Все PWM-каналы в 0% duty ---
    hw_write_pwm_output(0, 0); // PWM0 -> TIM1_CH1 PA8
    hw_write_pwm_output(1, 0); // PWM1 -> TIM4_CH1 PB6
    hw_write_pwm_output(2, 0); // PWM2 -> TIM4_CH2 PB7
    hw_write_pwm_output(3, 0); // PWM3 -> TIM1_CH2 PA9
}