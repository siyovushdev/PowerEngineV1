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
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim4;
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

/* Аналоговые входы (PLC_NODE_AI_IN) */
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t           channel;    // ADC_CHANNEL_x
    float              scale;      // масштаб, если нужно переводить в физику
} HwAnalogInMap;

/* ШИМ выходы (PLC_NODE_PWM_OUT)
 * duty задаётся в % [0..100].
 * Мы конвертируем duty в CCRx таймера.
 */
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t           channel;    // TIM_CHANNEL_1 и т.д.
} HwPwmOutMap;

/* ТУТ ТЫ ДОЛЖЕН ЗАПОЛНИТЬ ПРАВИЛЬНО ПОД СВОЮ F411 BLACK PILL */

/* Пример (заглушка):
   DI0 -> PA0 (прямой)
   DI1 -> PA1 (инверсный)
*/
static const HwDiscreteInMap hw_di[] = {
        { GPIOA, GPIO_PIN_10, false },  // DI0
        { GPIOA, GPIO_PIN_9, false  },  // DI1
};

/* Пример:
   DO0 -> PA5 (светодиод/реле), прямой
   DO1 -> PB12, прямой
*/
static const HwDiscreteOutMap hw_do[] = {
        { GPIOB, GPIO_PIN_4, false },  // DO0
        { GPIOB, GPIO_PIN_3,false },  // DO1
};

/* Пример:
   AI0 -> ADC1 CH1
   AI1 -> ADC1 CH2
*/
static const HwAnalogInMap hw_ai[] = {
        { &hadc1, ADC_CHANNEL_0, 1.0f }, // AI0
        { &hadc1, ADC_CHANNEL_1, 1.0f }, // AI1
};

/* Пример:
   PWM0 -> TIM4 CH1
   PWM1 -> TIM4 CH2
   Внимание: таймер должен быть уже настроен в PWM режиме и запущен.
*/
static const HwPwmOutMap hw_pwm[] = {
        { &htim4, TIM_CHANNEL_1 }, // PWM0
        { &htim4, TIM_CHANNEL_2 }, // PWM1
};

/* Размеры таблиц */
static const uint16_t hw_di_count  = sizeof(hw_di)  / sizeof(hw_di[0]);
static const uint16_t hw_do_count  = sizeof(hw_do)  / sizeof(hw_do[0]);
static const uint16_t hw_ai_count  = sizeof(hw_ai)  / sizeof(hw_ai[0]);
static const uint16_t hw_pwm_count = sizeof(hw_pwm) / sizeof(hw_pwm[0]);

/* ------------------------------------------------------------------------- */
/*                     Локальные вспомогательные функции                     */
/* ------------------------------------------------------------------------- */

/* Простое блочное чтение одного канала АЦП.
 * Для продакшена ты можешь перейти на DMA или мультиканальный скан.
 * Сейчас — минимум, чтобы жило.
 */
static uint32_t adc_read_blocking(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    HAL_ADC_Stop(hadc);
    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES; // подбери при настройке ADC
    // Для F4 может потребоваться заполнить дополнительные поля, смотри CubeMX

    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10); // timeout 10 мс
    uint32_t v = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return v;
}

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
 * Должно возвращать аптайм в миллисекундах.
 * На STM32 HAL_GetTick() уже даёт миллисекунды после HAL_Init().
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

    uint32_t raw = adc_read_blocking(m->hadc, m->channel);

    // Если хочешь физику:
    // float volts = raw * m->scale;
    // Но ПЛК-движку достаточно raw. Он сам хранит .out.i/.out.f.
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

