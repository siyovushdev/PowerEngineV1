#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ================== Константы и типы ================== */
#define PLC_MAX_NODES 64          // подбери под свою память
#define PLC_CYCLE_MS_MIN 5        // защита от слишком частого цикла
#define PLC_CYCLE_MS_MAX 100      // защита от слишком редкого цикла

typedef enum {
    PLC_VAL_BOOL  = 0,  // Логическое значение: 0 или 1 (дискретные сигналы, кнопки, реле, флаги)
    PLC_VAL_INT   = 1,  // Целое число (обычно 32-битное): используется для счётчиков, порогов, параметров
    PLC_VAL_FLOAT = 2   // Вещественное число (float): аналоговые данные, температура, напряжение, коэффициенты
} PlcValueType;

typedef enum {
    PLC_NODE_CONST_BOOL = 0,  // Постоянное логическое значение (0 или 1), используется как источник TRUE/FALSE
    PLC_NODE_CONST_INT  = 1,  // Постоянное целое число (уставка, порог, коэффициент)
    PLC_NODE_DIGITAL_IN = 2,  // Дискретный вход (чтение GPIO, кнопки, датчики)
    PLC_NODE_DIGITAL_OUT= 3,  // Дискретный выход (управление GPIO, лампы, реле)
    PLC_NODE_AND2       = 4,  // Логическое "И" (выход = 1, если оба входа = 1)
    PLC_NODE_OR2        = 5,  // Логическое "ИЛИ" (выход = 1, если хотя бы один вход = 1)
    PLC_NODE_NOT        = 6,  // Логическое "НЕ" (инвертирует сигнал: 1→0, 0→1)
    PLC_NODE_SR         = 7,  // Защёлка (Set/Reset latch): сохраняет состояние между циклами
    PLC_NODE_TON        = 8,  // Таймер на включение (Delay ON): срабатывает после удержания входа заданное время
    PLC_NODE_TOFF       = 9,  // Таймер на выключение (Delay OFF): задерживает выключение после отпускания входа
    PLC_NODE_R_TRIG     = 10, // Импульс по фронту (Rising trigger): 1 цикл при переходе 0→1
    PLC_NODE_F_TRIG     = 11, // Импульс по спаду (Falling trigger): 1 цикл при переходе 1→0
    PLC_NODE_AI_IN      = 12, // Аналоговый вход (чтение АЦП, датчиков, потенциометров)
    PLC_NODE_COMPARE_GT = 13, // Сравнение "БОЛЬШЕ" (A > B)
    PLC_NODE_COMPARE_LT = 14, // Сравнение "МЕНЬШЕ" (A < B)
    PLC_NODE_PWM_OUT    = 15  // ШИМ-выход (PWM): формирование аналогового сигнала по коэффициенту заполнения
} PlcNodeType;


// ──────────────────────────────────────────────────────────────
// Флаг для поля flags (битовые настройки узла)
// ──────────────────────────────────────────────────────────────
#define PLC_NODE_FLAG_FORCE   (1u << 0)  // Разрешает принудительное (ручное) управление выходом узла

// ──────────────────────────────────────────────────────────────
// Основная структура описания одного узла (Node) в графе ПЛК
// ──────────────────────────────────────────────────────────────
typedef struct {
    // ────────────────────────────────
    // Статическая часть — приходит по CBOR при загрузке графа
    // ────────────────────────────────

    uint16_t    id;         // Уникальный индекс узла в графе (0..65535)
    uint8_t     type;       // Тип узла (см. PlcNodeType — AND, TON, PWM и т.д.)
    uint8_t     valueType;  // Тип данных (см. PlcValueType — BOOL, INT, FLOAT)

    int16_t     inA;        // Индекс узла-источника A; -1 = вход не подключен
    int16_t     inB;        // Индекс узла-источника B; -1 = вход не подключен

    int32_t     paramInt;   // Целочисленный параметр (порог, коэффициент, номер пина и т.п.)
    float       paramFloat; // Вещественный параметр (уставка, коэффициент масштабирования)
    uint32_t    paramMs;    // Параметр времени (мс) — для таймеров TON/TOFF
    uint32_t    flags;      // Битовые флаги поведения узла (например, PLC_NODE_FLAG_FORCE и др.)

    // ────────────────────────────────
    // Динамическая часть — формируется и изменяется во время работы
    // ────────────────────────────────

    union {
        bool    b;  // Логическое значение (для дискретных узлов)
        int32_t i;  // Целое значение (для счётчиков, арифметики)
        float   f;  // Вещественное значение (для аналоговых сигналов)
    } out;          // Текущее вычисленное значение узла (результат работы)

    // ────────────────────────────────
    // Внутреннее состояние для "памятных" блоков
    // ────────────────────────────────

    bool     sr_q;            // Хранит состояние для блока SR (Set/Reset latch)
    bool     trig_prev;       // Предыдущее состояние входа для R_TRIG / F_TRIG (детектор фронта)

    bool     ton_active;      // TON: активен ли отсчёт таймера
    uint32_t ton_accum_ms;    // TON: накопленное время при удержании входа

    bool     toff_holding;    // TOFF: в фазе удержания после отпускания входа
    uint32_t toff_left_ms;    // TOFF: сколько миллисекунд осталось до выключения

    // ────────────────────────────────
    // Принудительный (ручной) режим управления выходом
    // ────────────────────────────────

    bool     force_en;        // Включён ли ручной режим (игнорировать входы)
    bool     force_bool;      // Принудительное значение (0/1), удерживаемое в ручном режиме
    uint32_t force_left_ms;   // Сколько миллисекунд ещё держать принудительное состояние
} PlcNode;

typedef struct {
    uint32_t cycleMs;              // период цикла ПЛК (желаемый)
    uint16_t nodeCount;
    PlcNode  nodes[PLC_MAX_NODES];
} PlcGraph;

/* ================== Глобальные графы ================== */

extern PlcGraph g_activeGraph;
extern PlcGraph g_stagingGraph;

extern bool g_activeGraphValid;
extern bool g_stagingGraphValid;

extern volatile bool g_needSwapGraph;

/* ================== Команды из TaskComms -> TaskPlcScan ================== */

typedef enum {
    CMD_FORCE_OUTPUT = 1,
    CMD_SET_NODE_PARAM = 2,
    // можно расширять
} PlcCmdType;

// команда на ПЛК-движок
typedef struct {
    PlcCmdType type;
    uint16_t   nodeIndex; // к какому узлу относится (например DIGITAL_OUT)
    uint32_t   value;     // значение (например 1/0 или новое значение порога)
    uint32_t   holdMs;    // для FORCE_OUTPUT: сколько мс держать
} PlcCmd;

// Очередь команд
extern QueueHandle_t plcCmdQueue;

/* ================== Публичные функции ПЛК ================== */

// вызывается TaskComms при получении upload_graph
bool plc_upload_graph(const PlcGraph* src);

// вызывается TaskComms при получении activate_graph
bool plc_request_activate_graph(void);

// основной тик ПЛК, вызывается TaskPlcScan в цикле
void plc_tick(void);

// сервисные куски (ниже будут реализованы в plc.c)
void plc_refresh_inputs_hw(PlcGraph* g);
void plc_apply_pending_cmds(PlcGraph* g, uint32_t dt_ms);
void plc_graph_step(PlcGraph* g, uint32_t dt_ms);

// внутренние вспомогательные
void plc_graph_reset_runtime(PlcGraph* g);

bool plc_validate_graph(PlcGraph* g);

/* аппаратно-зависимые функции, которые ты реализуешь под свою плату */
bool     hw_read_discrete_input(int logicalCh);
void     hw_write_discrete_output(int logicalCh, bool val);
int32_t  hw_read_adc_input(int logicalCh);
void     hw_write_pwm_output(int logicalCh, int dutyPercent);

uint32_t now_ms(void);     // системное время мс
void     iwdg_feed(void);  // кормим watchdog

/* Задачи RTOS (реализуются в tasks.c) */
/* TaskPlcScan:
   Вызывает plc_tick().
   Делает DelayUntil по cycleMs activeGraph.
*/
void TaskPlcScan(void* arg);

void UartTx_Task(void *arg);  // создать один раз

/* TaskComms:
   Читает кадры по UART и передаёт в handle_frame_from_host().
*/
void TaskComms(void* arg);

/* Функция init_rtos() вызывается из main.c
   после HAL_Init, настройки GPIO/UART/ADC/TIM/IWDG:
   - создаёт plcCmdQueue
   - создаёт таски
   - запускает планировщик
*/
void init_rtos(void);


#ifdef __cplusplus
}
#endif