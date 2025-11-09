#include "plc.h"
#include <string.h> // memcpy

PlcGraph g_activeGraph;
PlcGraph g_stagingGraph;

bool g_activeGraphValid = false;
bool g_stagingGraphValid = false;
volatile bool g_needSwapGraph = false;

QueueHandle_t plcCmdQueue = NULL;

#define LOGF_PLC(fmt, ...)  printf("CBOR %s:%d " fmt "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

typedef struct {
    bool     stable;         // стабилизированное состояние, которое отдаём наружу
    bool     candidate;      // кандидат в новое состояние (последний сырый)
    uint32_t sinceMs;        // с какого времени держится candidate
    uint32_t debounceMs;     // порог стабилизации
    bool     inited;
} DiFilterState;

static DiFilterState di_filter[PLC_MAX_NODES];
static const uint32_t DI_DEBOUNCE_DEFAULT_MS = 30; // по умолчанию 30 мс

/* ================== Внутренние хелперы ================== */

static void plc_node_reset_runtime(PlcNode *n) {
    n->out.b = false;
    n->out.i = 0;
    n->out.f = 0.0f;

    n->sr_q = false;
    n->trig_prev = false;

    n->ton_active = false;
    n->ton_accum_ms = 0;

    n->toff_holding = false;
    n->toff_left_ms = 0;

    n->force_en = false;
    n->force_bool = false;
    n->force_left_ms = 0;
}

void plc_graph_reset_runtime(PlcGraph *g) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        plc_node_reset_runtime(&g->nodes[i]);
    }
}

bool plc_validate_graph(PlcGraph *g) {
    if (g->nodeCount == 0) return false;
    if (g->nodeCount > PLC_MAX_NODES) return false;

    if (g->cycleMs < PLC_CYCLE_MS_MIN) g->cycleMs = PLC_CYCLE_MS_MIN;
    if (g->cycleMs > PLC_CYCLE_MS_MAX) g->cycleMs = PLC_CYCLE_MS_MAX;

    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];

        if (n->id != i) return false;
        if (n->type > PLC_NODE_PWM_OUT) return false;
        if (n->valueType > PLC_VAL_FLOAT) return false;

        if (n->inA >= 0 && n->inA >= (int16_t) i) return false;
        if (n->inB >= 0 && n->inB >= (int16_t) i) return false;

        if (n->type == PLC_NODE_DIGITAL_IN && (n->inA != -1 || n->inB != -1)) return false;
        if (n->type == PLC_NODE_AI_IN && (n->inA != -1 || n->inB != -1)) return false;

        if (n->type == PLC_NODE_DIGITAL_OUT && (n->inA < 0)) return false;
        if (n->type == PLC_NODE_TON && (n->inA < 0)) return false;
        if (n->type == PLC_NODE_TOFF && (n->inA < 0)) return false;
        if (n->type == PLC_NODE_PWM_OUT && (n->inA < 0)) return false;

        if ((n->type == PLC_NODE_TON || n->type == PLC_NODE_TOFF) &&
            (n->paramMs > 600000u)) // 10 мин защита
        {
            return false;
        }
    }

    return true;
}

/* ================== API для TaskComms ================== */

bool plc_upload_graph(const PlcGraph *src) {
    // копируем во staging
    g_stagingGraph = *src;

    if (!plc_validate_graph(&g_stagingGraph)) {
        LOGF_PLC("Graph is not valid!");
        g_stagingGraphValid = false;
        return false;
    }

    LOGF_PLC("Graph is valid!");

    plc_graph_reset_runtime(&g_stagingGraph);
    LOGF_PLC("plc_graph_reset_runtime!");

    g_stagingGraphValid = true;
    //Временно для теста включим, а так это командой активации включить!
//    g_needSwapGraph = true;
    return true;
}

bool plc_request_activate_graph(void) {
    if (!g_stagingGraphValid) {
        return false;
    }
    g_needSwapGraph = true; // TaskPlcScan обработает
    return true;
}

/* ================== Основной тик ПЛК (вызов из TaskPlcScan) ================== */

void plc_tick(void) {
    static uint32_t lastMs = 0;
    uint32_t now = now_ms();
    uint32_t dt = now - lastMs;
    lastMs = now;

    // 1. Активация нового графа, если запрошено
    if (g_needSwapGraph && g_stagingGraphValid) {
        taskENTER_CRITICAL();
        {
            g_activeGraph = g_stagingGraph;
            g_activeGraphValid = true;
            g_needSwapGraph = false;
        }
        taskEXIT_CRITICAL();
    }

    // если нет активного графа - просто покормить watchdog и выйти
    if (!g_activeGraphValid) {
        iwdg_feed();
        return;
    }

    // 2. Снять входы
    plc_refresh_inputs_hw(&g_activeGraph);

    // 3. Применить входящие команды (force, смена порогов и т.д.)
    plc_apply_pending_cmds(&g_activeGraph, dt);

    // 4. Посчитать граф (логика, таймеры, выходы)
    plc_graph_step(&g_activeGraph, dt);

    // 5. Watchdog
    iwdg_feed();
}

/* ================== Реализация refresh_inputs/apply_cmds/graph_step ================== */

void plc_refresh_inputs_hw(PlcGraph *g) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];
        switch (n->type) {
            case PLC_NODE_DIGITAL_IN: {
                const uint32_t tnow = now_ms();
                bool raw = hw_read_discrete_input(n->paramInt);

                DiFilterState* fs = &di_filter[i];

                if (!fs->inited) {
                    fs->inited     = true;
                    fs->stable     = raw;
                    fs->candidate  = raw;
                    fs->sinceMs    = tnow;
                    // debounceMs уже установлен при свапе графа; если граф статичный — можно подстраховать:
                    if (fs->debounceMs == 0) fs->debounceMs = DI_DEBOUNCE_DEFAULT_MS;
                }

                if (raw != fs->candidate) {
                    // новый кандидат — перезапускаем таймер
                    fs->candidate = raw;
                    fs->sinceMs   = tnow;
                } else {
                    // держится та же кандидатура — проверяем время
                    if ((tnow - fs->sinceMs) >= fs->debounceMs && fs->stable != fs->candidate) {
                        fs->stable = fs->candidate;
                    }
                }

                n->out.b = fs->stable;
                break;
            }
            case PLC_NODE_AI_IN: {
                int32_t raw = hw_read_adc_input(n->paramInt);
                n->out.i = raw;
                n->out.f = (float) raw;
                break;
            }
            default:
                // остальные не трогаем
                break;
        }
    }
}

void plc_apply_pending_cmds(PlcGraph *g, uint32_t dt_ms) {
    PlcCmd cmd;
    while (xQueueReceive(plcCmdQueue, &cmd, 0) == pdTRUE) {

        switch (cmd.type) {
            case CMD_FORCE_OUTPUT:
                if (cmd.nodeIndex < g->nodeCount) {
                    PlcNode *n = &g->nodes[cmd.nodeIndex];
                    if (n->type == PLC_NODE_DIGITAL_OUT || n->type == PLC_NODE_PWM_OUT) {
                        n->force_en = true;
                        n->force_bool = (cmd.value != 0);
                        n->force_left_ms = cmd.holdMs;
                        n->flags |= PLC_NODE_FLAG_FORCE;
                    }
                }
                break;

            case CMD_SET_NODE_PARAM:
                if (cmd.nodeIndex < g->nodeCount) {
                    PlcNode *n = &g->nodes[cmd.nodeIndex];
                    // базовый вариант: просто кладём value в paramInt
                    n->paramInt = (int32_t) cmd.value;
                    // (при желании можно позволить менять paramMs, paramFloat и т.д.)
                }
                break;

            default:
                break;
        }
    }

    // таймер принудительного режима
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];
        if (n->force_en) {
            if (n->force_left_ms <= dt_ms) {
                n->force_en = false;
                n->flags &= ~PLC_NODE_FLAG_FORCE;
                n->force_left_ms = 0;
            } else {
                n->force_left_ms -= dt_ms;
            }
        }
    }
}

static bool get_bool(const PlcNode *n) { return n->out.b; }

static int32_t get_int(const PlcNode *n) { return n->out.i; }

static float get_float(const PlcNode *n) { return n->out.f; }

void plc_graph_step(PlcGraph *g, uint32_t dt_ms) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *self = &g->nodes[i];
        const PlcNode *a = (self->inA >= 0) ? &g->nodes[self->inA] : NULL;
        const PlcNode *b = (self->inB >= 0) ? &g->nodes[self->inB] : NULL;

        switch (self->type) {

            case PLC_NODE_CONST_BOOL:
                self->out.b = (self->paramInt != 0);
                break;

            case PLC_NODE_CONST_INT:
                self->out.i = self->paramInt;
                self->out.f = (float) self->paramInt;
                break;

            case PLC_NODE_DIGITAL_IN:
            case PLC_NODE_AI_IN:
                // уже обновлено в plc_refresh_inputs_hw
                break;

            case PLC_NODE_AND2:
                self->out.b = get_bool(a) && get_bool(b);
                break;

            case PLC_NODE_OR2:
                self->out.b = get_bool(a) || get_bool(b);
                break;

            case PLC_NODE_NOT:
                self->out.b = !get_bool(a);
                break;

            case PLC_NODE_SR: {
                bool setSig = get_bool(a);
                bool rstSig = get_bool(b);
                if (rstSig) {
                    self->sr_q = false;
                } else if (setSig) {
                    self->sr_q = true;
                }
                self->out.b = self->sr_q;
                break;
            }

            case PLC_NODE_TON: {
                bool inSig = get_bool(a);
                if (inSig) {
                    if (!self->ton_active) {
                        self->ton_active = true;
                        self->ton_accum_ms = 0;
                    } else {
                        if (self->ton_accum_ms < self->paramMs) {
                            self->ton_accum_ms += dt_ms;
                            LOGF_PLC("self->ton_accum_ms->%lu", self->ton_accum_ms);
                            if (self->ton_accum_ms > self->paramMs)
                                self->ton_accum_ms = self->paramMs;
                        }
                    }
                } else {
                    self->ton_active = false;
                    self->ton_accum_ms = 0;
                }

                self->out.b = (self->ton_accum_ms >= self->paramMs);
                break;
            }

            case PLC_NODE_TOFF: {

                bool inSig = get_bool(a);
                if (inSig) {
                    self->out.b = true;
                    self->toff_holding = true;
                    self->toff_left_ms = self->paramMs;
                } else {
                    if (self->toff_holding) {
                        if (self->toff_left_ms <= dt_ms) {
                            self->toff_left_ms = 0;
                            self->toff_holding = false;
                        } else {
                            self->toff_left_ms -= dt_ms;
                        }
                    }
                    self->out.b = self->toff_holding;
                }
                break;
            }

            case PLC_NODE_R_TRIG: {
                bool cur = get_bool(a);
                bool pulse = (cur && !self->trig_prev);
                self->out.b = pulse;
                self->trig_prev = cur;
                break;
            }

            case PLC_NODE_F_TRIG: {
                bool cur = get_bool(a);
                bool pulse = (!cur && self->trig_prev);
                self->out.b = pulse;
                self->trig_prev = cur;
                break;
            }

            case PLC_NODE_COMPARE_GT: {
                int32_t val = get_int(a);
                self->out.b = (val > self->paramInt);
                break;
            }

            case PLC_NODE_COMPARE_LT: {
                int32_t val = get_int(a);
                self->out.b = (val < self->paramInt);
                break;
            }

            case PLC_NODE_DIGITAL_OUT: {
                bool desired = get_bool(a);

                if (self->force_en && (self->flags & PLC_NODE_FLAG_FORCE)) {
                    desired = self->force_bool;
                }

                hw_write_discrete_output(self->paramInt, desired);

                self->out.b = desired;
                break;
            }

            case PLC_NODE_PWM_OUT: {
                int32_t duty = get_int(a);
                if (duty < 0) duty = 0;
                if (duty > 100) duty = 100;

                if (self->force_en && (self->flags & PLC_NODE_FLAG_FORCE)) {
                    duty = self->force_bool ? 100 : 0;
                }

                hw_write_pwm_output(self->paramInt, duty);

                self->out.i = duty;
                self->out.f = (float) duty;
                break;
            }

            default:
                // неизвестный тип
                break;
        }
    }
}