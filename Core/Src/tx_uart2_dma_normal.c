#include "rx_to_idle_dma.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

// --- Конфиг очереди TX ---
#define U2_TXQ_CAP     8u          // кол-во блоков в очереди
#define U2_TXBLK_SZ    256u        // размер одного блока (большие сообщения режем на чанки)
#define U2_TX_SPLIT(len)  (((len) + U2_TXBLK_SZ-1)/U2_TXBLK_SZ)

static uint8_t  u2_tx_pool[U2_TXQ_CAP][U2_TXBLK_SZ];
static uint16_t u2_tx_len [U2_TXQ_CAP];

static volatile uint16_t u2_tx_head = 0;   // пишет продьюсер (enqueue)
static volatile uint16_t u2_tx_tail = 0;   // читает потребитель (DMA-complete)
static volatile bool     u2_tx_busy = false;
static volatile uint32_t u2_tx_drops = 0;  // счётчик дропов при переполнении очереди

static inline bool u2_txq_empty(void) { return u2_tx_head == u2_tx_tail; }
static inline bool u2_txq_full(void)  { return (uint16_t)((u2_tx_head + 1) % U2_TXQ_CAP) == u2_tx_tail; }

// Критическая секция: короткая, без HAL-вызовов
static inline void cs_enter(void){ __disable_irq(); }
static inline void cs_leave(void){ __enable_irq(); }

// Локальный запуск DMA (без проверок). ВЫЗЫВАТЬ ТОЛЬКО когда точно есть данные.
static void u2_tx_start_locked(void)
{
    // head/tail уже в консистентном состоянии
    uint16_t idx = u2_tx_tail;
    u2_tx_busy = true; // помечаем занято ДО запуска, чтобы параллельные enqueue не стартовали повторно
    // HAL можно вызывать с открытыми IRQ, но мы сейчас в крит. секции — лучше выходим:
    cs_leave();
    HAL_UART_Transmit_DMA(&huart2, u2_tx_pool[idx], u2_tx_len[idx]);
    // крит. секция восстановится у вызывающего кода, если нужно
    return;
}

// Публичная функция: поставить в очередь произвольный буфер (можно из IRQ)
bool U2_TxEnqueue(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return true;

    // Режем на чанки, каждый <= U2_TXBLK_SZ
    while (len) {
        uint16_t chunk = (len > U2_TXBLK_SZ) ? U2_TXBLK_SZ : len;

        cs_enter();
        if (u2_txq_full()) { cs_leave(); u2_tx_drops++; return false; }

        uint16_t idx = u2_tx_head;
        memcpy(u2_tx_pool[idx], data, chunk);
        u2_tx_len[idx] = chunk;
        u2_tx_head = (uint16_t)((idx + 1) % U2_TXQ_CAP);

        bool need_start = (!u2_tx_busy);
        cs_leave();

        if (need_start) {
            u2_tx_start_locked();   // запускаем DMA вне крит. секции
        }

        data += chunk;
        len  -= chunk;
    }
    return true;
}

bool u2_send_cbor_frame(const uint8_t* cbor, size_t len)
{
    if (!cbor || len == 0 || len > FRAME_MAX_PAYLOAD) return false;

    uint8_t hdr[2] = { (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };

    uint16_t crc = crc16_modbuss(hdr, 2);
    crc = crc16_modbus_update(crc, cbor, len);
    uint8_t tail[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };

    uint8_t tmp[2 + FRAME_MAX_PAYLOAD + 2];
    memcpy(&tmp[0], hdr, 2);
    memcpy(&tmp[2], cbor, len);
    memcpy(&tmp[2 + len], tail, 2);
    uint16_t total = (uint16_t)(2 + len + 2);
    return U2_TxEnqueue(tmp, total);
}



// Callback по окончанию DMA-передачи
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return;

    cs_enter();
    // Сдвигаем tail — текущий блок отправлен
    u2_tx_tail = (uint16_t)((u2_tx_tail + 1) % U2_TXQ_CAP);

    if (u2_tx_tail == u2_tx_head) {
        // Очередь пуста — освобождаем DMA
        u2_tx_busy = false;
        cs_leave();
        return;
    }
    // Есть ещё данные — запускаем следующий блок
    u2_tx_start_locked(); // сама выйдет из CS перед HAL-вызовом
}
