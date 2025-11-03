#include "rx_to_idle_dma.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

static uint8_t  rx_buf[FRAME_MAX_PAYLOAD];
static uint8_t  acc_buf[FRAME_MAX_PAYLOAD];
static uint16_t acc_len = 0;     // сколько уже накопили в acc_buf
static uint16_t acc_need = 0;    // сколько ещё байт ждём (payload + CRC, без первых 2-х байт LEN)
static uint8_t  len_lo = 0, len_hi = 0;
static bool     have_len_lo = false;

void u2_start(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, FRAME_MAX_PAYLOAD);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

//// --- Локальный помощник: отправка чанка через TX DMA (Normal)
//static void U2_TxDma(const uint8_t* data, size_t len)
//{
//    if (len == 0) return;
//
//    // Если текущий TX DMA занят — упростим и просто проигнорируем (или можно сделать очередь)
//    if (huart2.gState == HAL_UART_STATE_BUSY_TX) return;
//
//    // NB: источник данных должен жить до конца DMA. Копируем в локальный tmp, если нужно.
//    if (len > BUFFER_SIZE) len = BUFFER_SIZE;
//    memcpy(u2_echo_tmp, data, len);
//    HAL_UART_Transmit_DMA(&huart2, u2_echo_tmp, (uint16_t)len);
//}

static inline void acc_reset(void){
    acc_len = 0;
    acc_need = 0;
    len_lo = len_hi = 0;
    have_len_lo = false;
}

/* CRC-16/Modbus: poly=0xA001 (rev 0x8005), init=0xFFFF, xorout=0x0000, ref in/out */
uint16_t crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len){
    while (len--){
        crc ^= (uint16_t)(*data++);
        for (int i=0; i<8; ++i){
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc = (crc >> 1);
        }
    }
    return crc;
}

uint16_t crc16_modbuss(const uint8_t *data, size_t len){
    return crc16_modbus_update(0xFFFF, data, len);
}

//uint16_t crc16_modbus2(const uint8_t* data, uint16_t len)
//{
//    uint16_t crc = 0xFFFF;
//    for (uint16_t i = 0; i < len; i++) {
//        crc ^= data[i];
//        for (uint8_t b = 0; b < 8; b++) {
//            if (crc & 1)
//                crc = (crc >> 1) ^ 0xA001;
//            else
//                crc = (crc >> 1);
//        }
//    }
//    return crc;
//}

/* Подаём входящие байты порциями любого размера */
static void process_bytes(const uint8_t* p, uint16_t n){
    while (n--){
        uint8_t b = *p++;

        /* Шаг 1: читаем 2 байта длины (LE) */
        if (acc_need == 0 && !have_len_lo){
            len_lo = b;
            have_len_lo = true;
            continue;
        }
        if (acc_need == 0 && have_len_lo && len_hi == 0){  // читаем второй байт длины
            len_hi = b;

            uint16_t payload_len = (uint16_t)len_lo | ((uint16_t)len_hi << 8);
            if ((payload_len == 0) || payload_len > FRAME_MAX_PAYLOAD){
                acc_reset();
                continue; // ресинхронизация
            }
            acc_len  = 0;
            acc_need = payload_len + 2u; // ждём payload (+ CRC)
            continue;
        }

        /* Шаг 2: копим payload (+CRC, если включён) */
        if (acc_need){
            if (acc_len < sizeof(acc_buf)){
                acc_buf[acc_len++] = b;
            } else {
                acc_reset(); // защита на случай рассинхрона
                continue;
            }

            if (--acc_need == 0){
                uint16_t payload_len = acc_len - 2u;
                /* Разбираем CRC из хвоста */
                uint16_t crc_rx = (uint16_t)acc_buf[payload_len] |
                                  ((uint16_t)acc_buf[payload_len + 1] << 8);

                /* Считаем CRC по [LEN_LO, LEN_HI] + payload */
                uint8_t hdr[2] = { len_lo, len_hi };
                uint16_t crc = crc16_modbuss(hdr, 2);
                crc = crc16_modbus_update(crc, acc_buf, payload_len);

                if (crc != crc_rx){
                    acc_reset(); // CRC ошибка → отбросить кадр
                    continue;
                }

                /* Успех: отдаём чистый payload в CBOR-парсер */
                on_cbor_frame_received(acc_buf, payload_len);
//                U2_TxEnqueue(acc_buf, payload_len);   // безопасно и из IRQ (RxEventCallback)
                acc_reset();
            }
        } else {
            /* Если сюда попали — рассинхрон. Сброс. */
            acc_reset();
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size){
    if (huart != &huart2 || size == 0) goto rearm;
    process_bytes(rx_buf, size);
    rearm:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, FRAME_MAX_PAYLOAD);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

