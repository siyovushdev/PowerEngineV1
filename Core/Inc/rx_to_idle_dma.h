#pragma once

#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
extern UART_HandleTypeDef huart2;

#define FRAME_MAX_PAYLOAD  1024u

void u2_start(void);
bool U2_TxEnqueue(const uint8_t* data, uint16_t len);
/* Отправка готового CBOR пакета (payload -> COBS -> 0x00 -> TX DMA) */
bool u2_send_cbor_frame(const uint8_t* cbor, size_t len);

/* Колбэк верхнего уровня: сюда падает ДЕКОДИРОВАННЫЙ CBOR-пакет */
void on_cbor_frame_received(const uint8_t* buf, size_t len);

uint16_t crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len);
uint16_t crc16_modbuss(const uint8_t *data, size_t len);
//uint16_t crc16_modbus2(const uint8_t* data, uint16_t len)

#ifdef __cplusplus
}
#endif

