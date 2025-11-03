#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "plc.h"

/*
* Структура кадра, который мы принимаем из UART.
* Формат физически такой:
*   [len_lo][len_hi][payload...len байт...][crc_lo][crc_hi]
*
* payload = CBOR-карта вида:
*   {"cmd":"status"} или
*   {"cmd":"upload_graph","graph":{...}} и т.д.
*/

typedef struct {
    uint16_t len;
    uint8_t  payload[256]; // лимит задаёшь сам
} Frame;

/* CRC16 (MODBUS-style) */
uint16_t crc16_modbus(const uint8_t* data, uint16_t len);

/* UART низкоуровневые операции */
bool uart_get_frame_blocking(Frame* out); // ждать 1 полный кадр
void uart_send_frame(const uint8_t* payload, uint16_t len);

/* Обработка входящего кадра */
void handle_frame_from_host(const Frame* f);

/* Ответы/ACK, которые уходят хосту */
void send_upload_graph_ack(bool ok);
void send_activate_graph_ack(bool ok);
void send_force_output_ack(bool ok);
void send_status_reply(void);


#ifdef __cplusplus
}
#endif