#include "plc_comm.h"
#include "stm32f4xx_hal.h"
// tinycbor / твой CBOR-инклюд добавишь позже

extern UART_HandleTypeDef huart2;

/*
 * CRC16 MODBUS (0xA001 полином, начальное 0xFFFF)
 */
uint16_t crc16_modbus(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc = (crc >> 1);
            }
        }
    }
    return crc;
}

/*
 * uart_get_frame_blocking():
 *
 * Цель:
 * - Принять:
 *   2 байта длины (LE)
 *   payload (len байт)
 *   2 байта CRC16 (LE)
 *
 * - Проверить CRC16.
 * - Записать в out->len и out->payload.
 *
 * Сейчас даём заглушку, чтобы проект собрался и работал без связи.
 * Ты позже заменишь тело на нормальную реализацию через HAL_UART_Receive()
 * либо через свой Rx DMA / RxToIdle буфер.
 */
bool uart_get_frame_blocking(Frame* out)
{
    (void)out;
    // Заглушка: связи нет → говорим что новых кадров нет
    // Возвращаем false, чтобы TaskComms просто крутилась.
    return false;
}

/*
 * uart_send_frame():
 *
 * Упаковывает payload в формат [len][payload][crc] и отправляет по UART2.
 *
 * Сейчас есть рабочая реализация через HAL_UART_Transmit().
 * Если потом перейдёшь на DMA TX — адаптируешь.
 */
void uart_send_frame(const uint8_t* payload, uint16_t len)
{
    uint16_t crc = crc16_modbus(payload, len);

    uint8_t header[2];
    header[0] = (uint8_t)(len & 0xFF);
    header[1] = (uint8_t)((len >> 8) & 0xFF);

    HAL_UART_Transmit(&huart2, header, 2, 50);
    HAL_UART_Transmit(&huart2, (uint8_t*)payload, len, 50);

    uint8_t tail[2];
    tail[0] = (uint8_t)(crc & 0xFF);
    tail[1] = (uint8_t)((crc >> 8) & 0xFF);

    HAL_UART_Transmit(&huart2, tail, 2, 50);
}

/*
 * handle_frame_from_host():
 *
 * Здесь мы должны:
 * - разобрать CBOR (f->payload, f->len)
 * - прочитать поле "cmd"
 * - выполнить команду
 *   - upload_graph  -> plc_upload_graph()
 *   - activate_graph-> plc_request_activate_graph()
 *   - force_output  -> xQueueSend(plcCmdQueue,...)
 *   - status        -> send_status_reply()
 * - отправить ACK/ответ
 *
 * Пока делаем заглушку (ничего не делаем), чтобы сборка прошла.
 */
void handle_frame_from_host(const Frame* f)
{
    (void)f;
    // TODO: CBOR parse и вызовы plc_upload_graph(), plc_request_activate_graph(),
    //       отправка send_*_ack(), постановка PlcCmd в plcCmdQueue.
}

/*
 * send_upload_graph_ack(), send_activate_graph_ack(), send_force_output_ack():
 *
 * Эти функции должны сформировать небольшой CBOR или хотя бы простой payload,
 * и отправить через uart_send_frame().
 *
 * Пока делаем минимальные заглушки.
 */

void send_upload_graph_ack(bool ok)
{
    uint8_t buf[4];
    buf[0] = 'U';
    buf[1] = 'P';
    buf[2] = ok ? 1 : 0;
    buf[3] = 0;
    uart_send_frame(buf, 4);
}

void send_activate_graph_ack(bool ok)
{
    uint8_t buf[4];
    buf[0] = 'A';
    buf[1] = 'C';
    buf[2] = ok ? 1 : 0;
    buf[3] = 0;
    uart_send_frame(buf, 4);
}

void send_force_output_ack(bool ok)
{
    uint8_t buf[4];
    buf[0] = 'F';
    buf[1] = 'O';
    buf[2] = ok ? 1 : 0;
    buf[3] = 0;
    uart_send_frame(buf, 4);
}

/*
 * send_status_reply():
 *
 * Должно собрать snapshot текущего графа:
 *  - uptimeMs
 *  - cycleMs
 *  - для каждого узла: id, type, out, ton_accum_ms, toff_left_ms и т.д.
 *
 * Потом упаковать в CBOR и отослать.
 *
 * Пока делаем простейшую заглушку.
 */
void send_status_reply(void)
{
    uint8_t buf[16];
    // простой текстовый ответ "STAT" + аптайм младшие 4 байта
    uint32_t t = now_ms();
    buf[0] = 'S';
    buf[1] = 'T';
    buf[2] = 'A';
    buf[3] = 'T';
    buf[4] = (uint8_t)(t & 0xFF);
    buf[5] = (uint8_t)((t >> 8) & 0xFF);
    buf[6] = (uint8_t)((t >> 16) & 0xFF);
    buf[7] = (uint8_t)((t >> 24) & 0xFF);

    // остальное нулями
    for (int i = 8; i < 16; i++) buf[i] = 0;

    uart_send_frame(buf, 16);
}
