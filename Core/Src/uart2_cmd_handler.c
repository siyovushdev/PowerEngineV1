#include "uart2_cmd_handler.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include <stdint.h>
#define FRAME_MAX_PAYLOAD  1024u

//TX Handler
extern MessageBufferHandle_t s_mb;
extern SemaphoreHandle_t s_txDone;     // бинарный семафор "DMA complete"
extern UART_HandleTypeDef huart2;


//RX Handler
extern StreamBufferHandle_t g_uartRxStream;
static uint8_t  rx_buf[FRAME_MAX_PAYLOAD];

//RX Handler
void u2_start(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, FRAME_MAX_PAYLOAD);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

//TX Handler
BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to){
    if (!s_mb || !msg || len == 0) return pdFALSE;
    // MessageBuffer сам добавит заголовок длины и сохранит границы сообщения.
    size_t sent = xMessageBufferSend(s_mb, msg, len, to);
    return (sent == len) ? pdTRUE : pdFALSE;
}

//TX Handler
BaseType_t UartTx_SendFromISR(const void *msg, size_t len, BaseType_t *hpw){
    if (!s_mb || !msg || len == 0) return pdFALSE;
    size_t sent = xMessageBufferSendFromISR(s_mb, msg, len, hpw);
    return (sent == len) ? pdTRUE : pdFALSE;
}

//TX Handler
// Колбэк HAL — "DMA завершён"
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart != &huart2) return;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_txDone, &hpw);
    portYIELD_FROM_ISR(hpw);
}

//RX Handler
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size){
    if (huart != &huart2 || size == 0) goto rearm;
    BaseType_t xHPW = pdFALSE;
    // Слить пришедший кусок в StreamBuffer (без блокировок)
    (void)xStreamBufferSendFromISR(g_uartRxStream, rx_buf, size, &xHPW);

    // Если пробудили более приоритетную таску — уступаем
    portYIELD_FROM_ISR(xHPW);
    rearm:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, FRAME_MAX_PAYLOAD);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}





