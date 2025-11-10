#include "plc.h"
#include "plc_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "message_buffer.h"
#include "semphr.h"

#define UART_STREAM_SIZE     1024u
#define UART_TRIGGER_LEVEL   1u      // можно 1: пробуждать таск на любом байте

#ifndef UART_MB_SIZE
#define UART_MB_SIZE   1024  // кольцо сообщений в RAM
#endif
#ifndef UART_STAGING_SZ
#define UART_STAGING_SZ 1028  // линейный буфер под один DMA-транш
#endif

StreamBufferHandle_t g_uartRxStream;
MessageBufferHandle_t s_mb;
SemaphoreHandle_t s_txDone;
extern UART_HandleTypeDef huart2;

static uint8_t s_staging[UART_STAGING_SZ];

void init_rtos(void)
{
    plcCmdQueue = xQueueCreate(8, sizeof(PlcCmd));
    g_uartRxStream = xStreamBufferCreate(UART_STREAM_SIZE, UART_TRIGGER_LEVEL);
    s_mb = xMessageBufferCreate(UART_MB_SIZE);
    s_txDone = xSemaphoreCreateBinary();
    configASSERT(g_uartRxStream != NULL);

    xTaskCreate(TaskPlcScan, "PLC",   1024, NULL, tskIDLE_PRIORITY+3, NULL);
    xTaskCreate(TaskComms,   "COMMS", 768, NULL, tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(UartTx_Task, "uart_tx", 256, NULL, tskIDLE_PRIORITY+2, NULL);

    vTaskStartScheduler();
}

void TaskPlcScan(void* arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        plc_tick();

        uint32_t periodMs =
                g_activeGraphValid ? g_activeGraph.cycleMs : 10u;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
    }
}


void TaskComms(void* arg)
{
    uint8_t tmp[128];

    for (;;) {
        size_t got = xStreamBufferReceive(g_uartRxStream, tmp, sizeof(tmp), portMAX_DELAY);
        if (got > 0) {
            process_bytes_in_task(tmp, got);
        }
    }
}

// Единственный TX-таск
void UartTx_Task(void *arg){
    (void)arg;
    for(;;){
        // 1) Получаем одно сообщение целиком
        size_t n = xMessageBufferReceive(s_mb, s_staging, sizeof(s_staging), portMAX_DELAY);
        if (n == 0) continue;

        // 2) Отправляем через DMA; если сообщение длиннее staging — режем на куски
        size_t off = 0;
        while (off < n){
            size_t chunk = n - off;
            if (chunk > sizeof(s_staging)) chunk = sizeof(s_staging);

            // Если кусок < staging (обычно так), у нас уже в буфере; если > — копируйте по месту по необходимости.
            // Здесь простой путь: отправляем имеющийся [off:off+chunk] напрямую.
            if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)((&s_staging[0]) + off), chunk) != HAL_OK){
                // Если UART занят/ошибка — можно подождать и повторить
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            // Ждём завершение DMA без блокировок других задач
            xSemaphoreTake(s_txDone, portMAX_DELAY);
            off += chunk;
        }
        // цикл вернётся за следующим сообщением
    }
}
