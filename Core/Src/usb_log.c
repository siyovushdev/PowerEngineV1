#include "usb_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef USB_LOG_BUF_SZ
#define USB_LOG_BUF_SZ 2048
#endif

static uint8_t ring[USB_LOG_BUF_SZ];
static volatile size_t w = 0, r = 0;
static SemaphoreHandle_t mtx;
static TaskHandle_t txTask;

static size_t _space(void) {
    size_t rw = w, rr = r;
    return (rr <= rw) ? (USB_LOG_BUF_SZ - (rw - rr) - 1)
                      : (rr - rw - 1);
}
static void _push(const uint8_t* d, size_t n) {
    while (n--) {
        ring[w] = *d++;
        w = (w + 1) % USB_LOG_BUF_SZ;
    }
}
static size_t _pull(uint8_t* d, size_t maxn) {
    size_t cnt = 0;
    while ((r != w) && (cnt < maxn)) {
        *d++ = ring[r];
        r = (r + 1) % USB_LOG_BUF_SZ;
        cnt++;
    }
    return cnt;
}


static void UsbLog_Task(void* arg) {
    uint8_t chunk[256];
    for (;;) {
        // ждём уведомления или таймаут
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        size_t n;
        do {
            n = 0;
            xSemaphoreTake(mtx, portMAX_DELAY);
            n = _pull(chunk, sizeof(chunk));
            xSemaphoreGive(mtx);
            if (n) {
                while (CDC_Transmit_FS(chunk, n) == USBD_BUSY) {
                    vTaskDelay(1);
                }
            }
        } while (n);
    }
}

void UsbLog_Init(void) {
    mtx = xSemaphoreCreateMutex();
    xTaskCreate(UsbLog_Task, "UsbLogTx", 512, NULL, tskIDLE_PRIORITY+2, &txTask);
}

void UsbLog_PutFromISR(const char* s, size_t n) {
    if (!n) return;
    UBaseType_t prim = taskENTER_CRITICAL_FROM_ISR();
    size_t free = _space();
    if (n > free) n = free;               // без ожиданий — лишнее отбрасываем
    _push((const uint8_t*)s, n);
    taskEXIT_CRITICAL_FROM_ISR(prim);
    vTaskNotifyGiveFromISR(txTask, NULL); // разбудить TX-таску
}

void UsbLog_Put(const char* s, size_t n) {
    if (!n) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    size_t free = _space();
    if (n > free) n = free;           // обрезаем тихо (или можно дожидаться)
    _push((const uint8_t*)s, n);
    xSemaphoreGive(mtx);
}

void UsbLog_Puts(const char* s) {
    UsbLog_Put(s, strlen(s));
}

int UsbLog_printf(const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    UsbLog_Put(buf, (size_t)n);
    return n;
}
