#include "plc.h"
#include "plc_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"




void init_rtos(void)
{
    plcCmdQueue = xQueueCreate(8, sizeof(PlcCmd));

    xTaskCreate(TaskPlcScan, "PLC",   512, NULL, tskIDLE_PRIORITY+3, NULL);
//    xTaskCreate(TaskComms,   "COMMS", 768, NULL, tskIDLE_PRIORITY+2, NULL);

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
    Frame f;
    for (;;) {
        if (uart_get_frame_blocking(&f)) {
            handle_frame_from_host(&f);
        }
        // если uart_get_frame_blocking() блокирует сама внутри - ок,
        // если нет, можно добавить vTaskDelay(1)
    }
}
