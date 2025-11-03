#include "FreeRTOS.h"
#include "task.h"

/*
 * Память для Idle Task
 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t  **ppxIdleTaskStackBuffer,
                                    uint32_t     *pulIdleTaskStackSize )
{
    // TCB и стек idle таска живут в .bss, статически
    static StaticTask_t idleTaskTCB;
    static StackType_t  idleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer   = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = &idleTaskStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

/*
 * Память для Timer Service Task (если configUSE_TIMERS == 1)
 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t  **ppxTimerTaskStackBuffer,
                                     uint32_t     *pulTimerTaskStackSize )
{
    static StaticTask_t timerTaskTCB;
    static StackType_t  timerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer   = &timerTaskTCB;
    *ppxTimerTaskStackBuffer = &timerTaskStack[0];
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}
