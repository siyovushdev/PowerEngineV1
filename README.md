# STM32F411 PLC Runtime (FreeRTOS + CBOR + UART)

- Граф-движок ПЛК: узлы DI/DO/AI/TON/TOFF/SR/R_TRIG/F_TRIG/AND/OR/NOT/COMPARE/MUX2/PWM + TP/HYST/SCALE/ADD/LIMIT/PID/ANALOG_AVG/RAMP
- Протокол: CBOR + CRC16 (upload_graph / activate_graph / status / force_output)
- Persist: CBOR в Flash (CRC32 header)
- Задачи FreeRTOS: TaskPlcScan, TaskComms
- Пример тест-графов: HYST, TP, SCALE+LIMIT, PID (SP/PV), AVG+RAMP

## Сборка
- CLion + arm-none-eabi-gcc, FreeRTOS
- Target: STM32F411 Black Pill

## Быстрый тест
- Залить прошивку
- Отправить `upload_graph` → `activate_graph`
- Команда `status` для мониторинга (PID: u, PV, SP)
