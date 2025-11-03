// Core/Src/adc.c
#include "adc.h"
volatile uint16_t g_adc_buf[ADC_DMA_LEN] = {0}; // единственное определение