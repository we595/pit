#ifndef __Init_H
#define __Init_H
#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>


void Init(void);
void DAC_SetValue(float VoltageIn);

uint8_t  RxFlag(void);
uint8_t  ADCFlag(void);


#endif


