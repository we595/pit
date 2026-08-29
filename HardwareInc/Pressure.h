#ifndef __PRESSURE_H
#define __PRESSURE_H

#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>

void ADC_Filter(void);
void ADC_ZeroAdjust(void);



void Pressure_LifeCheck (uint16_t P1 ,  uint16_t P2);
void Press_Init(void);




#endif


