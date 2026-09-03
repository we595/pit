#ifndef __RS485_H
#define __RS485_H
#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>

void RS485_SendText(uint8_t Data1);
void RS485_SendTextU3(uint8_t Data1);

void RS485_ReciveText(uint8_t Data1);
void RS485Send_ReadSingleRingOrder(void);
void RS485Recive_SingleRingValue(void);

void RS485Send_ResetZeroOrder(void);
void RS485Recive_ResetZeroOrderCheck(void);

void RS485Send_DirectionSetOrder(uint8_t dir);

void RS485Send_SetValueOrder(uint16_t value);
void RS485Recive_SetValueOrderCheck(void);



#endif


