#ifndef __RS485_H
#define __RS485_H
#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>
#include "define.h"            /* T_rs485_op / T_rs485_msg: RS485_Post 参数类型 */

void RS485_SendText(uint8_t Data1);
void RS485_SendTextU3(uint8_t Data1);

void RS485_ReciveText(uint8_t Data1);
void RS485Send_ReadSingleRingOrder(void);
uint8_t RS485Recive_SingleRingValue(void);   /* 返回1=CRC通过并更新角度, 0=无应答/校验失败 */

void RS485Send_ResetZeroOrder(void);
void RS485Recive_ResetZeroOrderCheck(void);

void RS485Send_DirectionSetOrder(uint8_t dir);

void RS485Send_SetValueOrder(uint16_t value);
void RS485Recive_SetValueOrderCheck(void);

/* ---- RS485 服务线程对外接口: 其他线程只允许通过这两个函数使用 USART2 ---- */
rt_err_t RS485_Post(T_rs485_op op, uint16_t val);
void RS485_ServiceInit(void);



#endif


