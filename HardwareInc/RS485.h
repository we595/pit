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
uint8_t RS485Recive_ResetZeroOrderCheck(void);   /* 1=CRC通过, 0=失败/长度不足 */

void RS485Send_DirectionSetOrder(uint8_t dir);

void RS485Send_SetValueOrder(uint16_t value);
uint8_t RS485Recive_SetValueOrderCheck(void);    /* 1=CRC通过, 0=失败/长度不足 */

/* ---- USART2 帧接收事件(信号量), 由 USART2 IDLE 中断驱动 ---- */
void RS485_RxEventInit(void);    /* 初始化事件对象; 必须在使能 USART2 IDLE 中断之前调用 */
void RS485_RxFrameDone(void);    /* 仅由 USART2 IDLE 中断调用: 通知"一帧接收完成" */

/* ---- RS485 服务线程对外接口: 其他线程只允许通过这两个函数使用 USART2 ---- */
rt_err_t RS485_Post(T_rs485_op op, uint16_t val);
void RS485_ServiceInit(void);



#endif


