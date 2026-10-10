#ifndef __DEFINE_H
#define __DEFINE_H

#include <board.h>                 /* 板级: 芯片头文件 + 引脚宏(GET_PIN) */
#include <rtthread.h>                /* 加: RT-Thread 内核(rt_kprintf 等) */ 
#include <rtdevice.h>              /* 设备框架: pin/serial/adc 等设备 API */
#include <stdlib.h>
#include <stm32l4xx_hal.h>
#include <finsh.h> 

/**************************************跨文件调用************************************************/
extern uint16_t g_target;//main函数定义


/************************************ADC与压力传感器****************************************************/
typedef struct
{
  uint16_t ADC_DR_IN3;       
  uint16_t ADC_DR_IN13;
  uint16_t ADC_DR_VBAT;  /* 内部VBAT通道：测VCC_3.3电源（内部1/3分压，读数×3才是真实电压） */
  uint16_t ADC_Average;
	
  uint16_t ADC_DR_IN3Average;       
  uint16_t ADC_DR_IN13Average;
	
  uint16_t ADC_ZeroIN3;  
  uint16_t ADC_ZeroIN13; 
	
  uint16_t ADC_Index;
  uint16_t ADC_FilterIN3;
  uint16_t ADC_FilterIN13;
  
  uint16_t ADC_NumIN3;
  uint16_t ADC_NumIN13;	
	
  uint32_t ADC_Sum32_IN3;	
  uint32_t ADC_Sum32_IN13;
	
  uint8_t ADC_ZeroDone;	
}tDMA_Type;
extern volatile tDMA_Type tDMA;

typedef struct
{
  uint16_t Pressure1_IN3;
  uint16_t Pressure2_IN13;	


}tPressure_Type;
extern volatile tPressure_Type tPressure;


typedef enum
{
  Pressure_Normal    =            0,     //正常
  Pressure_Rusting   =            1,     //锈蚀
  Pressure_Leak      =            2,     //泄露
  Pressure_Overload  =            3      //过载 	

}enPress_State_Type;
extern enPress_State_Type  enPress_State;

typedef struct
{
   uint16_t Peak1_X100;         //峰值1压力
   uint16_t Peak2_X100;         //峰值2压力	
   uint32_t CycleCount;       /* 动作次数 */
    uint32_t OverloadCount;    /* 过载次数 */
    uint8_t  AlarmFlag; 

}tLife_Type;
extern  volatile tLife_Type tLife;
/************************************TX,RX*******************************************************/
//U2
typedef struct
{
  uint8_t Data[16];
  uint8_t Index;	
}tUsart2_RX_Type;
extern volatile tUsart2_RX_Type tUsart2_RX;

typedef struct
{
  uint8_t Data[16];
  uint8_t Index;	

}tUsart2_TX_Type;
extern volatile tUsart2_TX_Type tUsart2_TX;

//U3
typedef struct
{
  uint8_t Data[16];
  uint8_t Index;
  uint32_t Tick; 	
}tUsart3_RX_Type;
extern volatile tUsart3_RX_Type tUsart3_RX;

typedef struct
{
  uint8_t Data[16];
  uint8_t Index;	

}tUsart3_TX_Type;
extern volatile tUsart3_TX_Type tUsart3_TX;

/***********************************角度传感器*************************************************************/

typedef struct
{
  uint32_t Angle_Current_x10 ;
  uint32_t Angle_Target_x10 ;
	
   int32_t Angle_Error;	
	
   int16_t Angle_DeadEnd;	
  
   uint32_t Angle_Tick;     /* 最近一次成功解析角度的 tick(供 Read 命令判断缓存新鲜度) */	
}tAngleControl_Type;
extern volatile tAngleControl_Type tAngleControl;



/*********************************RS485消息队列枚举变量以及结构体******************************************/
typedef enum
{
    RS485_OP_NONE   = 0,
    RS485_OP_READ   = 1,      /* (可选) 立即读一次角度 */
    RS485_OP_SETVAL = 2,      /* 设置当前值, 参数放 val: raw 0~4095 */
    RS485_OP_RESET  = 3,      /* 复位零点, 无参数 */
    RS485_OP_DIRSET = 4,      /* 设递增方向, 参数放 val: 0=CW / 1=CCW */
} T_rs485_op;

typedef struct
{
    T_rs485_op op;            /* 要做什么: 命令码 */
    uint16_t   val;           /* 参数: 对应用上面注释的含义 */
    /* 以后需要"等结果"时再加:  rt_sem_t done; */
} T_rs485_msg;

#endif


