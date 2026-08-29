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
/***********************************角度传感器*************************************************************/

typedef struct
{
  uint32_t Angle_Current_x10 ;
  uint32_t Angle_Target_x10 ;
	
   int32_t Angle_Error;	
	
   int16_t Angle_DeadEnd;	
}tAngleControl_Type;
extern volatile tAngleControl_Type tAngleControl;


#endif


