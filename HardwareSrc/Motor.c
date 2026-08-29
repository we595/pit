#include <board.h>                 /* 板级: 芯片头文件 + 引脚宏(GET_PIN) */
#include <rtthread.h>                /* 加: RT-Thread 内核(rt_kprintf 等) */ 
#include <rtdevice.h>              /* 设备框架: pin/serial/adc 等设备 API */
#include <stdlib.h>
#include <stm32l4xx_hal.h>
#include <finsh.h>                 /*是 finsh/msh 命令框架的头文件,注册命令的那套东西*/
#include "init.h"
#include "define.h"
#include "RS485.h"
#include "motor.h"
#include "Pressure.h"


/*********************************闭环参数********************* */
#define OPEN_LIMIT_X10   20     /* 2.0° */
#define CLOSE_LIMIT_X10  950    /* 95.0° */
#define DEADBAND_X10     5      /* 0.5° */
#define MAX_SPEED        60
#define MIN_SPEED        50



/******************************方向引脚定义****************************************/
#define Motor_Pin_PWR          GET_PIN(D,12)
#define Motor_Pin_CW           GET_PIN(D,13)    //开 绿色 角度变小
#define Motor_Pin_CCW          GET_PIN(D,14)  //关   白色
//开阀门记为0°,关阀门90°
//0到97°极限位置,实际运行时候开的极限位置不能到达0°到3°停,97°这边,在94°停即可
//信号给到55
/**********************************************************************************/

/********************************方向定义*************************************************/
//#define Motor_Dir_Stop               0
//#define Motor_Dir_CW                 1
//#define Motor_Dir_CCW                2
/*****************************************************************************************/

/*****************************************************************************************
*函数名称：Motor_Gpio_Init
*功能    ：初始化电机要用到的gpio口
*****************************************************************************************/
void Motor_Gpio_Init(void)
{
    /*输出模式*/ 
    rt_pin_mode(Motor_Pin_PWR, PIN_MODE_OUTPUT);
    rt_pin_mode(Motor_Pin_CW, PIN_MODE_OUTPUT);
    rt_pin_mode(Motor_Pin_CCW, PIN_MODE_OUTPUT);
    
    /**初始化先低电平 */
    rt_pin_write(Motor_Pin_PWR, PIN_LOW);
    rt_pin_write(Motor_Pin_CW, PIN_LOW);
    rt_pin_write(Motor_Pin_CCW, PIN_LOW);
}

/******************************************************************************************
*函数名称：MotorDirectionSet
*功能    ：设置电机方向
*形参    ：外部给函数传来方向指令，函数内部做出方向判断
*******************************************************************************************/
void MotorDirectionSet(uint8_t Dir)
{
    static uint8_t LastDir = 0xFF;   /* 记住上次方向, 0xFF=初始未设 */

    if (Dir == LastDir)              /* 方向没变: 继电器不动作, 直接返回 */
        return;

    /* 方向真的变了才拉低等待(换向) */
    rt_pin_write(Motor_Pin_CW, PIN_LOW);
    rt_pin_write(Motor_Pin_CCW, PIN_LOW);
    rt_thread_mdelay(500);

    /* 方向选择 */
    if (Dir == Motor_Dir_CW)
    {
        rt_pin_write(Motor_Pin_CW, PIN_HIGH);   /* CW 高 */
        rt_pin_write(Motor_Pin_CCW, PIN_LOW);
    }
    else if (Dir == Motor_Dir_CCW)
    {
        rt_pin_write(Motor_Pin_CCW, PIN_HIGH);  /* CCW 高 */
        rt_pin_write(Motor_Pin_CW, PIN_LOW);
    }
    else /* Motor_Dir_Stop: 已经全部拉低 */
    {
    }

    LastDir = Dir;
}


/***************************************************************************************
*函数名称：MotorSpeedSet
*功能    ：设置电机转速
*****************************************************************************************/
void MotorSpeedSet(int8_t Speed)
{
	float Temp;
	
	//转速就设置在0-100%区间
   if (Speed < 0)  Speed = 0;
   else if	(Speed > 100) Speed = 100;

	
   //传出信号
   Temp = ((float)Speed / 100.0f) * 3.3f;
   DAC_SetValue(Temp); 	

}



/*********************************************命令测试***********************************/
static void Motor_Cmd(int WordNum , char *Word[])
{
	int8_t Temp;
	
  if(WordNum < 2)
  {
	  rt_kprintf("Input: Motor (CW|CCW|STOP) Speed<0-100> ");
	  return;
  }

  if(WordNum == 3)
  {
    if(rt_strcasecmp(Word[1],"CW") == 0)
	{
		
	  MotorDirectionSet(Motor_Dir_CW);
		
	  Temp =(int8_t)atoi(Word[2]);
	  MotorSpeedSet(Temp);
		
	 	
      
	rt_kprintf("MotorSpeedCW:%d \n" ,Temp )	;	
	}
	else if(rt_strcasecmp(Word[1] , "CCW") == 0)
     {
	   MotorDirectionSet(Motor_Dir_CCW);
		 
	   Temp = (int8_t)atoi(Word[2]);
        MotorSpeedSet(Temp);
		 
	 		 
        rt_kprintf("MotorSpeedCCW:%d \n" ,Temp );		 
	 }
     
  
  }
  else if(WordNum == 2) 
  {
     if (rt_strcasecmp(Word[1] , "STOP") == 0) 
	 {
	    MotorDirectionSet(Motor_Dir_Stop);
		 
		 Temp = 0;
		DAC_SetValue(Temp);

        rt_kprintf("Motor STOP \n");		 
	 }
	 else 
	 {
	   rt_kprintf("Input: Motor (CW|CCW|STOP) Speed<0-100> ");
	  return;
	 }
  }	  

}

//加入命令行
MSH_CMD_EXPORT_ALIAS(Motor_Cmd ,Motor ,"Input: Motor (CW|CCW|STOP) Speed<0-100>");


/*********************************************************
 * 函数名称:Motor_Control
 * 作用    :电机闭环控制函数
 * 备注    :在主循环里调用,每次调用都会根据当前角度
            当前控制策略开关控制
 *********************************************************/
void Motor_Control(uint16_t TargetAngle)
{
	static uint8_t WasRunning;
	
   tAngleControl.Angle_DeadEnd =5; //死区5,0.5°，在死区内不再控制电机转动
   tAngleControl.Angle_Target_x10 = TargetAngle * 10;

 

   /*1.读下当前角度值并且记录到结构体内*/
   RS485Send_ReadSingleRingOrder();   /* 1. 发读单圈命令,并将角度记录 */
   rt_thread_mdelay(50);              /* 2. 等应答（传感器14ms应答+收完）*/
   RS485Recive_SingleRingValue();     /* 3. 解析 + CRC */
   
   //读完再取角度
   uint32_t Temp = tAngleControl.Angle_Current_x10;
   /*2.限位保护 */

   if (Temp > 1800)
    {
        Temp = 3600 - Temp;
        tAngleControl.Angle_Current_x10 = Temp;//考虑到绝对值编码器往0°转,不小心转过去变成大角度时候
    }

   if (tAngleControl.Angle_Current_x10 < OPEN_LIMIT_X10)
   {
     WasRunning = 1;                    /* 限位也在转, 标记 */
     MotorDirectionSet(Motor_Dir_CCW);
      MotorSpeedSet(45);
      return;
      
   }
   else if (tAngleControl.Angle_Current_x10 > CLOSE_LIMIT_X10)
   {
      WasRunning = 1;                   /* 限位也在转, 标记 */
      MotorDirectionSet(Motor_Dir_CW);
      MotorSpeedSet(45);
      return;

   }
   
   /*3.计算误差*/
   int32_t Error = tAngleControl.Angle_Target_x10 - tAngleControl.Angle_Current_x10;
   tAngleControl.Angle_Error = Error;

    /*4.判断误差是否在死区内*/
    if (Error > tAngleControl.Angle_DeadEnd)
    {
      MotorDirectionSet(Motor_Dir_CCW);
        MotorSpeedSet(60);
       WasRunning = 1;
    }
    else if (Error < -tAngleControl.Angle_DeadEnd)
    {
      MotorDirectionSet(Motor_Dir_CW);
        MotorSpeedSet(60);
        WasRunning = 1;
    }
    else
    {
		//转到角度检测一次压力值
		if(WasRunning == 1)
		{
		
		Pressure_LifeCheck(tPressure.Pressure1_IN3, tPressure.Pressure2_IN13);
		WasRunning = 0;
		//转到角度,打印角度值
        rt_kprintf("Angle = %u.%u \n",
        tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);	 
	//转到角度,打印压力值
		rt_kprintf("P1 = %u.%02u MPa  P2 = %u.%02u MPa\n",
               tPressure.Pressure1_IN3 / 100, tPressure.Pressure1_IN3 % 100,
               tPressure.Pressure2_IN13 / 100, tPressure.Pressure2_IN13 % 100);
		}
		
    		
      MotorDirectionSet(Motor_Dir_Stop);
        MotorSpeedSet(0);
		
    }
   
   

}

/**********************************命令行函数*************************************************************/


static void Motor_ControlCmd(int WordNum , char *Word[])
{
	uint16_t Temp;
	
   if (WordNum != 2)
   {
       rt_kprintf("Input: MotorControl <3--94> ");
   
	   return;
   }
  
	Temp = atoi(Word[1]);
    if (Temp < 4)  Temp = 4;                   /* 范围限制 */
    if (Temp > 94) Temp = 94;
    
   g_target= Temp;
        
    Motor_Control(g_target);
 rt_kprintf("Target = %u deg\n", g_target);   

}

//命令行
MSH_CMD_EXPORT_ALIAS(Motor_ControlCmd ,MotorControl ,"Input: MotorControl<3--94>");










