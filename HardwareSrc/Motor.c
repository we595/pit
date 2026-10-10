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


/*********************************闭环参数(原配置, 保留)********************* */
#define OPEN_LIMIT_X10   20     /* 3.0°  (开极限: <3.0° 强制回关) */
#define CLOSE_LIMIT_X10  1000    /* 96.0° (关极限: >96° 强制回开; 全关目标95°留1°余量) */
#define DEADBAND_X10     10      /* 死区(0.1°): 到位判断阈值, 10=1.0° (已接入 Motor_Control) */
#define INPOS_RELEASE_X10 10     /* 迟滞释放余量(0.1°): 到位后误差须超过 死区+此值 才重新驱动, 防回弹反复启停 */
#define MAX_SPEED        60
#define MIN_SPEED        50

/******************************阀门配置表(量程/功率, 可切换)****************************
 * Big   = 原阀门 : 工作 3~95°,  极限 3.0/96.0°,  速度 60/45
 * Small = 小阀   : 工作 4~100°, 极限 4.0/100.0°, 信号最大 40
 * 切换: msh 命令 ValveCfg Big|Small (运行时立即生效, 不存Flash)
 * 开机默认: VALVE_CFG_DEFAULT  0=Big原阀  1=Small小阀
 ****************************************************************************************/
#define VALVE_CFG_DEFAULT   1               /* 开机默认: 1=小阀(当前) */

static const tValveCfg_Type cfg_big   = { OPEN_LIMIT_X10, CLOSE_LIMIT_X10, 3,  95,  MAX_SPEED, 45 };
static const tValveCfg_Type cfg_small = { 40,             1050,            4,  105, 40,        40 };
/* 小阀当前: 目标钳位 4~105°(tgt_max已从100放开到105), 关侧极限105.0° */

static uint8_t g_cfg_sel = VALVE_CFG_DEFAULT;   /* 0=Big, 1=Small */
tValveCfg_Type g_valve_cfg;                     /* 当前生效配置(Valve_CfgApply 填充) */

/* 把选中的配置拷贝到 g_valve_cfg */
static void Valve_CfgApply(void)
{
    g_valve_cfg = (g_cfg_sel == 0) ? cfg_big : cfg_small;
}



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
    rt_thread_mdelay(200);

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
	static uint8_t  WasRunning;
	static uint8_t  LastWarned  = 0;   /* 缓存过期警告只打一次 */
	static uint8_t  InPosition  = 0;   /* 到位锁定(迟滞): 防机构回弹/打滑引起反复启停 */
	static uint16_t LastTargetDeg = 0xFFFF;  /* 记住上次目标, 变化时解除锁定 */
	static uint8_t  StuckLatched = 0;        /* 老化/卡滞故障锁定: 停机等待, 改目标后自动解除 */
	
   /*0.目标值钳位到当前阀门配置的安全区间, 防 g_target=0 之类异常值 */
   if (TargetAngle < g_valve_cfg.tgt_min_deg)  TargetAngle = g_valve_cfg.tgt_min_deg;
   if (TargetAngle > g_valve_cfg.tgt_max_deg) TargetAngle = g_valve_cfg.tgt_max_deg;

   /* 目标变化(含上电首次) → 解除到位/卡滞锁定 + 清运行峰值, 保证改目标后重新动作 */
   if (TargetAngle != LastTargetDeg)
   {
       LastTargetDeg = TargetAngle;
       InPosition   = 0;
       StuckLatched = 0;
       Pressure_RunPeakClear();
   }
	
   tAngleControl.Angle_DeadEnd = DEADBAND_X10; //死区=DEADBAND_X10(0.1°单位), 在死区内不再控制电机转动
   tAngleControl.Angle_Target_x10 = TargetAngle * 10;

 

   /*1. 角度不在这里读了: RS485 服务线程周期读角度并刷新缓存,
    *   主循环只读 tAngleControl 缓存(不碰485总线);
    *   缓存超过500ms没更新 = 编码器读不到 → 停电机, 不能用旧角度瞎控 */
   if (rt_tick_get() - tAngleControl.Angle_Tick > rt_tick_from_millisecond(500))
   {
       if (LastWarned == 0)
       {
           rt_kprintf("[Motor] angle cache stale, motor STOP\n");
           LastWarned = 1;
       }
       WasRunning = 0;
       MotorDirectionSet(Motor_Dir_Stop);
       MotorSpeedSet(0);
       return;
   }
   LastWarned = 0;

   //读完再取角度
   uint32_t Temp = tAngleControl.Angle_Current_x10;
   /*2.限位保护 */

   if (Temp > 1800)
    {
//        Temp = 3600 - Temp;
//        tAngleControl.Angle_Current_x10 = Temp;//考虑到绝对值编码器往0°转,不小心转过去变成大角度时候
	WasRunning = 1;                    /* 限位也在转, 标记 */
     MotorDirectionSet(Motor_Dir_CCW);
      MotorSpeedSet(g_valve_cfg.lim_spd);
      return;
    }

   if (tAngleControl.Angle_Current_x10 < g_valve_cfg.open_lim_x10)
   {
     WasRunning = 1;                    /* 限位也在转, 标记 */
     MotorDirectionSet(Motor_Dir_CCW);
      MotorSpeedSet(g_valve_cfg.lim_spd);
      return;
      
   }
   else if (tAngleControl.Angle_Current_x10 > g_valve_cfg.close_lim_x10)
   {
      WasRunning = 1;                   /* 限位也在转, 标记 */
      MotorDirectionSet(Motor_Dir_CW);
      MotorSpeedSet(g_valve_cfg.lim_spd);
      return;

   }
   
   /*3.计算误差*/
   int32_t Error = tAngleControl.Angle_Target_x10 - tAngleControl.Angle_Current_x10;
   tAngleControl.Angle_Error = Error;

   /*3.5 老化/卡滞故障已锁定: 保持停机等待(改目标后自动解除, 重新尝试) */
   if (StuckLatched)
   {
       MotorDirectionSet(Motor_Dir_Stop);
       MotorSpeedSet(0);
       return;
   }

    /*4.误差判断: 未到位用死区驱动; 到位后加迟滞(误差超 死区+INPOS_RELEASE_X10 才重新驱动),
     *  防止棘轮/液压机构停泵回弹(约一个齿)造成 "到位→滑出→再驱动" 反复跳 */
    {
        int32_t Threshold = tAngleControl.Angle_DeadEnd + (InPosition ? INPOS_RELEASE_X10 : 0);

        if (Error > Threshold)
        {
            if (InPosition)   /* 到位后又被拉开: 打印一行, 便于观察回弹/打滑幅度 */
                rt_kprintf("[Motor] re-drive: cur=%u.%u err=%d\n",
                           tAngleControl.Angle_Current_x10 / 10,
                           tAngleControl.Angle_Current_x10 % 10, (int)Error);
            InPosition = 0;
            MotorDirectionSet(Motor_Dir_CCW);
            MotorSpeedSet(g_valve_cfg.drive_spd);
            WasRunning = 1;
            /* 老化/卡滞检测: 正在驱动(未到位)但压力已≥130%标称 → 故障: 停机并锁定 */
            if (Pressure_StuckCheck())
            {
                StuckLatched = 1;
                WasRunning = 0;
                MotorDirectionSet(Motor_Dir_Stop);
                MotorSpeedSet(0);
            }
        }
        else if (Error < -Threshold)
        {
            if (InPosition)   /* 到位后又被拉开: 打印一行, 便于观察回弹/打滑幅度 */
                rt_kprintf("[Motor] re-drive: cur=%u.%u err=%d\n",
                           tAngleControl.Angle_Current_x10 / 10,
                           tAngleControl.Angle_Current_x10 % 10, (int)Error);
            InPosition = 0;
            MotorDirectionSet(Motor_Dir_CW);
            MotorSpeedSet(g_valve_cfg.drive_spd);
            WasRunning = 1;
            /* 老化/卡滞检测: 正在驱动(未到位)但压力已≥130%标称 → 故障: 停机并锁定 */
            if (Pressure_StuckCheck())
            {
                StuckLatched = 1;
                WasRunning = 0;
                MotorDirectionSet(Motor_Dir_Stop);
                MotorSpeedSet(0);
            }
        }
        else
        {
            /* 在死区(或迟滞带)内: 到位保持 */
            if (WasRunning == 1)
            {
                WasRunning = 0;
                rt_kprintf("Angle = %u.%u \n",
                tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);

                /* 寿命评估: 只有"全行程到位"(开到开极限/关到关极限2°内)才评估;
                 * 压力只在电机顶着极限运行期间存在, 停机即消失 → 用运行期间峰值 */
                if (tAngleControl.Angle_Current_x10 <= (uint32_t)(g_valve_cfg.open_lim_x10 + 20) ||
                    tAngleControl.Angle_Current_x10 >= (uint32_t)(g_valve_cfg.close_lim_x10 - 20))
                {
                    uint16_t pk1 = 0, pk2 = 0;
                    Pressure_LifeCheck();
                    Pressure_GetLastPeak(&pk1, &pk2);
                    rt_kprintf("P1peak = %u.%02u MPa  P2peak = %u.%02u MPa\n",
                               pk1 / 100, pk1 % 100, pk2 / 100, pk2 % 100);
                }
            }
            InPosition = 1;
            MotorDirectionSet(Motor_Dir_Stop);
            MotorSpeedSet(0);
        }
    }
   
   

}

/**********************************命令行函数*************************************************************/


static void Motor_ControlCmd(int WordNum , char *Word[])
{
	uint16_t Temp;
	
   if (WordNum != 2)
   {
       rt_kprintf("Input: MotorControl <%u--%u> ", g_valve_cfg.tgt_min_deg, g_valve_cfg.tgt_max_deg);
   
	   return;
   }
  
	Temp = atoi(Word[1]);
    if (Temp < g_valve_cfg.tgt_min_deg)  Temp = g_valve_cfg.tgt_min_deg;   /* 按当前阀门配置限制 */
    if (Temp > g_valve_cfg.tgt_max_deg) Temp = g_valve_cfg.tgt_max_deg;
   g_target= Temp;
        
    /* 不再从 shell 线程直接驱动电机: 只改目标, 由主循环 Motor_Control 执行 */
 rt_kprintf("Target = %u deg\n", g_target);   

}

//命令行
MSH_CMD_EXPORT_ALIAS(Motor_ControlCmd ,MotorControl ,"MotorControl <deg> set target");


/********************************阀门配置切换命令*****************************************/
static void Valve_CfgCmd(int WordNum, char *Word[])
{
    if (WordNum < 2)
    {
        rt_kprintf("Input: ValveCfg Big|Small   (now=%s: %u~%u deg, lim %u.%u/%u.%u, spd %u/%u)\n",
                   (g_cfg_sel == 0) ? "Big" : "Small",
                   g_valve_cfg.tgt_min_deg, g_valve_cfg.tgt_max_deg,
                   g_valve_cfg.open_lim_x10 / 10,  g_valve_cfg.open_lim_x10 % 10,
                   g_valve_cfg.close_lim_x10 / 10, g_valve_cfg.close_lim_x10 % 10,
                   g_valve_cfg.drive_spd, g_valve_cfg.lim_spd);
        return;
    }

    if      (rt_strcasecmp(Word[1], "Big") == 0)    g_cfg_sel = 0;
    else if (rt_strcasecmp(Word[1], "Small") == 0)  g_cfg_sel = 1;
    else
    {
        rt_kprintf("Input Err, Please Input: ValveCfg Big|Small\n");
        return;
    }

    Valve_CfgApply();
    rt_kprintf("[Cfg] %s: %u~%u deg, lim %u.%u/%u.%u, spd drive=%u lim=%u\n",
               (g_cfg_sel == 0) ? "Big" : "Small",
               g_valve_cfg.tgt_min_deg, g_valve_cfg.tgt_max_deg,
               g_valve_cfg.open_lim_x10 / 10,  g_valve_cfg.open_lim_x10 % 10,
               g_valve_cfg.close_lim_x10 / 10, g_valve_cfg.close_lim_x10 % 10,
               g_valve_cfg.drive_spd, g_valve_cfg.lim_spd);
}
MSH_CMD_EXPORT_ALIAS(Valve_CfgCmd ,ValveCfg ,"ValveCfg Big|Small switch valve config");


/*****************************************************************************************************
*函数名称;Motor_Init
*功能:上电先把当前角度传感器的角度初始化为目标角度

*******************************************************************************************************/
void Motor_Init(void)
{
    uint8_t ok = 0;

    Valve_CfgApply();   /* 上电装载默认阀门配置(运行时可 msh ValveCfg Big|Small 切换) */

    /* 0. 上电瞬间编码器可能还没完成初始化: 方向命令补发一次(0=CW递增) */
    RS485Send_DirectionSetOrder(0);
    rt_thread_mdelay(50);

    /* 1. 重试读角度: 编码器上电慢, 一次读不到不代表坏了 */
    for (uint8_t i = 0; i < 5; i++)
    {
        RS485Send_ReadSingleRingOrder();
        rt_thread_mdelay(50);
        if (RS485Recive_SingleRingValue())
        {
            ok = 1;
            break;
        }
    }

    if (ok)
    {
        /* 2. 目标=当前角度 → 上电不动; 钳位到当前阀门配置区间 */
        g_target = tAngleControl.Angle_Current_x10 / 10;
        if (g_target < g_valve_cfg.tgt_min_deg)  g_target = g_valve_cfg.tgt_min_deg;
        if (g_target > g_valve_cfg.tgt_max_deg) g_target = g_valve_cfg.tgt_max_deg;

        rt_kprintf("Motor_Init: angle = %u.%u, target = %u deg (hold)\n",
                   tAngleControl.Angle_Current_x10 / 10,
                   tAngleControl.Angle_Current_x10 % 10, g_target);
    }
    else
    {
        /* 3. 编码器完全没响应: 目标给45°, 闭环里连续读失败会自动停电机 */
        g_target = 45;
        rt_kprintf("Motor_Init: angle read FAIL, target = 45 deg\n");
    }
}







