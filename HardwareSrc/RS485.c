#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>
#include "motor.h"
#include "define.h"
#include "RS485.h"
#include "init.h"
#include "Modbus.h"

/* ★纪律(USART2=编码器总线):
 *   1. 本文件的 RS485Send_* / RS485Recive_* / RS485_SendText 只允许 rs485_thread_entry 调用;
 *   2. 其他线程要发命令 → RS485_Post(); 要读角度 → 读 tAngleControl 缓存;
 *   3. 开机初始化(Motor_Init 等)必须在 RS485_ServiceInit() 之前完成. */

#define RS485_MQ_NUM        8
#define RS485_THREAD_STACK  1024        /* 跑一段时间后用 list_thread 看实际用量再调 */
#define RS485_THREAD_PRIO   9           /* 必须高于主线程(10): 发帧期间不能被主循环抢占,
                                           否则帧内字节间隙>1.5字符时间, Modbus 帧会被从站丢弃 */
#define RS485_THREAD_TICK   10

static struct rt_messagequeue g_mq;          /* ① 调度室: 控制块 */
/* 每槽实际开销 = 对齐后消息大小 + 4字节链表节点, 多留4字节最保险 */
static rt_uint8_t g_mq_pool[RS485_MQ_NUM * (sizeof(T_rs485_msg) + 4)];/* ② 仓库: 真正存数据 */





/******************************************************************************* 
 * 函数名称:RS485_Send
 * 功能   :从机mcu的数据发送出去
 *备注发送是u2的
*********************************************************************************/
void RS485_SendText(uint8_t Data1)
{
    while((USART2->ISR & USART_ISR_TXE) == 0);  //等待发送寄存器空
   tUsart2_TX.Data[tUsart2_TX.Index] = Data1;  //将数据存入发送缓冲区
   USART2->TDR =tUsart2_TX.Data[tUsart2_TX.Index] ;  //将数据写入发送寄存器,开始发送
   tUsart2_TX.Index++;  //发送缓冲区索引加1
  if (tUsart2_TX.Index >= sizeof(tUsart2_TX.Data))  //防止数组越界
  {
    tUsart2_TX.Index = 0;
  }

}

/******************************************************************************* 
 * 函数名称:RS485_Send
 * 功能   :从机mcu的数据发送出去
 *备注发送是u2的
*********************************************************************************/
void RS485_SendTextU3(uint8_t Data1)
{
    while((USART3->ISR & USART_ISR_TXE) == 0);  //等待发送寄存器空
   tUsart3_TX.Data[tUsart3_TX.Index] = Data1;  //将数据存入发送缓冲区
   USART3->TDR =tUsart3_TX.Data[tUsart3_TX.Index] ;  //将数据写入发送寄存器,开始发送
   tUsart3_TX.Index++;  //发送缓冲区索引加1
  if (tUsart3_TX.Index >= sizeof(tUsart3_TX.Data))  //防止数组越界
  {
    tUsart3_TX.Index = 0;
  }

}



/********************************************************************************************
*函数名称 :Rs485Send_ReadSingleRingOrder
*作用      :发送读单圈指令
*调用      :调用RS485_SendText,发送读单圈指令
*********************************************************************************************/
void RS485Send_ReadSingleRingOrder(void)
{
	//先清空接收缓冲区
	 tUsart2_RX.Index = 0;
	
  uint8_t cmd[] = {0x01, 0x03,0x00, 0x00, 0x00, 0x01, 0x84, 0x0a};
  
  for (uint8_t i = 0; i<8 ; i++)
  {
    RS485_SendText(cmd[i]);
  
  }

}

/********************************************************************************************
*函数名称:RS485Recive_SingleRingValue
*作用    :检查编码器发来的单圈数据
*备注    :单圈命令发出去,回传的是7字节
*备注    :从站(编码器)回传过来的数据除了crc校验的俩字节是按小端发过来的其余是大端
因此在拼接数据时候不要搞错
********************************************************************************************/
uint8_t RS485Recive_SingleRingValue(void)
{

  if (tUsart2_RX.Index >= 7)
  {
	  //crc校验
     uint16_t CRC_Calculate = CRC16_Modbus(tUsart2_RX.Data, 5);
	 uint16_t CRC_Receive =((uint8_t)tUsart2_RX.Data[tUsart2_RX.Index - 2]) |((uint16_t)tUsart2_RX.Data[tUsart2_RX.Index - 1]<<8);
      
     if(CRC_Calculate  == CRC_Receive)	
	 {
	 //拼接角度数据
	   uint16_t raw = ((uint16_t)tUsart2_RX.Data[3]<<8) | (uint16_t)tUsart2_RX.Data[4];
       uint32_t Angle = (uint32_t) raw * 3600UL / 4096UL; 
    //拼接完角度后将角度存进控制结构体内部
       tAngleControl.Angle_Current_x10 = Angle;
       tAngleControl.Angle_Tick = rt_tick_get();   /* 记录更新时间, 供 Read 命令读缓存 */
	   
	   return 1;                      /* 成功 */
	 }

      /* 失败: 不打印(主循环每60ms调用一次会刷屏), 由调用方计数处理 */
      return 0;		  
	  
  
  }
  
  else
  {
  
    return 0;   /* 无应答 */
  
  }

//rt_thread_mdelay(200);  

}

/********************************************************************************************
*函数名称 :RS485Send_ResetZeroOrder
*作用      :发送复位零点指令
*调用      :调用RS485_SendText,发送复位零点指令,此时当前编码器值为0
*********************************************************************************************/
void RS485Send_ResetZeroOrder(void)
{
  //先清空接收缓冲区
   tUsart2_RX.Index = 0;
  
uint8_t cmd[8];
cmd[0]=0x01; cmd[1]=0x06; cmd[2]=0x00; cmd[3]=0x08; cmd[4]=0x00; cmd[5]=0x01;
uint16_t crc = CRC16_Modbus(cmd, 6);   /* 根据当前内容重新算 CRC */
cmd[6] = crc & 0xFF;
cmd[7] = (crc >> 8) & 0xFF;
  
  for (uint8_t i = 0; i<8 ; i++)
  {
    RS485_SendText(cmd[i]);
  
  }

}

/***************************************************************************************
*函数作用:发送命令,用来置零当前角度值
****************************************************************************************/
static void RS485Send_ResetZeroAdjust(int WordNum ,char *Word[])
{

	
	if (WordNum < 2)
	{
	   rt_kprintf("Input: Sensor Reset ");
	  return;
	}
	else if(WordNum == 2)
	{
         if(rt_strcasecmp(Word[1],"reset") == 0)
	   {
		
		/* 只投递命令, 执行/校验/读回由 RS485 服务线程负责(见 rs485_thread_entry) */
		if (RS485_Post(RS485_OP_RESET, 0) == RT_EOK)
			rt_kprintf("Sensor Reset queued (exec by rs485 thread)\n");
		else
			rt_kprintf("rs485 busy, try again\n");
		/* 结果由 rs485 服务线程打印: "Reset Zero Order OK/CRC err" + "[485] Angle = .." */
	  }	
	else if(rt_strcasecmp(Word[1],"reset")!= 0)
	  {
	    rt_kprintf("Input Err, Please Input: Sensor Reset ");
	
	  }
	
	}
	
	
	

}

//添加命令行
MSH_CMD_EXPORT_ALIAS(RS485Send_ResetZeroAdjust ,Sensor ,"Input: Input: Sensor Reset");


/********************************************************************************************
*函数名称:RS485Recive_ResetZeroOrderCheck
*作用    :检查编码器发来的复位零点指令的应答
*备注    :复位命令发出去,回传的是8字节
*备注    :从站(编码器)回传过来的数据除了crc校验的俩字节是按小端发过来的其余是大端
因此在拼接数据时候不要搞错
********************************************************************************************/
void RS485Recive_ResetZeroOrderCheck(void)
{

  if (tUsart2_RX.Index >= 8)
  {
    //crc校验
     uint16_t CRC_Calculate = CRC16_Modbus(tUsart2_RX.Data, 6);
     uint16_t CRC_Receive =((uint8_t)tUsart2_RX.Data[tUsart2_RX.Index - 2]) |((uint16_t)tUsart2_RX.Data[tUsart2_RX.Index - 1]<<8);
      
     if(CRC_Calculate  == CRC_Receive)	
   {
      rt_kprintf("Reset Zero Order OK\n");
   
   }
      else
    {
       rt_kprintf("CRC err\n");
    
    }     
  
  
  }
  
  else
  {
  
    rt_kprintf("NO Response the(Len = %d)\n" ,tUsart2_RX.Index );
  
  }
}

/*************************************************************************************************
*函数名称:RS485Send_CWDirectionSet
*作用    :设定编码器转动的方向为递增,即是顺时针递增还是逆时针递增
此函数设定逆时针为递增
**************************************************************************************************/
/* dir: 0 = CW 顺时针递增, 1 = CCW 逆时针递增 */
void RS485Send_DirectionSetOrder(uint8_t dir)
{
    tUsart2_RX.Index = 0;
    uint8_t cmd[8];
    cmd[0] = 0x01; cmd[1] = 0x06;

    cmd[2] = 0x00; cmd[3] = 0x09;       /* 寄存器 0x0009 */
    cmd[4] = 0x00; cmd[5] = dir;       /* 0=CW, 1=CCW */

    uint16_t crc = CRC16_Modbus(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    for (uint8_t i = 0; i < 8; i++)
    {
        RS485_SendText(cmd[i]);
    }
}

/********************************************************************************************
*函数名称:RS485Send_SetValueOrder
*作用    :设置编码器当前值(写角度, 寄存器 0x000B)
*参数    :value 0~4095 (raw LSB), 对应 0~360°
*备注    :手册 6.4.11: Tx 01 06 00 0B <值高> <值低> <CRC>  立即生效, 仅写(功能码0x06)
*********************************************************************************************/
void RS485Send_SetValueOrder(uint16_t value)
{
    tUsart2_RX.Index = 0;               /* 清接收缓冲 */

    uint8_t cmd[8];
    cmd[0] = 0x01; cmd[1] = 0x06;
    cmd[2] = 0x00; cmd[3] = 0x0B;       /* 寄存器 0x000B: 设置当前值 */
    cmd[4] = (uint8_t)(value >> 8);     /* 数据高字节在前 */
    cmd[5] = (uint8_t)(value & 0xFF);

    uint16_t crc = CRC16_Modbus(cmd, 6);
    cmd[6] = crc & 0xFF;                /* CRC 低字节在前 */
    cmd[7] = (crc >> 8) & 0xFF;

    for (uint8_t i = 0; i < 8; i++)
    {
        RS485_SendText(cmd[i]);
    }
}

/********************************************************************************************
*函数名称:RS485Recive_SetValueOrderCheck
*作用    :检查设置当前值的应答(回显 8 字节 + CRC 校验)
*********************************************************************************************/
void RS485Recive_SetValueOrderCheck(void)
{
    if (tUsart2_RX.Index >= 8)
    {
        uint16_t CRC_Calculate = CRC16_Modbus(tUsart2_RX.Data, 6);
        uint16_t CRC_Receive = ((uint8_t)tUsart2_RX.Data[tUsart2_RX.Index - 2])
                             | ((uint16_t)tUsart2_RX.Data[tUsart2_RX.Index - 1] << 8);
        if (CRC_Calculate == CRC_Receive)
        {
            rt_kprintf("Set Value Order OK\n");
        }
        else
        {
            rt_kprintf("CRC err\n");
        }
    }
    else
    {
        rt_kprintf("NO Response the(Len = %d)\n", tUsart2_RX.Index);
    }
}

/***************************************************************************************
*函数作用:发送命令,设置编码器当前角度值(输入度 0~360, 内部换算 raw 再写), 并读回确认
****************************************************************************************/
static void RS485Send_SetValueAdjust(int WordNum, char *Word[])
{
    if (WordNum != 2)
    {
        rt_kprintf("Input: WriteAngle <0-360> (degree) ");
        return;
    }

    uint16_t TempDeg = (uint16_t)atoi(Word[1]);
    if (TempDeg > 360) TempDeg = 360;

    /* 角度(度) → raw: raw = deg × 4096 / 360 = deg × 1024 / 90 */
    uint16_t Raw = (uint16_t)(((uint32_t)TempDeg * 1024) / 90);
    if (Raw > 4095) Raw = 4095;

    /* 只投递命令, 执行/校验/读回由 RS485 服务线程负责(见 rs485_thread_entry) */
    if (RS485_Post(RS485_OP_SETVAL, Raw) == RT_EOK)
        rt_kprintf("WriteAngle queued: %u deg -> raw %u\n", TempDeg, Raw);
    else
        rt_kprintf("rs485 busy, try again\n");
}
//添加命令
MSH_CMD_EXPORT_ALIAS(RS485Send_SetValueAdjust, WriteAngle, "WriteAngle <0-360> set encoder angle");


/***************************************************************************************
*函数作用:发送命令,读当前角度值
****************************************************************************************/
static void RS485Angle_Read(int WordNum ,char *Word[])
{
  if (WordNum < 2)
	{
	   rt_kprintf("Input: Read Angle ");
	  return;
	}
	else if(WordNum == 2)
	{
         if(rt_strcasecmp(Word[1],"Angle") == 0)
	   {
		
		/* 角度由 RS485 服务线程周期读取并更新缓存: 这里只读缓存, 不碰总线 */
		if (rt_tick_get() - tAngleControl.Angle_Tick < rt_tick_from_millisecond(2000))
		{
			rt_kprintf("Angle = %u.%u (cached)\n",
				tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);
		}
		else
		{
			/* 缓存超过2秒没更新 = 服务线程读不到编码器 */
			rt_kprintf("Read Angle FAIL (encoder no response)\n");
		}
	 	
	  }	
	else if(rt_strcasecmp(Word[1],"Angle")!= 0)
	  {
	    rt_kprintf("Input Err, Please Input: Read Angle ");
	
	  }
	
	}
	
	
	

}
//添加命令
MSH_CMD_EXPORT_ALIAS(RS485Angle_Read ,Read ,"Input: Read Angle");


/* 线程内读一次角度并打印(仅限本线程调用): 用于写/复位命令后立即刷新缓存 */
static void RS485_ServiceReadOnce(void)
{
    rt_thread_mdelay(50);              /* 等上一条命令的总线彻底空闲 */
    RS485Send_ReadSingleRingOrder();
    rt_thread_mdelay(50);              /* 等应答: 编码器14ms + 7字节传输 */
    if (RS485Recive_SingleRingValue())
    {
        rt_kprintf("[485] Angle = %u.%u\n",
                   tAngleControl.Angle_Current_x10 / 10,
                   tAngleControl.Angle_Current_x10 % 10);
    }
    else
    {
        rt_kprintf("[485] read after cmd FAIL\n");
    }
}

/*创建线程主体,防止俩线程同时访问U2,导致帧拆分产生bug*/
static void rs485_thread_entry(void *param)
{
    T_rs485_msg rxmsg;                 /* 收消息用局部变量 */

    /* 上电初始化: 此时无人竞争, 放在线程里做 */
    RS485Send_DirectionSetOrder(0);
    rt_thread_mdelay(50);

    for (;;)
    {
        /* ---- 1. 固定节拍读角度 → 刷新缓存(供主循环/Read命令用) ---- */
        RS485Send_ReadSingleRingOrder();
        rt_thread_mdelay(50);          /* 等应答: 编码器14ms + 7字节传输 */
        RS485Recive_SingleRingValue(); /* 成功时更新 Angle_Current_x10 / Angle_Tick */

        /* ---- 2. 队列里攒的命令, 一次性执行完(0超时=非阻塞) ---- */
        while (rt_mq_recv(&g_mq, &rxmsg, sizeof(rxmsg), 0) == RT_EOK)
        {
            switch (rxmsg.op)
            {
            case RS485_OP_SETVAL:
                RS485Send_SetValueOrder(rxmsg.val);
                rt_thread_mdelay(50);
                RS485Recive_SetValueOrderCheck();
                RS485_ServiceReadOnce();   /* 写后读回, 立刻刷新角度缓存 */
                break;

            case RS485_OP_RESET:
                RS485Send_ResetZeroOrder();
                rt_thread_mdelay(50);
                RS485Recive_ResetZeroOrderCheck();
                RS485_ServiceReadOnce();   /* 置零后读回, 立刻刷新角度缓存 */
                break;

            case RS485_OP_DIRSET:
                RS485Send_DirectionSetOrder((uint8_t)rxmsg.val);
                rt_thread_mdelay(50);
                break;

            default:
                break;
            }
            /* 以后加了 done 信号量, 在这里: if (rxmsg.done) rt_sem_release(rxmsg.done); */
        }
    }
}

/*下单接口函数,将要使用U2的资源给到static void rs485_thread_entry(void *param)
这个函数************************************************************************/
rt_err_t RS485_Post(T_rs485_op op, uint16_t val)
{
    T_rs485_msg txmsg;                 /* 发送方局部变量, rt_mq_send 会拷贝走 */
    txmsg.op  = op;
    txmsg.val = val;
    return rt_mq_send(&g_mq, &txmsg, sizeof(txmsg));   /* 满 → -RT_EFULL */
}



/*********************************************************************************************************************/
//rt_mq_init(...)	租下一块仓库地皮 + 办一张仓位证（g_mq）。证上登记：地皮地址、每格多大、共几格；随即把地切成格子并记下
//"哪些格空着"
//rt_mq_send(&g_mq, ...)	你拿着证去存货。管理员翻证上的"空闲格清单"，给你一格，把货倒进去，再把这格登记到"已占用"清单末尾
//rt_mq_recv(&g_mq, ...)	凭证取货。管理员从"已占用"清单头部取一格，把货倒给你，格子收回"空闲"清单

//RS485_Post(RS485_OP_SETVAL, Raw);          /* Raw=386 */

/* 2. RS485_Post 内部: 往消息盒子赋值 */
//txmsg.op  = RS485_OP_SETVAL;   /* op  盒子里放入数字 2 */
//txmsg.val = Raw;               /* val 盒子里放入数字 386 */
//rt_mq_send(&g_mq, &txmsg, sizeof(txmsg));   /* 整个盒子(字节)拷进队列 */

///* 3. 服务线程: 从队列取出盒子 */
//rt_mq_recv(&g_mq, &rxmsg, sizeof(rxmsg), 0);   /* 盒子拷进 rxmsg, op=2, val=386 */

///* 4. 分发: 拿 op 盒子里的数字去比 case 里的数字 */
//switch (rxmsg.op)              /* 当前值 = 2 */
//{
//case RS485_OP_SETVAL:          /* case 常量 = 2 → 命中! */
//    RS485Send_SetValueOrder(rxmsg.val);   /* 用参数 386 */
//    break;
//case RS485_OP_RESET:           /* = 3, 这次不命中 */
//...
//}


void RS485_ServiceInit(void)
{
    rt_thread_t tid;

    /* 1) 消息队列(静态内存池, 不占堆) */
    if (rt_mq_init(&g_mq, "rs485mq",
                   g_mq_pool, sizeof(T_rs485_msg),
                   sizeof(g_mq_pool), RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("[485] mq init FAIL\n");
        return;
    }

    /* 2) 线程(动态创建, 用堆) */
    tid = rt_thread_create("rs485",
                           rs485_thread_entry,
                           RT_NULL,
                           RS485_THREAD_STACK,
                           RS485_THREAD_PRIO,
                           RS485_THREAD_TICK);
    if (tid == RT_NULL)
    {
        rt_kprintf("[485] thread create FAIL\n");
        return;
    }
    rt_thread_startup(tid);
    rt_kprintf("[485] service thread started\n");
}
















