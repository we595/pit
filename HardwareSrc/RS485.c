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
void RS485Recive_SingleRingValue(void)
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

       //打印角度值
//        rt_kprintf("Angle = %u.%u deg (raw=%u)\n",
//               Angle / 10, Angle % 10, raw);	 
	 
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
		
	 RS485Send_ResetZeroOrder();
     rt_thread_mdelay(50); 
     RS485Recive_ResetZeroOrderCheck();
     rt_thread_mdelay(50);   
		   
     RS485Send_ReadSingleRingOrder();   /* 1. 发读单圈命令 */
    rt_thread_mdelay(50);              /* 2. 等应答（传感器14ms应答+收完）*/
    RS485Recive_SingleRingValue();     /* 3. 解析 + CRC 校验+存值到结构体  */
		   
 //打印角度值,用结构体的值
        rt_kprintf("Angle = %u.%u \n",
        tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);	 
	 
	 	
      
	rt_kprintf("Sensor Reset OK \n" )	;	
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

    rt_kprintf("Set %u deg -> raw %u\n", TempDeg, Raw);

    RS485Send_SetValueOrder(Raw);        /* 1. 发设置当前值命令 */
    rt_thread_mdelay(50);                /* 2. 等应答 */
    RS485Recive_SetValueOrderCheck();    /* 3. 应答 + CRC 检查 */

    rt_thread_mdelay(50);
    RS485Send_ReadSingleRingOrder();     /* 4. 读回确认 */
    rt_thread_mdelay(50);
    RS485Recive_SingleRingValue();
    rt_kprintf("Angle = %u.%u\n",
        tAngleControl.Angle_Current_x10 / 10, tAngleControl.Angle_Current_x10 % 10);
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
		
     RS485Send_ReadSingleRingOrder();   /* 1. 发读单圈命令 */
     rt_thread_mdelay(50);              /* 2. 等应答（传感器14ms应答+收完）*/
    RS485Recive_SingleRingValue();     /* 3. 解析 + CRC 校验 */
		   
		  //打印角度值
        rt_kprintf("Angle = %u.%u \n",
        tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);	  
	 	
	  }	
	else if(rt_strcasecmp(Word[1],"Angle")!= 0)
	  {
	    rt_kprintf("Input Err, Please Input: Read Angle ");
	
	  }
	
	}
	
	
	

}
//添加命令
MSH_CMD_EXPORT_ALIAS(RS485Angle_Read ,Read ,"Input: Read Angle");














