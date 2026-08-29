/*
 * Copyright (c) 2026, JingXiaxianmu Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2026-08-12     JingXiaxianmu   first implementation (开发平台骨架)
 *
 * 说明：本文件为"井下监测控制系统"开发平台的主程序骨架。
 * 当前仅实现：RT-Thread 启动 + 板载 LED(PE7) 闪烁验证。
 * 后续业务模块（阀门控制/传感器采集/扭矩计算/电池管理/微动测试等）
 * 将按文档《井下监测控制系统设计方案简版》在此工程基础上逐模块添加。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "Init.h"
#include "RS485.h"
#define DBG_TAG "JingXiaxianmu"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#include "define.h"
#include "motor.h"
#include "Pressure.h"

/* 潘多拉板板载 LED（PE7，与裸机工程"实验0-1"一致） */
#define LED_PIN             PIN_LED_R


uint16_t g_target = 45;
int main(void)
{
	Init();
	
	//0顺时针角度递增,1逆时针角度递增
	 RS485Send_DirectionSetOrder(0);

    /* 初始化 LED 引脚为输出模式 */
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    LOG_I("========================================");
    LOG_I("  JingXiaxianmu Underground System");
    LOG_I("  MCU: STM32L496   OS: RT-Thread");
    LOG_I("  Development Platform Skeleton");
    LOG_I("========================================");
    
//	do
//	{
//	 if ((USART2->ISR & USART_ISR_TXE) == 0)   /* ① 查询：发送寄存器空吗？*/
//    {
//        USART2->TDR = 0x99;                   /* ② 空就写 TDR，硬件自动发 */
//    }
//             
//	
//	}while(1);
//	
//	
	
//	

//调零
//RS485Send_ResetZeroOrder();
// rt_thread_mdelay(50); 
// RS485Recive_ResetZeroOrderCheck();
//rt_thread_mdelay(500); 

    while (1)
    {
		
   rt_thread_mdelay(10);//给个延时使系统让出
		
    //先校准0点
   ADC_ZeroAdjust();		
	
		
	//滤波调用	
	ADC_Filter();
		
	//滤波完换算
     Press_Init();
		
		
//  //输入角度在4到90°之间即可
Motor_Control(g_target);//初始45°
//		
		
		
		
		
		
	/*下面是测试函数*/
		
//        /* LED 亮 */
//        rt_pin_write(LED_PIN, PIN_LOW);
//        rt_thread_mdelay(500);

        /* LED 灭 */
//        rt_pin_write(LED_PIN, PIN_HIGH);
//        rt_thread_mdelay(500);

		//测试串口模块
//	 RS485_SendText(0x55);
//     rt_thread_mdelay(100);
//     RS485_SendText(0x35);
//     rt_thread_mdelay(100);	
//     RS485_SendText(0x15);	
//     rt_thread_mdelay(100);
		
//if (RxFlag() == 1)                          /* 中断通知：收到数据了 */
//        {
//			rt_thread_mdelay(30);
//			
//            rt_kprintf("Recv(%d): ", tUsart2_RX.Index);
//            for (uint8_t i = 0; i < tUsart2_RX.Index; i++)
//            {
//                rt_kprintf("%02X ", tUsart2_RX.Data[i]);
//            }
//            rt_kprintf("\n");
//            tUsart2_RX.Index = 0;              /* 清空计数，准备收下一帧 */
//        }
//        rt_thread_mdelay(10);	


/* 1. MCU 主动发读命令：读从站1，寄存器0x0000，1个寄存器 */
//    uint8_t cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
//    for (int i = 0; i < 8; i++)
//    {
//        RS485_SendText(cmd[i]);
//    }

//    /* 2. 等传感器应答 */
//    rt_thread_mdelay(100);

//    /* 3. 打印收到的应答 */
//    if (RxFlag())
//    {
//        rt_kprintf("Recv(%d): ", tUsart2_RX.Index);
//        for (uint8_t i = 0; i < tUsart2_RX.Index; i++)
//        {
//            rt_kprintf("%02X ", tUsart2_RX.Data[i]);
//        }
//        rt_kprintf("\n");
//        tUsart2_RX.Index = 0;
//    }

//    rt_thread_mdelay(1000);   /* 每秒读一次 */


//     RS485Send_ReadSingleRingOrder();   /* 1. 发读单圈命令 */
//    rt_thread_mdelay(50);              /* 2. 等应答（传感器14ms应答+收完）*/
//    RS485Recive_SingleRingValue();     /* 3. 解析 + CRC 校验 + 打印度数 */
//    /* RS485Recive_SingleRingCheck 里已有 mdelay(200)，轮询周期约 250ms */

//RS485Send_ReadSingleRingOrder();   /* 1. 发读单圈命令 */
//     rt_thread_mdelay(50);              /* 2. 等应答（传感器14ms应答+收完）*/
//    RS485Recive_SingleRingValue();     /* 3. 解析 + CRC 校验 */
//		   
//		  //打印角度值
//        rt_kprintf("Angle = %u.%u \n",
//        tAngleControl.Angle_Current_x10  / 10,  tAngleControl.Angle_Current_x10  % 10);


    }
}






