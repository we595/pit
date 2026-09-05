#include "ModBus.h"
#include "Init.h"
#include "define.h"
#include "RS485.h"
#include "Motor.h"

/******************** 上位机协议: 读当前数据(40011~40014) ********************/
#define HW_VERSION   1          /* 硬件版本 (40013/0x000C) */
#define SW_VERSION   1          /* 软件版本 (40014/0x000D) */
static uint16_t s_ValveState = 0;   /* 阀门状态 (40011/0x000A): 0=停止 1=开阀 2=关阀 */

static void Modbus_Slave_Process(uint8_t *rx, uint8_t len);


/*********************************************************************************
*函数名称 : CRC_Modbus
*功能     : 校验CRC
*介绍     :将接收的字节进行计算,要求数据字节要与后两位校验字节一致
*输出     : 输出两字节 
**************************************************************************************/
uint16_t CRC16_Modbus(volatile uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

/* ************************************************************************************************
*备注;见下
******************************************************************************************************/
void U3_Modbus_Process(void)
{
    uint8_t frame[32];

    /* ① 兜底: 缓冲里有不足 8 字节的残帧(干扰/发到一半/不支持的变长帧残留),
         且 50ms 没再来字节 → 就是半截垃圾, 清掉, 防止粘住下一帧 */
    if (rt_ringbuffer_data_len(&usart3_rb) > 0 &&
        rt_ringbuffer_data_len(&usart3_rb) < 8 &&
        (rt_tick_get() - (uint32_t)tUsart3_RX.Tick) >= rt_tick_from_millisecond(50))
    {
        rt_ringbuffer_reset(&usart3_rb);
    }

    /* ② 攒够 8 字节 = 完整一帧, 立刻取走解析。一帧一帧来, 绝不取空混帧 */
    while (rt_ringbuffer_data_len(&usart3_rb) >= 8)
    {
        rt_ringbuffer_get(&usart3_rb, frame, 8);   /* 一次取 8 字节,并且将环形缓冲区字节放到frame里 */
        Modbus_Slave_Process(frame, 8);
    }
}


/*用于给上位机回复*/
static void U3_SendFrame(uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
        RS485_SendTextU3(buf[i]);          /* 需要 include RS485.h */
}



static void Modbus_Slave_Process(uint8_t *rx, uint8_t len)
{
    uint8_t tx[64], n;
    uint16_t crc;

	/*校验*/
    if (len < 8) return;                          /* 帧太短 */
    crc = CRC16_Modbus(rx, len - 2);              /* CRC 校验 */
    if (crc != (uint16_t)(rx[len-2] | (rx[len-1] << 8))) return;
    if (rx[0] != MODBUS_ADDR) return;             /* 只认从站地址 01 */

	/*功能*/
    switch (rx[1])
    {
    case 0x03:                                    /* 读保持寄存器 */
    {
        uint16_t start = (uint16_t)((rx[2] << 8) | rx[3]);/*  从第几个寄存器开始读 */
        uint16_t qty   = (uint16_t)((rx[4] << 8) | rx[5]); /*    一次读几个 */
        if (qty == 0 || qty > 16) return;

        tx[0] = MODBUS_ADDR; tx[1] = 0x03;              /* ② 先填应答头: 地址+功能码 */
        n = 3;                                         /* 应答=地址+功能码+字节数 */
        for (uint8_t i = 0; i < qty; i++)
        {
            uint16_t v = 0;
            switch (start + i)
            {
            case 0: v = (uint16_t)tAngleControl.Angle_Current_x10; break; /* PLC(40001)--00 00  当前角度0.1° */
            case 1: v = tPressure.Pressure1_IN3;  break;                  /* PLC40002 --00 01   压力1 0.01MPa */
            case 2: v = tPressure.Pressure2_IN13; break;                  /* PLC40003 -- 00 02    压力2 */
            case 3: v = tLife.AlarmFlag;          break;                  /* PLC40004 --00 03    报警字 */
            case 10: v = s_ValveState; break;                             /* PLC40011--00 0A 阀门状态: 0停 1开阀 2关阀 */
            case 11: v = (uint16_t)tAngleControl.Angle_Current_x10; break;/* PLC40012--00 0B 当前开度 0.1° (900=90.0°) */
            case 12: v = HW_VERSION; break;                               /* PLC40013--00 0C 硬件版本 */
            case 13: v = SW_VERSION; break;                               /* PLC40014--00 0D 软件版本 */
            default: v = 0xFFFF; break;
            }
            tx[n++] = (uint8_t)(v >> 8);                     /* 数据字节:高字节先进 */
            tx[n++] = (uint8_t)(v & 0xFF);                   /* 数据字节:低字节跟上 (大端) */
        }
        tx[2] = (uint8_t)(n - 3);
        crc = CRC16_Modbus(tx, n);
        tx[n++] = (uint8_t)(crc & 0xFF);
        tx[n++] = (uint8_t)(crc >> 8);
        if (start == 0x000A)   /* 【临时调试】上位机读当前数据时, 打印发送瞬间的缓存值 */
            rt_kprintf("[R] st=%u ang=%u.%u\n", s_ValveState,
                       (uint16_t)(tAngleControl.Angle_Current_x10 / 10),
                       (uint16_t)(tAngleControl.Angle_Current_x10 % 10));
        U3_SendFrame(tx, n);
        break;
    }
    case 0x06:                                    /* 写单寄存器 */
    {
        uint16_t reg = (uint16_t)((rx[2] << 8) | rx[3]);
        uint16_t val = (uint16_t)((rx[4] << 8) | rx[5]);
        switch (reg)
        {
        case 9:                                   /* 40010 -- 00 09 目标开度(度) */
            if (val < 3)  val = 3;
            if (val > 95) val = 95;
            g_target = (uint16_t)val;
            break;
        case 1:                                   /* 40002 -- 00 01 开/关阀命令(上位机协议): 1开阀 2关阀 */
        case 10:                                  /* 40011--00 0a 命令: 0停 1开阀 2关阀 */
            if (val <= 3) s_ValveState = val;     /* 记录阀门状态, 供读当前数据(40011)回传 */
            if      (val == 1) g_target = 3;      /* 开阀→全开 */
            else if (val == 2) g_target = 95;     /* 关阀→全关 */
		    else if (val == 0)
    {
        MotorDirectionSet(Motor_Dir_Stop);   /* 立即拉低两方向线 */
        MotorSpeedSet(0);                    /* DAC 归零         */
//        /* 再把目标改成当前位置, 防止闭环下一轮重启 */
//        uint32_t cur = tAngleControl.Angle_Current_x10;
//        if (cur > 1800) cur = 3600 - cur;
//        g_target = (uint16_t)((cur + 5) / 10);
//        if (g_target < 4)  g_target = 4;
//        if (g_target > 94) g_target = 94;
    }
            break;
        default:
            return;
        }
        U3_SendFrame(rx, len);                    /* 回显请求帧=成功 */
        break;
    }
    default:
        break;
    }
}



/*
Modbus协议是这样分功能的第一个模块表示从机地址位(一般占用一个字节)
第二个模块表示功能码位(一般占用一个字节),第三个模块表示(起始寄存器地址位)
第四个模块按功能码不同分为读几个寄存器或者写入数据

开阀
发送：01 06 00 01 00 01 19 CA
应答：01 06 00 01 00 01 19 CA
关阀
发送：01 06 00 01 00 02 59 CB
应答：01 06 00 01 00 02 59 CB

读当前数据
发送：01 03 00 0A 00 04 64 0B
应答：01 03 08 00 01 03 84 00 01 00 01 E5 3A
01    03        08  
地址  功能码  字节数
00 01   阀门状态 1表示开阀状态
03 84  当前角度 900，表示开度90.0度
00 01  硬件版本 1
00 01  软件版本 1
E5 3A  CRC校验


功能	完整报文（8 字节，可直接发）
读 40001 当前角度（0.1°）	02 03 00 00 00 01 84 39
读 40002 压力1（0.01MPa）	02 03 00 01 00 01 D5 F9
读 40003 压力2（0.01MPa）	02 03 00 02 00 01 25 F9
读 40004 报警字	            02 03 00 03 00 01 74 39
一次读角度+P1+P2（3 个）	02 03 00 00 00 03 05 F8

写指令（功能码 06，成功应答 = 原帧回显）
功能	                 完整报文（8 字节，可直接发)值
40011 = 0 停	          02 06 00 0A 00 00 A9 FB	
40011 = 1 开阀(全开4°)	  02 06 00 0A 00 01 68 3B	
40011 = 2 关阀(全关95°)  02 06 00 0A 00 02 28 3A	
40010 = 45° 目标开度	  02 06 00 09 00 2D 99 E6	
40010 = 4°（全开）	      02 06 00 09 00 04 58 38	
40010 = 95°（全关）	  02 06 00 09 00 5F 19 C3	

应答:02   03   02   0D C2   79 45
      │    │    │    │       │
      │    │    │    │       └─ CRC 校验(低字节79在前,高字节45在后) = 0x4579
      │    │    │    └───────── 寄存器数据(大端: 0D是高字节, C2是低字节)
      │    │    │              = 0x0DC2 = 3522
      │    │    └────────────── 后面"数据"占几个字节 = 2 字节(1个寄存器×2)
      │    └─────────────────── 功能码 03 = "这是对读请求的应答"
      └──────────────────────── 从站地址 02 = 回话的是 MCU

===== 当前上位机协议(从站地址 = 01, 见 Modbus_Slave_Process) =====
读当前数据(03, 40011~40014):
  发送: 01 03 00 0A 00 04 64 0B
  应答: 01 03 08 00 01 03 84 00 01 00 01 E5 3A
        40011(0x000A) 阀门状态: 0停 1开阀 2关阀
        40012(0x000B) 当前开度 0.1° (900 = 90.0°)
        40013(0x000C) 硬件版本 = 1
        40014(0x000D) 软件版本 = 1
开阀(06, 写 0x0001=1): 01 06 00 01 00 01 19 CA  (应答=原帧回显)
关阀(06, 写 0x0001=2): 01 06 00 01 00 02 59 CB  (应答=原帧回显)
*/




