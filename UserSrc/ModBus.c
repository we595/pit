#include "ModBus.h"
#include "Init.h"
#include "define.h"
#include "RS485.h"
#include "Motor.h"
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

    if (len < 8) return;                          /* 帧太短 */
    crc = CRC16_Modbus(rx, len - 2);              /* CRC 校验 */
    if (crc != (uint16_t)(rx[len-2] | (rx[len-1] << 8))) return;
    if (rx[0] != MODBUS_ADDR) return;             /* 只认地址2 */

    switch (rx[1])
    {
    case 0x03:                                    /* 读保持寄存器 */
    {
        uint16_t start = (uint16_t)((rx[2] << 8) | rx[3]);
        uint16_t qty   = (uint16_t)((rx[4] << 8) | rx[5]);
        if (qty == 0 || qty > 16) return;

        tx[0] = MODBUS_ADDR; tx[1] = 0x03;
        n = 3;                                    /* 应答=地址+功能码+字节数 */
        for (uint8_t i = 0; i < qty; i++)
        {
            uint16_t v = 0;
            switch (start + i)
            {
            case 0: v = (uint16_t)tAngleControl.Angle_Current_x10; break; /* 40001 当前角度0.1° */
            case 1: v = tPressure.Pressure1_IN3;  break;                  /* 40002 压力1 0.01MPa */
            case 2: v = tPressure.Pressure2_IN13; break;                  /* 40003 压力2 */
            case 3: v = tLife.AlarmFlag;          break;                  /* 40004 报警字 */
            default: v = 0xFFFF; break;
            }
            tx[n++] = (uint8_t)(v >> 8);
            tx[n++] = (uint8_t)(v & 0xFF);
        }
        tx[2] = (uint8_t)(n - 3);
        crc = CRC16_Modbus(tx, n);
        tx[n++] = (uint8_t)(crc & 0xFF);
        tx[n++] = (uint8_t)(crc >> 8);
        U3_SendFrame(tx, n);
        break;
    }
    case 0x06:                                    /* 写单寄存器 */
    {
        uint16_t reg = (uint16_t)((rx[2] << 8) | rx[3]);
        uint16_t val = (uint16_t)((rx[4] << 8) | rx[5]);
        switch (reg)
        {
        case 9:                                   /* 40010 目标开度(度) */
            if (val < 4)  val = 4;
            if (val > 94) val = 95;
            g_target = (uint16_t)val;
            break;
        case 10:                                  /* 40011 命令: 0停 1开阀 2关阀 */
            if      (val == 1) g_target = 4;      /* 开阀→全开 */
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

*/




