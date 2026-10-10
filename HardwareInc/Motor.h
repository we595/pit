

#include <stm32l4xx_hal.h>



#ifndef __Motor_H
#define __Motor_H


void Motor_Gpio_Init(void);
void MotorSpeedSet(int8_t Speed);
 void MotorDirectionSet(uint8_t Dir);


void Motor_Control(uint16_t TargetAngle);

void Motor_Init(void);

#define Motor_Dir_Stop               0
#define Motor_Dir_CW                 1
#define Motor_Dir_CCW                2

/* ---- 阀门配置(量程/功率, 可切换): msh 命令 ValveCfg Big|Small ---- */
typedef struct
{
    uint16_t open_lim_x10;    /* 开侧极限(0.1°): 低于此值强制朝角度增大方向回拉 */
    uint16_t close_lim_x10;   /* 关侧极限(0.1°): 高于此值强制朝角度减小方向回拉 */
    uint16_t tgt_min_deg;     /* 目标/开阀全开位置钳位下限(°) */
    uint16_t tgt_max_deg;     /* 目标/关阀全关位置钳位上限(°) */
    uint8_t  drive_spd;       /* 误差驱动速度(信号0-100) */
    uint8_t  lim_spd;         /* 限位/回绕拉回速度(信号0-100) */
} tValveCfg_Type;

extern tValveCfg_Type g_valve_cfg;   /* 当前生效配置(闭环/上位机共用) */




#endif

