

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




#endif

