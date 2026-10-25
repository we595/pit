#ifndef __PRESSURE_H
#define __PRESSURE_H

#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>

void ADC_Filter(void);
void ADC_ZeroAdjust(void);



void Pressure_LifeCheck(void);           /* 全行程到位时调用(记录/评估) */
void Pressure_RunPeakSample(void);       /* 主循环每圈调用(运行峰值保持) */
void Pressure_RunPeakClear(void);        /* 目标变化/重标定时清运行峰值 */
void Pressure_GetLastPeak(uint16_t *P1, uint16_t *P2);
void Pressure_GetTripPeak(uint16_t *P1, uint16_t *P2);  /* 最近一次超压事件的峰值 */
void Pressure_OverloadReportReset(void);                /* 重新允许过载上报(复位/正常恢复后) */
uint8_t Pressure_StuckCheck(void);       /* 驱动中调用: 运行峰值≥130%标称 → 1=老化/卡滞故障 */
void Press_Init(void);
void Press_OverloadStop(void);



#endif


