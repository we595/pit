#include "Pressure.h"
#include "define.h"
#include "init.h"
#include "motor.h"

/* α = 64/256 ≈ 0.25；越大跟随越快、滤波越弱 */
#define  q8_ADC_FilterALPHA     64 

/*参数换算*/
#define Press_ADC_4MA       596       //用的12位adc
#define Press_ADC_20MA      2978

/*寿命预测参数,以基准线浮动正负百分之20*/
#define Press_LifeBase_X100  1500
#define Press_LifeLow_X100   (Press_LifeBase_X100 * 8 / 10)    /* 1200 */
#define Press_LifeHigh_X100  (Press_LifeBase_X100 * 12 / 10)   /* 1800 */

/*******************************************************************************************
*函数作用:对ADC的值进行滤波处理
*备注   :此函数不用放在中断里,不像foc需要硬实时,这个慢一点也没事,放在主循环里就行
*备注   :此函数在dma中断每次执行后置标志位拿到新数据
********************************************************************************************/

void ADC_Filter(void)
{
	
	if(ADCFlag() == 1)
	{
	 /* IN3 一阶低通 */
    tDMA.ADC_FilterIN3 = (uint16_t)(
        ((uint32_t)(256 - q8_ADC_FilterALPHA) * tDMA.ADC_FilterIN3
         + (uint32_t)q8_ADC_FilterALPHA * tDMA.ADC_DR_IN3) >> 8);
    tDMA.ADC_DR_IN3Average = tDMA.ADC_FilterIN3;

    /* IN13 一阶低通 */
    tDMA.ADC_FilterIN13 = (uint16_t)(
        ((uint32_t)(256 - q8_ADC_FilterALPHA) * tDMA.ADC_FilterIN13
         + (uint32_t)q8_ADC_FilterALPHA * tDMA.ADC_DR_IN13) >> 8);
    tDMA.ADC_DR_IN13Average = tDMA.ADC_FilterIN13;
	
	}
	
}

/*****************************************************************************************
*函数名称:ADC_ZeroAdjust
*作用:校准ADC的0点
*备注:校准完毕后在运行 滤波,电机闭环,此时有压力
*******************************************************************************************/
void ADC_ZeroAdjust(void)
{
	 if (tDMA.ADC_ZeroDone) return;  //校准完成后面不在跑
	
   if (tDMA.ADC_Index == 0)
   {
	  tDMA.ADC_NumIN3++;
     tDMA.ADC_Sum32_IN3 += tDMA.ADC_DR_IN3;
	   
	 if(tDMA.ADC_NumIN3 >= 1024)
	 {
	   tDMA.ADC_ZeroIN3 = tDMA.ADC_Sum32_IN3 >> 10;
	    
		 tDMA.ADC_NumIN3 = 0;
		 tDMA.ADC_Sum32_IN3 = 0;
	 }
     
     tDMA.ADC_Index = 1;
   }
   else if(tDMA.ADC_Index == 1)
   {
      tDMA.ADC_NumIN13++;
     tDMA.ADC_Sum32_IN13 += tDMA.ADC_DR_IN13;
	   
	 if(tDMA.ADC_NumIN13 >= 1024)
	 {
	   tDMA.ADC_ZeroIN13 = tDMA.ADC_Sum32_IN13 >> 10;
	    
		 tDMA.ADC_NumIN13 = 0;
		 tDMA.ADC_Sum32_IN13 = 0;
	 }
	 tDMA.ADC_Index = 2;
   
   }
   else{
    tDMA.ADC_ZeroDone = 1;
   
   }
}



/*******************************************************************************************
*函数名称:ADC_To_Press_X100
*作用    :将ADC值转为压力值
*备注    :采样电阻模块120豪欧,校准完成done置1执行adc到压力转化
**********************************************************************************************/
static void ADC_To_Press_X100(void)
{
	uint32_t Temp1,Temp2;
	
	if (tDMA.ADC_ZeroDone == 1)
	{
	
	if(tDMA.ADC_DR_IN13Average <= Press_ADC_4MA )        tPressure.Pressure2_IN13 = 0;
   else if(tDMA.ADC_DR_IN13Average >= Press_ADC_20MA)   tPressure.Pressure2_IN13 = 3000; 
   
   if(tDMA.ADC_DR_IN3Average <= Press_ADC_4MA )	        tPressure.Pressure1_IN3 = 0;  
 else if(tDMA.ADC_DR_IN3Average >= Press_ADC_20MA)      tPressure.Pressure1_IN3 = 3000;

	//in13
	if(tDMA.ADC_DR_IN13Average>Press_ADC_4MA && tDMA.ADC_DR_IN13Average<Press_ADC_20MA)
	{
	//用校准的0点进行计算
	Temp1 = ((tDMA.ADC_DR_IN13Average - tDMA.ADC_ZeroIN13) * 3000)/(Press_ADC_20MA - tDMA.ADC_ZeroIN13);
    tPressure.Pressure2_IN13 = Temp1;
	}
	
	//in3
    if(tDMA.ADC_DR_IN3Average>Press_ADC_4MA && tDMA.ADC_DR_IN3Average<Press_ADC_20MA)
	 {
	 Temp2 = ((tDMA.ADC_DR_IN3Average - tDMA.ADC_ZeroIN3) * 3000)/(Press_ADC_20MA - tDMA.ADC_ZeroIN3);
    tPressure.Pressure1_IN3 = Temp2;
	 }
	
	}
	
   
	
}

/********************************************************************************************
*函数名称:Press_OverloadJudge
*作用    :判断是否过载
*返回参数:作为过载判断(任一油缸过载就报警)
***********************************************************************************************/
 static uint8_t Press_OverloadJudge(void)
{
    if (tPressure.Pressure1_IN3 > 2500 || tPressure.Pressure2_IN13 > 2500)
        return 1;         /* 过载 */
	
    return 0;        /* 正常 */
}

/************************************************************************************************
*函数名称:Pressure_LifeCheck
*作用:    寿命判断
**************************************************************************************************/
void Pressure_LifeCheck (uint16_t P1 ,  uint16_t P2)
{
   if (P1 > tLife.Peak1_X100)   tLife.Peak1_X100 = P1;
    if (P2 > tLife.Peak2_X100)  tLife.Peak2_X100 = P2;
    tLife.CycleCount++;
	
	
	//过载优先
	 if (Press_OverloadJudge())
        tLife.AlarmFlag = Pressure_Overload;

    else if (P1 > Press_LifeHigh_X100 ||
             P2 > Press_LifeHigh_X100)       tLife.AlarmFlag =  Pressure_Rusting; /* 锈蚀/摩擦.1 */
	
    else if (P1 < Press_LifeLow_X100 ||
             P2 < Press_LifeLow_X100)        tLife.AlarmFlag =  Pressure_Leak; /* 泄漏.2 */
	
    else                                     tLife.AlarmFlag = Pressure_Normal;//0


}



//过载停机,放主循环
static void Press_OverloadStop(void)
{
if (Press_OverloadJudge())
    {
        MotorDirectionSet(Motor_Dir_Stop);
        MotorSpeedSet(0);
        tLife.AlarmFlag = Pressure_Overload;
        rt_kprintf("OVERLOAD!\n");
    }

}


//压力模块初始化
void Press_Init(void)
{
	ADC_To_Press_X100();
   Press_OverloadStop();
}

/********************************************************************************************
*msh命令: Press
*作用   : 查看两路压力(0.01MPa)
***********************************************************************************************/
static void Press_Cmd(int argc, char **argv)
{
    rt_kprintf("P1 = %u.%02u MPa  P2 = %u.%02u MPa\n",
               tPressure.Pressure1_IN3 / 100, tPressure.Pressure1_IN3 % 100,
               tPressure.Pressure2_IN13 / 100, tPressure.Pressure2_IN13 % 100);
}
MSH_CMD_EXPORT_ALIAS(Press_Cmd, Press, "Show 2 channel pressure");





