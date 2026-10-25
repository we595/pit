#include "Pressure.h"
#include "define.h"
#include "init.h"
#include "motor.h"

/* α = 64/256 ≈ 0.25；越大跟随越快、滤波越弱 */
#define  q8_ADC_FilterALPHA     64 

/*参数换算*/
#define Press_ADC_4MA       596       //用的12位adc
#define Press_ADC_20MA      2978

/*寿命预测参数*/
#define Press_LifeBase_X100  1500   /* [已弃用] 旧固定基线, 现改为 PressBase 命令实测标称值 */
#define Press_LifeLow_X100   (Press_LifeBase_X100 * 8 / 10)
#define Press_LifeHigh_X100  (Press_LifeBase_X100 * 12 / 10)
#define PRESS_OVERLOAD_X100  1800  /* 过载阈值24MPa(原4000=40MPa超出30MPa量程, 永不触发; 设计24~25MPa停) */
#define PRESS_OVERLOAD_RELEASE_X100 (PRESS_OVERLOAD_X100 - 200)  /* 22MPa: 回落到此值以下才算本次超限结束(回差防抖) */
#define PRESS_OVERLOAD_REPORT_MAX   3     /* 同一故障周期最多打印几次(putty调试防刷屏) */
#define PRESS_STUCK_PCT      130    /* 老化/卡滞故障线: 标称值的130% */
#define PRESS_STUCK_HOLD_CNT 40     /* "持续超线"判定次数(主循环≈10~15ms一次 → 约0.5s): 连续/密集超线达此值才判卡滞, 瞬时尖峰不触发 */
#define PRESS_BASE_MIN_X100  300    /* 标称值采集下限3MPa(防没顶出压力时误采) */

/*上电默认标称值(0.01MPa, 500=5.00MPa, 0=未标定)*/
/*标定完后把 LifeDbg 实测值填这里 → 免每次上电敲PressBase; 测试时可填小值(如50=0.50MPa)*/
#define PRESS_BASE1_DEFAULT_X100  1200
#define PRESS_BASE2_DEFAULT_X100  1200

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
*作用:校准ADC的0点 —— 【已停用】两缸交替承压, 上电时总有一路带压, 不能当零点
*备注:保留仅供以后"确认无压时手动校准"扩展; 主循环不再调用, 4mA零点用常数换算
*******************************************************************************************/
void ADC_ZeroAdjust(void)
{
	 if (tDMA.ADC_ZeroDone) return;  //校准完成后面不在跑
	
   if (tDMA.ADC_Index == 0)
   {
	  tDMA.ADC_NumIN3++;
     tDMA.ADC_Sum32_IN3 += tDMA.ADC_DR_IN3;
	   
	 if(tDMA.ADC_NumIN3 >= 256)
	 {
	   tDMA.ADC_ZeroIN3 = tDMA.ADC_Sum32_IN3 >> 8;
	    
		 tDMA.ADC_NumIN3 = 0;
		 tDMA.ADC_Sum32_IN3 = 0;
	 }
     
     tDMA.ADC_Index = 1;
   }
   else if(tDMA.ADC_Index == 1)
   {
      tDMA.ADC_NumIN13++;
     tDMA.ADC_Sum32_IN13 += tDMA.ADC_DR_IN13;
	   
	 if(tDMA.ADC_NumIN13 >= 256)
	 {
	   tDMA.ADC_ZeroIN13 = tDMA.ADC_Sum32_IN13 >> 8;
	    
		 tDMA.ADC_NumIN13 = 0;
		 tDMA.ADC_Sum32_IN13 = 0;
		 tDMA.ADC_ZeroDone = 1;
	 }
	 tDMA.ADC_Index = 2;
   
   }
   else{
	   tDMA.ADC_Index = 0;
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
	
	/* 两缸交替承压: 不再上电校零(上电时总有一路带压, 会被当成零点);
	   4-20mA 的 4mA 即标准零点, 直接用固定常数 596?0MPa / 2978?30MPa 换算 */
	
	if(tDMA.ADC_DR_IN13Average <= Press_ADC_4MA )        tPressure.Pressure2_IN13 = 0;
   else if(tDMA.ADC_DR_IN13Average >= Press_ADC_20MA)   tPressure.Pressure2_IN13 = 3000; 
   
   if(tDMA.ADC_DR_IN3Average <= Press_ADC_4MA )	        tPressure.Pressure1_IN3 = 0;  
 else if(tDMA.ADC_DR_IN3Average >= Press_ADC_20MA)      tPressure.Pressure1_IN3 = 3000;

	//in13
	if(tDMA.ADC_DR_IN13Average>Press_ADC_4MA && tDMA.ADC_DR_IN13Average<Press_ADC_20MA)
	{
	//4mA 固定零点换算
	Temp1 = ((uint32_t)(tDMA.ADC_DR_IN13Average - Press_ADC_4MA) * 3000)/(Press_ADC_20MA - Press_ADC_4MA);
    tPressure.Pressure2_IN13 = Temp1;
	}
	
	//in3
    if(tDMA.ADC_DR_IN3Average>Press_ADC_4MA && tDMA.ADC_DR_IN3Average<Press_ADC_20MA)
	 {
	 Temp2 = ((uint32_t)(tDMA.ADC_DR_IN3Average - Press_ADC_4MA) * 3000)/(Press_ADC_20MA - Press_ADC_4MA);
    tPressure.Pressure1_IN3 = Temp2;
	 }
	
   
	
}

/********************************************************************************************
*函数名称:Press_OverloadJudge
*作用    :判断是否过载
*返回参数:作为过载判断(任一油缸过载就报警)
***********************************************************************************************/
 static uint8_t Press_OverloadJudge(void)
{
    if (tPressure.Pressure1_IN3 > PRESS_OVERLOAD_X100 || tPressure.Pressure2_IN13 > PRESS_OVERLOAD_X100)
        return 1;         /* 过载 */
	
    return 0;        /* 正常 */
}

/*======================= 老化/卡滞检测(标称值对比法) =======================
 * 思路: 标定时顶到极限把压力记为"标称值"(两路各记一个);
 *       运行中(未到位)时压力"持续"达到标称值的130% → 老化/卡滞故障。
 * 原因: 顶到极限的瞬间有压力冲击尖峰, 只看"持续超线"(漏桶计数),
 *       瞬时尖峰/ADC噪声不会误触发; 真正卡死是持续顶高压。
 *=========================================================================*/
static uint16_t s_RunPeak1 = 0;    /* 本次运行期间峰值(Press_Init 每圈保持, 打印/历史用) */
static uint16_t s_RunPeak2 = 0;
static uint16_t s_LastPeak1 = 0;   /* 最近一次全行程到位时的峰值(打印用) */
static uint16_t s_LastPeak2 = 0;

static uint16_t s_Base1 = PRESS_BASE1_DEFAULT_X100;   /* 标称值: 开极限侧(P1); PressBase采集 或 写死默认值/PressBaseSet手动设 */
static uint16_t s_Base2 = PRESS_BASE2_DEFAULT_X100;   /* 标称值: 关极限侧(P2) */
static uint8_t  s_Base1Valid = (PRESS_BASE1_DEFAULT_X100 > 0);
static uint8_t  s_Base2Valid = (PRESS_BASE2_DEFAULT_X100 > 0);
static uint16_t s_Hold1 = 0;       /* "持续超线"漏桶计数: 超线+1 低于线-1, 达 PRESS_STUCK_HOLD_CNT 判卡滞 */
static uint16_t s_Hold2 = 0;
static uint16_t s_TripPeak1 = 0;   /* 最近一次超限事件峰值(停机瞬间压力可能还会冲一下, 持续取最大) */
static uint16_t s_TripPeak2 = 0;
static uint8_t  s_OverloadReportCnt = 0;   /* 本故障周期已打印次数(达 PRESS_OVERLOAD_REPORT_MAX 后静默) */

/************************************************************************************************
*函数名称:Pressure_RunPeakSample
*作用:    运行期间连续保持两路压力峰值(主循环每圈调用)
**************************************************************************************************/
void Pressure_RunPeakSample(void)
{
    uint32_t lim1 = (uint32_t)s_Base1 * PRESS_STUCK_PCT / 100;
    uint32_t lim2 = (uint32_t)s_Base2 * PRESS_STUCK_PCT / 100;

    if (tPressure.Pressure1_IN3  > s_RunPeak1)  s_RunPeak1 = tPressure.Pressure1_IN3;
    if (tPressure.Pressure2_IN13 > s_RunPeak2)  s_RunPeak2 = tPressure.Pressure2_IN13;

    /* "持续超线"计数(漏桶): 只在压力≥故障线时累积, 低于线立刻回落,
     * 所以顶极限一瞬间的冲击尖峰累计不到阈值, 只有持续顶高压才会 */
    if (s_Base1Valid && tPressure.Pressure1_IN3 >= lim1)  { if (s_Hold1 < PRESS_STUCK_HOLD_CNT) s_Hold1++; }
    else if (s_Hold1 > 0)                                 s_Hold1--;

    if (s_Base2Valid && tPressure.Pressure2_IN13 >= lim2) { if (s_Hold2 < PRESS_STUCK_HOLD_CNT) s_Hold2++; }
    else if (s_Hold2 > 0)                                 s_Hold2--;
}

/* 清运行峰值+持续计数(目标变化/重新标定时调用) */
void Pressure_RunPeakClear(void)
{
    s_RunPeak1 = 0;
    s_RunPeak2 = 0;
    s_Hold1 = 0;
    s_Hold2 = 0;
}

/* 读取最近一次全行程到位的峰值(打印用) */
void Pressure_GetLastPeak(uint16_t *P1, uint16_t *P2)
{
    if (P1) *P1 = s_LastPeak1;
    if (P2) *P2 = s_LastPeak2;
}

/* 读取最近一次超限事件的峰值(Modbus/打印用) */
void Pressure_GetTripPeak(uint16_t *P1, uint16_t *P2)
{
    if (P1) *P1 = s_TripPeak1;
    if (P2) *P2 = s_TripPeak2;
}

/* 重新允许过载上报(故障复位/全行程正常恢复后调用) */
void Pressure_OverloadReportReset(void)
{
    s_OverloadReportCnt = 0;
}

/************************************************************************************************
*函数名称:Pressure_StuckCheck
*作用:    老化/卡滞故障检查 —— 闭环"正在驱动(未到位)"分支里每圈调用:
*         任一路压力"持续"≥ 该路标称值的130%(s_Hold 计数达阈值) → 返回1(并置故障标志)
*备注:    改自"运行峰值"判据: 顶极限瞬间的冲击尖峰会误触发;
*         持续计数由 Pressure_RunPeakSample 维护, 瞬时尖峰累计不到阈值
**************************************************************************************************/
uint8_t Pressure_StuckCheck(void)
{
    if (s_Base1Valid && s_Hold1 >= PRESS_STUCK_HOLD_CNT)
    {
        rt_kprintf("[Life] STUCK/aged: P1=%u.%02u MPa held >= %u%% of base %u.%02u (peak %u.%02u), angle NOT reached\n",
                   tPressure.Pressure1_IN3 / 100, tPressure.Pressure1_IN3 % 100, (unsigned)PRESS_STUCK_PCT,
                   s_Base1 / 100, s_Base1 % 100, s_RunPeak1 / 100, s_RunPeak1 % 100);
        if (tLife.AlarmFlag != Pressure_Overload)   /* 过载故障优先, 不被老化/卡滞覆盖 */
            tLife.AlarmFlag = Pressure_Rusting;
        return 1;
    }
    if (s_Base2Valid && s_Hold2 >= PRESS_STUCK_HOLD_CNT)
    {
        rt_kprintf("[Life] STUCK/aged: P2=%u.%02u MPa held >= %u%% of base %u.%02u (peak %u.%02u), angle NOT reached\n",
                   tPressure.Pressure2_IN13 / 100, tPressure.Pressure2_IN13 % 100, (unsigned)PRESS_STUCK_PCT,
                   s_Base2 / 100, s_Base2 % 100, s_RunPeak2 / 100, s_RunPeak2 % 100);
        if (tLife.AlarmFlag != Pressure_Overload)   /* 过载故障优先, 不被老化/卡滞覆盖 */
            tLife.AlarmFlag = Pressure_Rusting;
        return 1;
    }
    return 0;
}

/************************************************************************************************
*函数名称:Pressure_LifeCheck
*作用:    全行程到位时调用一次: 记录历史峰值/动作次数; 正常完成 → 清故障(自动恢复)
*备注:    运行峰值用完即清零, 下一次运行重新累计
**************************************************************************************************/
void Pressure_LifeCheck(void)
{
    uint16_t P1 = s_RunPeak1;
    uint16_t P2 = s_RunPeak2;

    s_RunPeak1 = 0;
    s_RunPeak2 = 0;
    s_Hold1 = 0;
    s_Hold2 = 0;
    s_LastPeak1 = P1;
    s_LastPeak2 = P2;

    if (P1 > tLife.Peak1_X100)  tLife.Peak1_X100 = P1;   /* 历史最大峰值 */
    if (P2 > tLife.Peak2_X100)  tLife.Peak2_X100 = P2;
    tLife.CycleCount++;                                   /* 完成一次全行程动作 */

    /* 全行程正常走完(全程没被"持续超压"判据拦停) → 故障恢复;
     * 到位瞬间的峰值含冲击尖峰, 不再单独作为故障依据 */
    if (s_Base1Valid || s_Base2Valid)
    {
        tLife.AlarmFlag = Pressure_Normal;
        s_OverloadReportCnt = 0;          /* 已恢复 → 下次过载重新允许上报3次 */
    }
}


/********************************************************************************************
*函数名称:Press_OverloadStop
*作用    :过载(压力超限)保护 —— 主循环每圈调用:
*         超限 → 立即停机, 记录"当时运行方向"+峰值, 故障码置过载;
*                同时目标角度锁成当前角度, 压力消失后闭环不会自己重启(防启停振荡);
*         回落 → 结束本次事件。之后上位机/msh 重新下发目标即可正常再启动,
*                再超限照样再停(不锁死方向)。
*备注    :方向取自 Motor.c 的 g_motor_drive_dir(CW=开阀/角度变小, CCW=关阀/角度变大)
*********************************************************************************************/
void Press_OverloadStop(void)
{
    uint16_t p1 = tPressure.Pressure1_IN3;
    uint16_t p2 = tPressure.Pressure2_IN13;

    if (Press_OverloadJudge())
    {
        /* 停机 + 记录方向 + 目标锁当前角度(新事件才做, 持续超限只保持停机) */
        uint8_t is_new = Motor_ProtectTrip();

        if (is_new)                       /* 新事件: 峰值从当前值起算 */
        {
            s_TripPeak1 = p1;
            s_TripPeak2 = p2;
        }
        else                              /* 同一次事件: 持续取最大(停机瞬间压力可能还会冲一下) */
        {
            if (p1 > s_TripPeak1) s_TripPeak1 = p1;
            if (p2 > s_TripPeak2) s_TripPeak2 = p2;
        }
        if (s_TripPeak1 > tLife.Peak1_X100) tLife.Peak1_X100 = s_TripPeak1;  /* 历史最大峰值 */
        if (s_TripPeak2 > tLife.Peak2_X100) tLife.Peak2_X100 = s_TripPeak2;

        if (is_new)                       /* 每个事件只在上升沿处理一次 */
        {
            tLife.AlarmFlag = Pressure_Overload;
            tLife.OverloadCount++;

            /* putty 调试防刷屏: 同一故障周期只连报3次, 之后静默;
             * ProtectReset / 全行程正常恢复后重新计数 */
            if (s_OverloadReportCnt == 0)
            {
                uint8_t i;
                for (i = 0; i < PRESS_OVERLOAD_REPORT_MAX; i++)
                {
                    rt_kprintf("[PROT] OVERLOAD STOP! dir=%s  P1=%u.%02u  P2=%u.%02u MPa  tripPeak P1=%u.%02u P2=%u.%02u\n",
                               (g_fault_dir == Motor_FaultDir_Open)  ? "OPEN/CW(开阀,角度变小)" :
                               (g_fault_dir == Motor_FaultDir_Close) ? "CLOSE/CCW(关阀,角度变大)" : "IDLE(停机,方向未知)",
                               p1 / 100, p1 % 100, p2 / 100, p2 % 100,
                               s_TripPeak1 / 100, s_TripPeak1 % 100,
                               s_TripPeak2 / 100, s_TripPeak2 % 100);
                }
                s_OverloadReportCnt = PRESS_OVERLOAD_REPORT_MAX;
            }
        }
    }
    else if (p1 < PRESS_OVERLOAD_RELEASE_X100 && p2 < PRESS_OVERLOAD_RELEASE_X100)
    {
        /* 压力回落: 本次事件结束。目标保持当前角度, 不自动重启;
         * 上位机/msh 重新下发目标(新命令)才恢复驱动 */
        Motor_ProtectRelease();
    }
}


//压力模块初始化
void Press_Init(void)
{
	ADC_To_Press_X100();
   Pressure_RunPeakSample();   /* 运行峰值保持: 压力只在顶极限运行时存在, 停机即消失 */
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

/********************************************************************************************
*msh命令: AdcDbg
*作用   : 打印压力ADC的原始值/滤波值/零点/校准状态, 用于定位"压力恒0"问题
*判读   : Raw≈0     → 模块没供电/Signal没接/传感器回路断开(0mA输出0V)
*         Raw≈596   → 有4mA但加压不变 → 传感器没进回路或I+/I-接反
*         Raw随压力变但Press=0 → 软件换算问题(看Zero/Filt)
***********************************************************************************************/
static void AdcDbg_Cmd(int argc, char **argv)
{
    rt_kprintf("Raw   IN3=%u  IN13=%u  VBAT=%u\n",
               tDMA.ADC_DR_IN3, tDMA.ADC_DR_IN13, tDMA.ADC_DR_VBAT);
    rt_kprintf("Filt  IN3=%u  IN13=%u\n",
               tDMA.ADC_DR_IN3Average, tDMA.ADC_DR_IN13Average);
    rt_kprintf("Zero  IN3=%u  IN13=%u  Done=%u  Index=%u\n",
               tDMA.ADC_ZeroIN3, tDMA.ADC_ZeroIN13,
               tDMA.ADC_ZeroDone, tDMA.ADC_Index);
}
MSH_CMD_EXPORT_ALIAS(AdcDbg_Cmd, AdcDbg, "Show raw ADC/filter/zero for pressure debug");

/********************************************************************************************
*msh命令: PressMon
*作用   : 连续打印两路压力+运行峰值(默认25次×200ms≈5秒), 观察推极限时的压力曲线
*用法   : PressMon [count]   配合 Motor CW/CCW 开环顶极限使用最直观
*判读   : 正常推极限: 稳态压力应明显低于故障线(标称×130%);
*         若"稳态"就超线 → 标称值定得太低, 需要重标
*********************************************************************************************/
static void PressMon_Cmd(int argc, char **argv)
{
    int n = 25;
    int i;

    if (argc >= 2)
    {
        n = atoi(argv[1]);
        if (n <= 0 || n > 600) n = 25;
    }

    for (i = 0; i < n; i++)
    {
        rt_kprintf("P1=%u.%02u P2=%u.%02u  pk1=%u.%02u pk2=%u.%02u\n",
                   tPressure.Pressure1_IN3 / 100, tPressure.Pressure1_IN3 % 100,
                   tPressure.Pressure2_IN13 / 100, tPressure.Pressure2_IN13 % 100,
                   s_RunPeak1 / 100, s_RunPeak1 % 100,
                   s_RunPeak2 / 100, s_RunPeak2 % 100);
        rt_thread_mdelay(200);
    }
}
MSH_CMD_EXPORT_ALIAS(PressMon_Cmd, PressMon, "Print pressure/peak every 200ms: PressMon [count]");

/********************************************************************************************
*msh命令: PressBase — 标定"标称压力"
*用法   : 开环把阀顶到极限(压力已顶起来)时敲此命令;
*         自动把当前"运行峰值"记到当时有压的那一路(开极限→P1, 关极限→P2)。
*********************************************************************************************/
static void PressBase_Cmd(int argc, char **argv)
{
    if (s_RunPeak1 >= s_RunPeak2)
    {
        if (s_RunPeak1 >= PRESS_BASE_MIN_X100)
        {
            uint32_t lim;
            s_Base1 = s_RunPeak1;
            s_Base1Valid = 1;
            lim = (uint32_t)s_Base1 * PRESS_STUCK_PCT / 100;
            rt_kprintf("[Life] Base1(P1/开极限侧)=%u.%02u MPa, 故障线=%u.%02u MPa\n",
                       s_Base1 / 100, s_Base1 % 100, lim / 100, lim % 100);
        }
        else
            rt_kprintf("[Life] P1peak太低(%u.%02u MPa), 没顶出压力? 不记录\n",
                       s_RunPeak1 / 100, s_RunPeak1 % 100);
    }
    else
    {
        if (s_RunPeak2 >= PRESS_BASE_MIN_X100)
        {
            uint32_t lim;
            s_Base2 = s_RunPeak2;
            s_Base2Valid = 1;
            lim = (uint32_t)s_Base2 * PRESS_STUCK_PCT / 100;
            rt_kprintf("[Life] Base2(P2/关极限侧)=%u.%02u MPa, 故障线=%u.%02u MPa\n",
                       s_Base2 / 100, s_Base2 % 100, lim / 100, lim % 100);
        }
        else
            rt_kprintf("[Life] P2peak太低(%u.%02u MPa), 没顶出压力? 不记录\n",
                       s_RunPeak2 / 100, s_RunPeak2 % 100);
    }
    Pressure_RunPeakClear();
}
MSH_CMD_EXPORT_ALIAS(PressBase_Cmd, PressBase, "Record current run-peak as nominal (push to limit first)");

/********************************************************************************************
*msh命令: PressBaseSet — 手动设置标称值(测试用低值 / 标定后写死)
*用法   : PressBaseSet <P1_x100> <P2_x100>   单位0.01MPa(500=5.00MPa); 0=该路禁用
*用途   : ① 设低值(如 50 50 = 0.50MPa, 故障线0.65MPa)验证"超130%停机"逻辑
*         ② 标定值写死: 把 LifeDbg 实测值填进 PRESS_BASEx_DEFAULT_X100, 或此处手动设(掉电失效)
*********************************************************************************************/
static void PressBaseSet_Cmd(int argc, char **argv)
{
    int v1, v2;
    uint32_t lim1, lim2;

    if (argc != 3)
    {
        rt_kprintf("Input: PressBaseSet <P1_x100> <P2_x100>  (0.01MPa, 0=off)  e.g. 50 50\n");
        rt_kprintf("Now   Base1=%u.%02u MPa(%s)  Base2=%u.%02u MPa(%s)\n",
                   s_Base1 / 100, s_Base1 % 100, s_Base1Valid ? "ok" : "--",
                   s_Base2 / 100, s_Base2 % 100, s_Base2Valid ? "ok" : "--");
        return;
    }

    v1 = atoi(argv[1]);   if (v1 < 0) v1 = 0;   if (v1 > 3000) v1 = 3000;   /* 上限30MPa量程 */
    v2 = atoi(argv[2]);   if (v2 < 0) v2 = 0;   if (v2 > 3000) v2 = 3000;

    s_Base1 = (uint16_t)v1;   s_Base1Valid = (v1 > 0);
    s_Base2 = (uint16_t)v2;   s_Base2Valid = (v2 > 0);
    Pressure_RunPeakClear();   /* 清旧峰值, 从下一次运行重新累计 */

    lim1 = (uint32_t)s_Base1 * PRESS_STUCK_PCT / 100;
    lim2 = (uint32_t)s_Base2 * PRESS_STUCK_PCT / 100;
    rt_kprintf("[Life] Base1=%u.%02u MPa(%s) 故障线%u.%02u | Base2=%u.%02u MPa(%s) 故障线%u.%02u\n",
               s_Base1 / 100, s_Base1 % 100, s_Base1Valid ? "ok" : "off", lim1 / 100, lim1 % 100,
               s_Base2 / 100, s_Base2 % 100, s_Base2Valid ? "ok" : "off", lim2 / 100, lim2 % 100);
}
MSH_CMD_EXPORT_ALIAS(PressBaseSet_Cmd, PressBaseSet, "Manually set nominal pressure (0.01MPa, 0=off)");

/********************************************************************************************
*msh命令: LifeDbg — 查看寿命/卡滞检测状态
*********************************************************************************************/
static void LifeDbg_Cmd(int argc, char **argv)
{
    rt_kprintf("Base    P1=%u.%02u MPa(%s)  P2=%u.%02u MPa(%s)\n",
               s_Base1 / 100, s_Base1 % 100, s_Base1Valid ? "ok" : "--",
               s_Base2 / 100, s_Base2 % 100, s_Base2Valid ? "ok" : "--");
    rt_kprintf("RunPeak P1=%u.%02u  P2=%u.%02u   LastPeak P1=%u.%02u  P2=%u.%02u\n",
               s_RunPeak1 / 100, s_RunPeak1 % 100, s_RunPeak2 / 100, s_RunPeak2 % 100,
               s_LastPeak1 / 100, s_LastPeak1 % 100, s_LastPeak2 / 100, s_LastPeak2 % 100);
    rt_kprintf("Cycle=%u  Alarm=%u (0正常 1锈蚀/老化 2泄漏 3过载)\n",
               tLife.CycleCount, tLife.AlarmFlag);
    rt_kprintf("Protect dir=%u(%s)  TripPeak P1=%u.%02u P2=%u.%02u MPa  OverloadCnt=%u\n",
               g_fault_dir,
               (g_fault_dir == Motor_FaultDir_Open)  ? "开阀/CW" :
               (g_fault_dir == Motor_FaultDir_Close) ? "关阀/CCW" : "无/停机",
               s_TripPeak1 / 100, s_TripPeak1 % 100,
               s_TripPeak2 / 100, s_TripPeak2 % 100,
               tLife.OverloadCount);
}
MSH_CMD_EXPORT_ALIAS(LifeDbg_Cmd, LifeDbg, "Show life/stuck detection state");

/********************************************************************************************
*msh命令: ProtectReset — 清除过载故障码/解除停机保持
*备注   : 目标角度仍保持当前角度; 压力若仍超限, 下一圈会立即再次停机报故障
*********************************************************************************************/
static void ProtectReset_Cmd(int argc, char **argv)
{
    Motor_ProtectReset();
    Pressure_OverloadReportReset();      /* 复位后下次过载重新报3次 */
    tLife.AlarmFlag = Pressure_Normal;
    rt_kprintf("[PROT] fault cleared, target held at %u deg\n", g_target);
}
MSH_CMD_EXPORT_ALIAS(ProtectReset_Cmd, ProtectReset, "Clear overload fault/hold (motor stays)");





