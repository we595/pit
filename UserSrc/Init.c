#include "init.h"
#include "motor.h"
#include "define.h"
#include "RS485.h"    /* RS485_ServiceInit: 启动485服务线程 */


/******************************************************************** 
*函数名称: DAC_Init
*函数功能: DAC初始化
*参数: 无
*返回值: 无
********************************************************************/
static DAC_HandleTypeDef DAC_Handle = {0};

void DAC_Init(void)
{
  /***************************************************************** */
    //使能DAC1，与GPIO时钟
	 __HAL_RCC_DAC1_CLK_ENABLE();
     __HAL_RCC_GPIOA_CLK_ENABLE();

     //配置GPIOA的PA4
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.Pin = GPIO_PIN_4;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
    /****************************DAC配置************************************* */ 
    
    DAC_Handle.Instance = DAC1;
    HAL_DAC_Init(&DAC_Handle);

    /************************通道配置1***************************************** */ 
    DAC_ChannelConfTypeDef DAC_ChannelConf = {0};  
    DAC_ChannelConf.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    DAC_ChannelConf.DAC_Trigger = DAC_TRIGGER_NONE;
    DAC_ChannelConf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    DAC_ChannelConf.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    HAL_DAC_ConfigChannel(&DAC_Handle, &DAC_ChannelConf, DAC_CHANNEL_1);

    /*************************校准**************************************/
    HAL_DACEx_SelfCalibrate(&DAC_Handle, &DAC_ChannelConf, DAC_CHANNEL_1);

    /*****************************启动+先输出0v*******************************/ 
    HAL_DAC_Start(&DAC_Handle, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&DAC_Handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
   

}

/***************************************************************** 
*函数名称: DAC_SetValue
*函数功能: 设置DAC输出电压
*参数: 无
*返回值: 无
*备注：dac用的是12位的，
********************************************************************/
void DAC_SetValue(float VoltageIn)
{
  uint32_t VoltageOut;
  
  if(VoltageIn < 0.0f) VoltageIn = 0.0f;
  if(VoltageIn > 3.3f) VoltageIn = 3.3f;
	
  VoltageOut = (VoltageIn/3.3f)*4095.0f;
  HAL_DAC_SetValue(&DAC_Handle,DAC_CHANNEL_1,DAC_ALIGN_12B_R,VoltageOut);
	
}

/**************************************************************** 
*函数名称: DAC_Test
*函数功能: DAC测试
********************************************************************/
static void DAC_Test(int WordNum , char *Word[])
{
	float Temp_Voltage; 
	
   if (WordNum < 2) 
   {
	   rt_kprintf("Please Input: DAC <0-3.3V>");
       return;
   }
   
   if (WordNum == 2)
   {
       Temp_Voltage = (float)atof(Word[1]);
	   DAC_SetValue(Temp_Voltage);
	   
	   rt_kprintf("DAC = %d.%02d V\n", (int)Temp_Voltage, (int)(Temp_Voltage * 100) % 100);
   
   }
   
   if (WordNum > 2)
   {
     rt_kprintf("Input ERR Please Input Word Num = 2");
	  return; 
   
   }
}

/******************************************************************************
操作系统命令添加函数，具体如何作用参考下列解析
MSH_CMD_EXPORT_ALIAS(DAC_Test, dac, set dac output voltage 0-3.3V);
                    ①函数名  ②命令名 ③帮助文字(第3个参数)
msh /> help
...
dac: set dac output voltage 0-3.3V   ← 你写的那句就显示在这
motor: motor control ...
...

*********************************************************************************/

//DAC命令行测试
MSH_CMD_EXPORT_ALIAS(DAC_Test,DAC,set dac output voltage 0-3.3V);

/**********************************************************************************
*函数名称：ADC_Init
*功能    ；ADC初始化
*备注    ：PC2(ADC1,2,3)-IN3(press1),
PC4(ADC1,2)-IN13（Press2）,VBAT内部通道（power电源检测，接VCC_3.3，不占IO）
用VBAT引脚真實值為：真实电压 = 读数 × 3 × 3.3 / 4095
*************************************************************************************/
static  ADC_HandleTypeDef ADC_Handle = {0};
void ADC_Init(void)
{
	
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure = {0};
  GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_4;   /* PC5不再配模拟输入(它是SDIO/WiFi线) */
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC_Handle.Instance = ADC1;
    ADC_Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	ADC_Handle.Init.Resolution = ADC_RESOLUTION_12B;
    ADC_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    ADC_Handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    ADC_Handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;//全部读完
    ADC_Handle.Init.LowPowerAutoWait = DISABLE;   //禁用低功耗自动等待功能
    ADC_Handle.Init.ContinuousConvMode = ENABLE;//扫描完一次就停止
    ADC_Handle.Init.NbrOfConversion = 3;
    ADC_Handle.Init.DiscontinuousConvMode = DISABLE; //整个 3 通道序列在一次触发后连续转换完
    ADC_Handle.Init.NbrOfDiscConversion   = 0;
    ADC_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;//软件触发
    ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//禁用外部触发
    ADC_Handle.Init.DMAContinuousRequests = ENABLE; //使能DMA
    ADC_Handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;//如果溢出新数据覆盖老数据
    ADC_Handle.Init.OversamplingMode = DISABLE;//过采样禁用
    HAL_ADC_Init(&ADC_Handle);

    //校准ADC
    HAL_ADCEx_Calibration_Start(&ADC_Handle, ADC_SINGLE_ENDED);
	
	/*采样通道（这一块配置需要去数据手册查下对应的INx）*/
	ADC_ChannelConfTypeDef SConfing = {0};
	SConfing.SamplingTime = ADC_SAMPLETIME_24CYCLES_5; //24.5个周期
	SConfing.SingleDiff = ADC_SINGLE_ENDED;    //用单端模式
	
	//PC2通道的配置
	SConfing.Channel = ADC_CHANNEL_3;
	SConfing.Rank = 1;      
	HAL_ADC_ConfigChannel(&ADC_Handle,&SConfing);
	
	//PC4
	SConfing.Channel = ADC_CHANNEL_13;
	SConfing.Rank = 2;
	HAL_ADC_ConfigChannel(&ADC_Handle,&SConfing);
	
	//VBAT内部通道（接VCC_3.3，电源检测用，不占外部IO；内部1/3分压，读数×3才是真实电压）
	SConfing.Channel = ADC_CHANNEL_VBAT;
	SConfing.Rank = 3;
	SConfing.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;  //VBAT内部路径阻抗高，用最大采样时间
	HAL_ADC_ConfigChannel(&ADC_Handle,&SConfing);
} 

/************************************************************************************
*函数名称:Usart2_Init
*作用    :用于485通信模块初始化
*备注1    :PA2_TX_Usart2,PA3_RX_Usart2,查数据手册这俩io口复用AF7
*备注2     :串口的数据位、停止位、校验位、波特率这组参数，
必须和总线上其他设备（角度传感器、上位机）完全一致 .
Modbus Rtu一般规定:数据位8位、停止位1位、校验位无、波特率115200或者9600
Modbus RTU 的校验不在串口层，而是每帧末尾带 CRC16
*备注3     : 开发板上丝印的 TXD对应PA2,连接时别搞错了
***************************************************************************************/
static UART_HandleTypeDef USART2_Handle = {0};
void Usart2_Init(void)
{
 __HAL_RCC_USART2_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();

 //PA2
 GPIO_InitTypeDef GPIO_InitStructure = {0};
 GPIO_InitStructure.Pin = GPIO_PIN_2 ;
 GPIO_InitStructure.Pull = GPIO_NOPULL;
 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
 GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

 //PA3
  GPIO_InitStructure.Pin = GPIO_PIN_3 ;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART2_Handle.Instance = USART2;
  USART2_Handle.Init.BaudRate = BAUD_RATE_9600;
  USART2_Handle.Init.WordLength = UART_WORDLENGTH_8B;//数据位8位
  USART2_Handle.Init.StopBits = UART_STOPBITS_1;  //停止位1位
  USART2_Handle.Init.Parity = UART_PARITY_NONE;  //无校验
  USART2_Handle.Init.Mode = UART_MODE_TX_RX;
  USART2_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;  //无硬件流控(没有RTS/CTS)引脚,有的话就是5线串口
  USART2_Handle.Init.OverSampling = UART_OVERSAMPLING_8; //每位采样8次
  HAL_UART_Init(&USART2_Handle);

  /* 帧接收完成事件(信号量)必须先就绪, 再打开 IDLE 中断 */
  RS485_RxEventInit();

  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);//中断号,抢占优先级,响应优先级
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  __HAL_UART_ENABLE_IT(&USART2_Handle, UART_IT_RXNE);  //使能接收中断
  __HAL_UART_ENABLE_IT(&USART2_Handle, UART_IT_IDLE);  //使能总线空闲(帧结束)中断
}

/*************************************************************************** 
 * 函数名称 : USART2_IRQHandler
 * 功能描述 : 串口2中断服务函数,每接收一个字节就会进入一次中断
***************************************************************************/
volatile uint8_t RX_Flag = 0;
void USART2_IRQHandler(void)
{
  rt_interrupt_enter();      /* 进入中断上下文(使用 RT-Thread IPC 必须) */

  /* ① 收一个字节 */
  if (__HAL_UART_GET_FLAG(&USART2_Handle, UART_FLAG_RXNE) != RESET)  //接收中断,接收寄存器有数据,SET代表1,有数据
  {
    RX_Flag = 1;
    tUsart2_RX.Data[tUsart2_RX.Index++] = (uint8_t)(USART2->RDR & 0xff);  //读RDR同时清RXNE,并存入接收缓冲区
    if (tUsart2_RX.Index >= sizeof(tUsart2_RX.Data))  //防止数组越界
    {
      tUsart2_RX.Index = 0;
    }
  }

  /* ② IDLE: 总线空闲 = 一帧接收完成 → 唤醒 RS485 服务线程 */
  if (__HAL_UART_GET_FLAG(&USART2_Handle, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&USART2_Handle);
    RS485_RxFrameDone();
  }

  rt_interrupt_leave();      /* 退出中断; 需要切换时在中断返回时完成 */
}

//标志位函数
uint8_t  RxFlag(void)
{
  if (RX_Flag == 1)
  {
	 RX_Flag = 0; 
    return 1;
  
  }
  return 0;

}	

/****************************************************************************
 * 函数名称:USART3_Init
 * 功能    :用于上位机与mcu的通信
 * 备注1    :PB10_TX_Usart3,PB11_RX_Usart3,查数据手册这俩io口复用AF7
******************************************************************************/
struct rt_ringbuffer usart3_rb;
static uint8_t usart3_rb_buf[64];   /* 环形缓冲 64 字节 */

static UART_HandleTypeDef USART3_Handle = {0};
void Usart3_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    /* PB10 TX */
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* PB11 RX */
    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART3_Handle.Instance = USART3;
    USART3_Handle.Init.BaudRate = BAUD_RATE_9600;   /* 和上位机一致 */
    USART3_Handle.Init.WordLength = UART_WORDLENGTH_8B;
    USART3_Handle.Init.StopBits = UART_STOPBITS_1;
    USART3_Handle.Init.Parity = UART_PARITY_NONE;
    USART3_Handle.Init.Mode = UART_MODE_TX_RX;
    USART3_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    USART3_Handle.Init.OverSampling = UART_OVERSAMPLING_8;
    HAL_UART_Init(&USART3_Handle);

    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&USART3_Handle, UART_IT_RXNE);
	
	  /* 环形缓冲初始化(只调一次, 别放中断!),将usart3_rb_buf[16]这个数组放到这个usart3_rb结构体内 */
	rt_ringbuffer_init(&usart3_rb,usart3_rb_buf, sizeof(usart3_rb_buf)); 
}

/*************************************************************************** 
 * 函数名称 : USART3_IRQHandler
 * 功能描述 : 串口3中断服务函数,每接收一个字节就会进入一次中断
 * 备注     :使用了环形缓冲区来接收来自上位机的数据
***************************************************************************/

void USART3_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&USART3_Handle, UART_FLAG_RXNE) != RESET)
  {
    uint8_t Data = (uint8_t)(USART3->RDR & 0xff);  /* 读取接收寄存器数据 */
    rt_ringbuffer_putchar(&usart3_rb, Data);
     tUsart3_RX.Tick = rt_tick_get();
  }

}

/******************************************************************************************
*函数名称：DMA_Init
*功能    ：将adc数据搬到tdma结构体内
*备注    ：查数据手册用DMA1，通道一进行对adc1数据搬运他的请求号是0000，在数据手册339页
*附加函数：HAL_DMA_IRQHandler
*********************************************************************************************/
static DMA_HandleTypeDef DMA_ADC1 = {0};
void DMA_Init(void)
{
   __HAL_RCC_DMA1_CLK_ENABLE();
	
  DMA_ADC1.Instance = DMA1_Channel1;
  DMA_ADC1.Init.Request = DMA_REQUEST_0;	
  DMA_ADC1.Init.Direction = DMA_PERIPH_TO_MEMORY;//外设到基地址
  DMA_ADC1.Init.PeriphInc = DMA_PINC_DISABLE;    //外设地址不自增，即使有多个ADC他们也只公用一个DR寄存器	
  DMA_ADC1.Init.MemInc = DMA_MINC_ENABLE;  //内存地址自增
  DMA_ADC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  //每次读取16位，与uint16_t结构体匹配
	DMA_ADC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;     //接收也按16位接收，之前MDA不小心寫錯為PDA	
  DMA_ADC1.Init.Mode = DMA_CIRCULAR;                         //循环搬运
  DMA_ADC1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&DMA_ADC1);	

 //DMA与ADC双向关联
  __HAL_LINKDMA(&ADC_Handle,DMA_Handle,DMA_ADC1);

//中断设置
HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,0,0);//优先级
HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);    //使能

HAL_ADC_Start_DMA(&ADC_Handle,(uint32_t *)&tDMA , 3);	//放到这个结构体内	
}

/**************************************************************************/
volatile uint8_t ADC_Flag = 0;
void DMA1_Channel1_IRQHandler (void)
{

	HAL_DMA_IRQHandler(&DMA_ADC1);   //中断清除标志位函数
	
	ADC_Flag =1;
    
}

//标志位函数
uint8_t  ADCFlag(void)
{
  if (ADC_Flag == 1)
  {
	 ADC_Flag = 0; 
    return 1;
  
  }
  return 0;

}	




/**********************************************************************************
*函数名称：Init
*功能 ： 放置所有外设初始化函数
***********************************************************************************/
void Init(void)
{
  Motor_Gpio_Init();	
  DAC_Init();
  ADC_Init();
  DMA_Init();

 
	
  Usart2_Init();
  Usart3_Init();
  Motor_Init();	    /* ① 上电的编码器总线操作先做完(此时服务线程未启动, 无竞争) */

  RS485_ServiceInit();  /* ② 启动 RS485 服务线程: 从此由它独占 USART2 */	
}





