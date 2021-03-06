#include <rtthread.h>
#include <stm32f10x_adc.h> 
#include <stm32f10x_dma.h>
#include <stdio.h>

#include "finsh.h"
#ifdef RT_USING_M3AD
#include "../applications/uploadapp.h"
#include "m3ad.h"	
#if USE_ADC_DMA		
// 注：ADC为12位模数转换器，只有ADCConvertedValue的低12位有效
__IO uint16_t advalues[M3ADC_CHANNELS*MEAN_TIME];

#endif

void m3ad_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;      //ADC初始化结构体声明 
#if USE_ADC_DMA
	DMA_InitTypeDef DMA_InitStructure;      //DMA初始化结构体声明
#else	
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	 ADC_DeInit(ADC1); 
#if USE_ADC_DMA								
	/* Enable DMA1 clock */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	memset((void*)advalues,0,sizeof(advalues));

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;    //DMA对应的外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&advalues;   //内存存储基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA的转换模式为SRC模式，由外设搬移到内存
	DMA_InitStructure.DMA_BufferSize = M3ADC_CHANNELS*MEAN_TIME;		   //DMA缓存大小，1个,单位为DMA_MemoryDataSize
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//接收一次数据后，设备地址禁止后移
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//关闭接收一次数据后，目标内存地址后移
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //定义外设数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA搬数据尺寸，HalfWord就是为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //转换模式，循环缓存模式。
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2M模式禁用
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
  
	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
#endif
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	/* Enable ADC1 and GPIOC clock */
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);	

	/* Configure PC.00 (ADC Channel10) as analog input -------------------------*/
	//PC0 作为模拟通道10输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //独立的转换模式
#if USE_ADC_DMA
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //开启扫描模式
#else
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		  //关闭扫描模式
#endif
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //开启连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC外部开关，关闭状态
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //对齐方式,ADC为12位中，右对齐方式
	ADC_InitStructure.ADC_NbrOfChannel = M3ADC_CHANNELS;	 //开启通道数，4个
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel10 configuration ADC通道组， 第10个通道 采样顺序1，转换时间 */ 	 	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
#if USE_ADC_DMA
#if M3ADC_CHANNELS > 1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5);
#endif
#if M3ADC_CHANNELS > 2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_239Cycles5);
#endif
#if M3ADC_CHANNELS > 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_239Cycles5);
#endif
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);	  //ADC DMA命令，使能	 
#else
	ADC_DMACmd(ADC1,DISABLE);	//ADC DMA 
#endif
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);  //开启ADC1
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);	  //重新校准
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));  //等待重新校准完成
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);		//开始校准
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));	   //等待校准完成

#if USE_ADC_DMA

#else	
	//设置中断向量
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);														  
#endif
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//连续转换开始，ADC通过DMA方式不断的更新RAM区。
}
#if USE_ADC_DMA
static int m3ap = 0;
static int m3af = 1;
void m3ad_print(void)
{	 
	int i = 0;
	int j = 0;
	unsigned int values[M3ADC_CHANNELS]={0};
	if(m3ap){
		printf("m3ad values: \n");
		for(i=0;i<M3ADC_CHANNELS;i++){
			for(j=0;j<MEAN_TIME;j++){
				values[i]+= advalues[j*M3ADC_CHANNELS + i];
			}
			printf("\tchannel %d: 0x%04x,%d,%.4fV\n",i,values[i]/MEAN_TIME,values[i]/MEAN_TIME,19.51 *(2.042 - (values[i]/MEAN_TIME) *3.29 / 4096)); 
		}
	}
	rt_event_send(&upeve,M3AD_EVENT);
	return;
}

static void m3ad_entry(void * parameter)
{
 	while(1){
		rt_thread_delay(RT_TICK_PER_SECOND/m3af);
		m3ad_print();
	}
}

void m3ad_th(void)
{
 	rt_thread_t m3adthread = rt_thread_create("m3ad",m3ad_entry,RT_NULL,1024,9,10);
	if(m3adthread){
	 	rt_thread_startup(m3adthread);
	}
	else{
	 	rt_kprintf("create m3ad thread failed!!!!!\n");
	}
	return;
}
#else


void ADC1_2_IRQHandler(void)
{
	unsigned short Valuetemp=ADC_GetConversionValue(ADC1);   //返回最近一次ADC1的转换结果   问题就在这里，如果一次只测一个通道，那么返回的就是要测的值，如果是扫描7个通道呢？这里面该怎么取值？                                
	rt_kprintf("M3AD Value %x\n",Valuetemp);
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);//清楚中断标志
} 
#endif

#ifdef RT_USING_FINSH
#if USE_ADC_DMA
#ifdef FINSH_USING_SYMTAB
FINSH_VAR_EXPORT(m3ap, finsh_type_int, print m3ad data)
FINSH_VAR_EXPORT(m3af, finsh_type_int, m3ad sample rate)
#endif
#endif
#ifdef FINSH_USING_SYMTAB
//FINSH_VAR_EXPORT(advalue, finsh_type_uint, value of m3ad for finsh)
#endif

#endif

#endif

