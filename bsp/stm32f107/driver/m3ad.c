#include <rtthread.h>
#include <stm32f10x_adc.h> 
#include <stm32f10x_dma.h>
#include <stdio.h>

#include "finsh.h"
#ifdef RT_USING_M3AD
#include "../uploadapp.h"
#include "m3ad.h"	
#if USE_ADC_DMA		
// ע��ADCΪ12λģ��ת������ֻ��ADCConvertedValue�ĵ�12λ��Ч
__IO uint16_t advalues[M3ADC_CHANNELS*MEAN_TIME];

#endif

void m3ad_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;      //ADC��ʼ���ṹ������ 
#if USE_ADC_DMA
	DMA_InitTypeDef DMA_InitStructure;      //DMA��ʼ���ṹ������
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

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;    //DMA��Ӧ���������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&advalues;   //�ڴ�洢����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA��ת��ģʽΪSRCģʽ����������Ƶ��ڴ�
	DMA_InitStructure.DMA_BufferSize = M3ADC_CHANNELS*MEAN_TIME;		   //DMA�����С��1��,��λΪDMA_MemoryDataSize
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//����һ�����ݺ��豸��ַ��ֹ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�رս���һ�����ݺ�Ŀ���ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //�����������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA�����ݳߴ磬HalfWord����Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //ת��ģʽ��ѭ������ģʽ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA���ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2Mģʽ����
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
  
	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
#endif
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	/* Enable ADC1 and GPIOC clock */
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);	

	/* Configure PC.00 (ADC Channel10) as analog input -------------------------*/
	//PC0 ��Ϊģ��ͨ��10��������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //������ת��ģʽ
#if USE_ADC_DMA
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //����ɨ��ģʽ
#else
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		  //�ر�ɨ��ģʽ
#endif
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //��������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC�ⲿ���أ��ر�״̬
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //���뷽ʽ,ADCΪ12λ�У��Ҷ��뷽ʽ
	ADC_InitStructure.ADC_NbrOfChannel = M3ADC_CHANNELS;	 //����ͨ������4��
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel10 configuration ADCͨ���飬 ��10��ͨ�� ����˳��1��ת��ʱ�� */ 	 	
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
	ADC_DMACmd(ADC1, ENABLE);	  //ADC DMA���ʹ��	 
#else
	ADC_DMACmd(ADC1,DISABLE);	//ADC DMA 
#endif
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);  //����ADC1
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);	  //����У׼
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));  //�ȴ�����У׼���
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);		//��ʼУ׼
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));	   //�ȴ�У׼���

#if USE_ADC_DMA

#else	
	//�����ж�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);														  
#endif
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//����ת����ʼ��ADCͨ��DMA��ʽ���ϵĸ���RAM����
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
	unsigned short Valuetemp=ADC_GetConversionValue(ADC1);   //�������һ��ADC1��ת�����   �������������һ��ֻ��һ��ͨ������ô���صľ���Ҫ���ֵ�������ɨ��7��ͨ���أ����������ôȡֵ��                                
	rt_kprintf("M3AD Value %x\n",Valuetemp);
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);//����жϱ�־
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

