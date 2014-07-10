#include <rtthread.h>  
#include "stm32f10x.h"
#include <stm32f10x_spi.h>
#include <stm32f10x_dma.h>
#include <finsh.h>
#include <stdio.h>
#include "../applications/uploadapp.h"

#define AD7606_SPI	SPI3
#define AD_SPI_RCC	RCC_APB1Periph_SPI3

#define DMA_SPI3
#define AD_USE_PWM

#ifdef DMA_SPI3

static volatile unsigned short ad_dma_sd=0;
#pragma pack(push)
#pragma pack(16)
volatile unsigned short ad_dma_buf[POOLSIZE][AD_TIMES*AD_CHS]={0};
#pragma pack(pop)
#endif

#define AD_OS0_PORT		GPIOD
#define AD_OS0_PIN		GPIO_Pin_3

#define AD_OS1_PORT		GPIOD
#define AD_OS1_PIN		GPIO_Pin_4

#define AD_OS2_PORT		GPIOD
#define AD_OS2_PIN		GPIO_Pin_7

#define AD_CVA_PORT		GPIOC
#define AD_CVA_PIN		GPIO_Pin_6

#define LP_CH_PORT		GPIOD
#define LP_CH1_PIN		GPIO_Pin_12
#define LP_CH2_PIN		GPIO_Pin_13
#define LP_CH3_PIN		GPIO_Pin_14
#define LP_CH4_PIN		GPIO_Pin_15

#define AD_SCK_PORT		GPIOC
#define AD_SCK_PIN		GPIO_Pin_10

#define AD_RST_PORT		GPIOD
#define AD_RST_PIN		GPIO_Pin_2

#define AD_BUSY_PORT	GPIOD
#define AD_BUSY_PIN		GPIO_Pin_1

#define AD_CS_PORT		GPIOC
#define AD_CS_PIN		GPIO_Pin_12

#define AD_FIRST_PORT	GPIOD
#define AD_FIRST_PIN	GPIO_Pin_0

#define AD_MISO_PORT	GPIOC
#define AD_MISO_PIN		GPIO_Pin_11

#define ADBUSY_PORTSOURCE	GPIO_PortSourceGPIOD
#define ADBUSY_PINSOURCE	GPIO_PinSource1
#define ADBUSY_EXTI_IRQn	EXTI1_IRQn
#define ADBUSY_EXTI_LINE	EXTI_Line1

const unsigned short lp_values[]={4000,2000,800,400,200,80,40,20,8,4};

int curgear = 0;

void ad_read(unsigned short * buf,int len)                                        //SPI��д���ݺ���
{
	int i = 0 ;	
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_RESET);
	for(i=0 ;i < len ;i++){
		//u8 retry=0;	
		//retdata <<= 8;
		//while (SPI_I2S_GetFlagStatus(AD7606_SPI, SPI_I2S_FLAG_TXE) == RESET)      //���ͻ����־λΪ��
		//{
		//	retry++;
		//	if(retry>200) {
		//		GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET);
		//		return ;
		//	}
		//}			  
		/* Send byte through the SPI1 peripheral */
		SPI_I2S_SendData(AD7606_SPI, 0);                                    //ͨ������SPI1����һ������
		//retry=0;
		/* Wait to receive a byte */
		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(AD7606_SPI, SPI_I2S_FLAG_RXNE) == RESET);   //���ջ����־λ��Ϊ��
		
									    
		/* Return the byte read from the SPI bus */
		//retdata |=  SPI_I2S_ReceiveData(AD7606_SPI);                                 //ͨ��SPI3���ؽ�������				    
		buf[i] =  SPI_I2S_ReceiveData(AD7606_SPI);                                 //ͨ��SPI3���ؽ�������				    
		
	}
	
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET);	
	return ;
}



#ifdef AD_USE_PWM
long ad_timers_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOC clock enable */
	//GPIOBʹ�õ�RCCʱ��ʹ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD |RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	

	//����ʹ�õ�GPIO
	GPIO_InitStructure.GPIO_Pin = AD_CVA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AD_CVA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LP_CH1_PIN | LP_CH2_PIN | LP_CH3_PIN | LP_CH4_PIN  ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LP_CH_PORT, &GPIO_InitStructure);
	
	return 0;
}
extern int udps;
long set_gear(int gear)
{
	 TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	 TIM_OCInitTypeDef 	 TIM_OCInitStructure;

	 if(gear < 0 || gear > 9)
	 	return curgear;		
	 rt_kprintf("set gear %d\n",gear);
	 //���ò���Ƶ��
	//�����ڲ�ʱ�Ӹ�TIM3�ṩʱ��Դ
	//TIM_InternalClockConfig(TIM3);
	TIM_Cmd(TIM3,DISABLE);
	TIM_Cmd(TIM4,DISABLE);

	// Time base configuration 
	TIM_TimeBaseStructure.TIM_Period = (lp_values[gear]-1); //1000us ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 80K
	TIM_TimeBaseStructure.TIM_Prescaler = (703-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	// Output Compare Active Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = (lp_values[gear]-1)/2; //���ô�װ�벶��ȽϼĴ���������ֵ������ռ�ձ�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIMx��CCR3�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���


	//���ÿ������ֹƵ��
	// Time base configuration 
	TIM_TimeBaseStructure.TIM_Period = (lp_values[gear]-1);//(18-1); //1000us ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 80K
	TIM_TimeBaseStructure.TIM_Prescaler = (18-1);//(lp_values[gear]-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	// Output Compare Active Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = (lp_values[gear]-1)/2; //���ô�װ�벶��ȽϼĴ���������ֵ������ռ�ձ�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIMx��CCR3�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM4,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	curgear = gear;	
	if(udps!=1)
		udps=1;  
	return 0;
}
#endif
#ifdef DMA_SPI3
/*******************************************************************************
* Function Name  : SPI3_DMA_Configuration
* Description    : ����SPI3_RX��DMA2ͨ��1��SPI3_TX��DMA2ͨ��2
* Input          : None
* Output         : None
* Return         : None
* Attention             : 
*******************************************************************************/
void SPI3_DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
    
    /* DMA2 Channel1 (triggered by SPI3 Rx event) Config */
	DMA_DeInit(DMA2_Channel1);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;                  //���� SPI1 ��������(0x4001300C) ��ַ(Ŀ�ĵ�ַ)
	//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ad_dma_buf;                    //���� SRAM �洢��ַ(Ŀ�ĵ�ַ)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                                //���䷽�� ����-�ڴ�
	DMA_InitStructure.DMA_BufferSize = AD_CHS;                         //���� SPI1 ���ͳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Channel1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Channel1, DMA_IT_HT, DISABLE); 
	DMA_ITConfig(DMA2_Channel1, DMA_IT_TE, ENABLE); 
	
	
	
	/* DMA2 Channel2 (triggered by SPI3 Tx event) Config */
	DMA_DeInit(DMA2_Channel2);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;                    //����  ��������(0x4001300C) ��ַ(Դ��ַ)
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ad_dma_sd;                    //���� SRAM �洢��ַ(Դ��ַ)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                                //���䷽�� �ڴ�-����
	DMA_InitStructure.DMA_BufferSize = AD_CHS;                           //���� SPI1 ���ճ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                  //�����ַ����(����)
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;                           //�ڴ��ַ����(����)
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;           //���贫����(�ֽ�)
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;                   //�ڴ洫����(�ֽ�)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                     //���䷽ʽ,һ�δ�����ֹͣ,�����¼���
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                           //�жϷ�ʽ-��(����)
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                      //�ڴ浽�ڴ淽ʽ��ֹ
	DMA_Init(DMA2_Channel2, &DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Channel2, DMA_IT_TC, DISABLE);                                   //�ر� DMA2_Channel2 ��������ж�
	DMA_ITConfig(DMA2_Channel2, DMA_IT_TE, ENABLE);                                   //���� DMA2_Channel2 ��������ж�
	DMA_ITConfig(DMA2_Channel2, DMA_IT_HT, DISABLE);                                   //���� DMA2_Channel2 ���������ж�

}


rt_inline void DMA_SPI_Start(__IO unsigned short * addr )
{	
	//DMA_Cmd(DMA2_Channel2, DISABLE);                                                  //ֹͣ DMA ͨ�� DMA1_Channel3
	DMA2_Channel2->CCR 	&= (uint16_t)(~DMA_CCR1_EN);
	//DMA_Cmd(DMA2_Channel1, DISABLE);
	DMA2_Channel1->CCR 	&= (uint16_t)(~DMA_CCR1_EN);
 
    /* DMA2 Channel1 (triggered by SPI3 Rx event) Config */
	//DMA_SetCurrDataCounter(DMA2_Channel1,0);
	//DMA2_Channel1->CNDTR = 0;
	//DMA_SetCurrDataCounter(DMA2_Channel1,AD_CHS);
	DMA2_Channel1->CNDTR = AD_CHS;
    DMA2_Channel1->CMAR = (uint32_t)addr;
	
	/* DMA2 Channel2 (triggered by SPI3 Tx event) Config */
	//DMA_SetCurrDataCounter(DMA2_Channel2,0);
	//DMA2_Channel2->CNDTR = 0;
	//DMA_SetCurrDataCounter(DMA2_Channel2,AD_CHS);
	DMA2_Channel2->CNDTR = AD_CHS;
	

	//DMA_Cmd(DMA2_Channel2, ENABLE);  												   //���� DMA ͨ�� DMA1_Channel3 
	DMA2_Channel2->CCR |= DMA_CCR1_EN;                                                 
	//DMA_Cmd(DMA2_Channel1, ENABLE);
	DMA2_Channel1->CCR |= DMA_CCR1_EN;
}
#endif
void ad7606_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
	SPI_InitTypeDef  SPI_InitStructure;

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(AD_SPI_RCC, ENABLE);		//ʹ��SPI3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);    //ʹ��GPIO��ʱ��
#ifdef 	 AD_USE_PWM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
#endif
#ifdef DMA_SPI3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	SPI3_DMA_Configuration();
#endif
	GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);

	//����ź�
	GPIO_InitStructure.GPIO_Pin = AD_OS0_PIN;          //������ OS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AD_OS0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS1_PIN;          //������ OS1
	GPIO_Init(AD_OS1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS2_PIN;
	GPIO_Init(AD_OS2_PORT, & GPIO_InitStructure);

#ifndef   AD_USE_PWM
	GPIO_InitStructure.GPIO_Pin = AD_CVA_PIN;          //��������
	GPIO_Init(AD_CVA_PORT, &GPIO_InitStructure);
#endif
	
	GPIO_InitStructure.GPIO_Pin = AD_RST_PIN;          //AD����
	GPIO_Init(AD_RST_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_CS_PIN;          //ʹ��
	GPIO_Init(AD_CS_PORT, &GPIO_InitStructure);
										
	GPIO_InitStructure.GPIO_Pin = AD_SCK_PIN;          //SPI ʱ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD_SCK_PORT, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = AD_MISO_PIN;          //����
	GPIO_Init(AD_MISO_PORT, &GPIO_InitStructure);


	//���¿�ʼ�����ź�
	GPIO_InitStructure.GPIO_Pin = AD_BUSY_PIN;          //�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ; 
	GPIO_Init(AD_BUSY_PORT, &GPIO_InitStructure);
	GPIO_WriteBit(AD_BUSY_PORT,AD_BUSY_PIN,Bit_RESET);

	GPIO_InitStructure.GPIO_Pin = AD_FIRST_PIN;          //���ֽ�
	GPIO_Init(AD_FIRST_PORT, &GPIO_InitStructure);

	//��ʼ����ֵ 
	//GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);    //���ﲻ�ø�����Ϊ����ڴ����ûᵼ���ϵ�ͻ����һ��AD����
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET); 

	//û�й�����
	GPIO_WriteBit(AD_OS0_PORT, AD_OS0_PIN, Bit_RESET);
	GPIO_WriteBit(AD_OS1_PORT, AD_OS1_PIN, Bit_SET);
	GPIO_WriteBit(AD_OS2_PORT, AD_OS2_PIN, Bit_RESET); //010 ���������50kHz

	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_RESET);	 


	//���¿�ʼSPI����
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊһ�ߵ���
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                //����SPIΪ��ģʽ
#ifdef DMA_SPI3
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		            //SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //SPI������Ԥ��ƵֵΪ8
#else
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		            //SPI���ͽ���16λ֡�ṹ
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //SPI������Ԥ��ƵֵΪ2
#endif
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                    //����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	                    //��һ��ʱ���ؿ�ʼ��������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                    //NSS�ź��������ʹ��SSIλ������
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                        //CRCֵ����Ķ���ʽ

	SPI_Init(AD7606_SPI, &SPI_InitStructure);   //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPI3�Ĵ���
#ifdef DMA_SPI3
	/* Enable SPI1 DMA RX request */
	SPI_I2S_DMACmd(AD7606_SPI,SPI_I2S_DMAReq_Rx,ENABLE);					 
	//SPI3->CR2 |= 1<<0;                                                                 //���ջ�����DMAʹ��
	/* Enable SPI1 DMA TX request */
	SPI_I2S_DMACmd(AD7606_SPI,SPI_I2S_DMAReq_Tx,ENABLE);					 
	//SPI3->CR2 |= 1<<1;																   //���ͻ�����DMAʹ��
#endif
	/* Enable SPI3  */
	SPI_Cmd(AD7606_SPI, ENABLE);                                      //ʹ��SPI����

	//�жϳ�ʼ��
	GPIO_EXTILineConfig(ADBUSY_PORTSOURCE, ADBUSY_PINSOURCE);
	EXTI_InitStructure.EXTI_Line = ADBUSY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	//�ж�������ʼ��	
	EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

#ifdef DMA_SPI3
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
#endif

	NVIC_InitStructure.NVIC_IRQChannel = ADBUSY_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

}

void ad_reset(void)
{
	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_SET);
	rt_thread_delay(RT_TICK_PER_SECOND/1000);
	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_RESET);
}

long ad_start(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//���½�Timer����Ϊȱʡֵ
	//TIM_DeInit(TIM3);
	TIM_Cmd(TIM3,DISABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);

	GPIO_InitStructure.GPIO_Pin = AD_CVA_PIN;          //��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(AD_CVA_PORT, &GPIO_InitStructure);

	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_RESET);
	rt_thread_delay(RT_TICK_PER_SECOND/1000);
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);

	return 0;
}

#ifdef DMA_SPI3

void EXTI1_IRQHandler(void) /* AD Data ok */
{
  static rt_uint32_t datacount = 0;
  static rt_uint32_t bufindex = 0;
  //if(EXTI_GetITStatus(ADBUSY_EXTI_LINE) != RESET)
  //uint32_t enablestatus = 0;
  //enablestatus =  EXTI->IMR & ADBUSY_EXTI_LINE;
  //if (((EXTI->PR & ADBUSY_EXTI_LINE) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
  if ((AD_BUSY_PORT->IDR & AD_BUSY_PIN) == (uint32_t)Bit_RESET)
  {
  	//GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_RESET);
		
	AD_CS_PORT->BRR = AD_CS_PIN;
	
  	DMA_SPI_Start(((short *)ad_dma_buf[bufindex])+datacount*AD_CHS);
	if(++datacount >= AD_TIMES){
		rt_mb_send(&fullmb,bufindex);
	 	//rt_mb_recv(&emptymb,&bufindex,RT_WAITING_FOREVER);
		bufindex++;
		if(bufindex >= POOLSIZE)
			bufindex = 0;
		datacount = 0;
	}
    /* Clear the EXTI Line 1 */
    EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);
  }
}

void DMA2_Channel2_IRQHandler(void)
{
 	if(DMA_GetFlagStatus(DMA2_FLAG_TC2) == SET){
		//rt_kprintf("DMA2 Tx Complete\n");
	 	DMA_ClearITPendingBit(DMA2_IT_TC2);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_TE2) == SET){
		rt_kprintf("!!!!DMA2 Tx Error\n");
	 	DMA_ClearITPendingBit(DMA2_IT_TE2);
		GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_HT2) == SET){
		//rt_kprintf("DMA2 Tx Half\n");
	 	DMA_ClearITPendingBit(DMA2_IT_HT2);
	}
}
long ad_pt(void)
{
	int i = 0 ;
	rt_kprintf("ad data:\n");
	for(i=0;i<AD_CHS;i++){
		//printf("channel %d : %04x  %d %fV\n",i,ad_dma_buf[i],(short)ad_dma_buf[i],(((short)ad_dma_buf[i])*10.0/32768.0));
	}
	return 0;	
}
void DMA2_Channel1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_FLAG_TC1) == SET){
		//ad_pt();
		DMA_ClearITPendingBit(DMA2_IT_TC1);
		GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_TE1) == SET){
		rt_kprintf("!!!!DMA2 Rx Error\n");
	 	DMA_ClearITPendingBit(DMA2_IT_TE1);
		GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_HT1) == SET){
	 	DMA_ClearITPendingBit(DMA2_IT_HT1);
	}
}


#else
void EXTI1_IRQHandler(void) /* AD Data ok */
{
  if(EXTI_GetITStatus(ADBUSY_EXTI_LINE) != RESET)
  {
  	int i = 0 ;
  	unsigned short data [8] ={0};
	
	ad_read(data,8);
	rt_kprintf("ad data:\n");
	for(i=0;i<8;i++){
		printf("channel %d : %04x  %d %fV\n",i,data[i],(short)data[i],(((short)data[i])*10.0/32768.0));
	}	
	//rt_kprintf("\n");    
    ad_reset();
    /* Clear the EXTI Line 4 */
    EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);
  }
}
#endif
#ifdef FINSH_USING_SYMTAB
//FINSH_VAR_EXPORT(ad_mode, finsh_type_int, set ad spi mode 1 for software 0 for hardware)
#endif

#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(ad_start, start ad7606 convert);
#ifdef AD_USE_PWM
FINSH_FUNCTION_EXPORT(set_gear, set ad pwm gear);
#endif
#ifdef DMA_SPI3
FINSH_FUNCTION_EXPORT(ad_pt, print ad7606 convert result);
#endif
#endif

