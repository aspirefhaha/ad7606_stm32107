#include <rtthread.h>  
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include <finsh.h>
#include <stdio.h>

#define AD7606_SPI	SPI3
#define AD_SPI_RCC	RCC_APB1Periph_SPI3

#define AD_OS0_PORT		GPIOD
#define AD_OS0_PIN		GPIO_Pin_3

#define AD_OS1_PORT		GPIOD
#define AD_OS1_PIN		GPIO_Pin_4

#define AD_CVA_PORT		GPIOC
#define AD_CVA_PIN		GPIO_Pin_6

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


void ad7606_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(AD_SPI_RCC, ENABLE);		//ʹ��SPI3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);    //ʹ��GPIO��ʱ��
	
	//����ź�
	GPIO_InitStructure.GPIO_Pin = AD_OS0_PIN;          //������ OS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AD_OS0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS1_PIN;          //������ OS1
	GPIO_Init(AD_OS1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_CVA_PIN;          //��������
	GPIO_Init(AD_CVA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_SCK_PIN;          //SPI ʱ��
	GPIO_Init(AD_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_RST_PIN;          //AD����
	GPIO_Init(AD_RST_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_CS_PIN;          //ʹ��
	GPIO_Init(AD_CS_PORT, &GPIO_InitStructure);

	//���¿�ʼ�����ź�
	GPIO_InitStructure.GPIO_Pin = AD_BUSY_PIN;          //�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ; 
	GPIO_Init(AD_BUSY_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_FIRST_PIN;          //���ֽ�
	GPIO_Init(AD_FIRST_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_MISO_PIN;          //����
	GPIO_Init(AD_MISO_PORT, &GPIO_InitStructure);
	//��ʼ����ֵ 
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);  
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET); 

	//û�й�����
	GPIO_WriteBit(AD_OS0_PORT, AD_OS0_PIN, Bit_RESET);
	GPIO_WriteBit(AD_OS1_PORT, AD_OS1_PIN, Bit_RESET); 

	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_RESET);	 

	//���¿�ʼSPI����
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;  //SPI����Ϊһ�ߵ���
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                //����SPIΪ��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		            //SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		                    //����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	                    //��һ��ʱ���ؿ�ʼ��������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                    //NSS�ź��������ʹ��SSIλ������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //SPI������Ԥ��ƵֵΪ2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                        //CRCֵ����Ķ���ʽ

	SPI_Init(AD7606_SPI, &SPI_InitStructure);   //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPI3�Ĵ���

	//�жϳ�ʼ��
	GPIO_EXTILineConfig(ADBUSY_PORTSOURCE, ADBUSY_PINSOURCE);
	EXTI_InitStructure.EXTI_Line = ADBUSY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	//�ж�������ʼ��
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = ADBUSY_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);

}

static unsigned char ad_readbyte(void)                                        //SPI��д���ݺ���
{		
	u8 retry=0;	
	while (SPI_I2S_GetFlagStatus(SPI_WIRELESS, SPI_I2S_FLAG_TXE) == RESET)      //���ͻ����־λΪ��
	{
		retry++;
		if(retry>200)return 0;
	}			  
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(AD7606_SPI, 0);                                    //ͨ������SPI3����һ������
	retry=0;
	/* Wait to receive a byte */
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(AD7606_SPI, SPI_I2S_FLAG_RXNE) == RESET)   //���ջ����־λ��Ϊ��
	{
		retry++;
		if(retry>200)
			break;
	}	  						    
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(AD7606_SPI);                                 //ͨ��SPI3���ؽ�������				    
}

void ad_read(unsigned char * buf,int len)
{
	int i = 0 ;
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_RESET);			 	
	for(i=0;i<len;i++){
	 	buf[i]=ad_readbyte();
	}
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET);			 	
}

void ad_reset(void)
{
	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_SET);
	rt_thread_delay(RT_TICK_PER_SECOND/1000);
	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_RESET);
}

long ad_start(void)
{
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_RESET);
	rt_thread_delay(RT_TICK_PER_SECOND/1000);
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);
	return 0;
}

void EXTI1_IRQHandler(void) /* AD Data ok */
{
  if(EXTI_GetITStatus(ADBUSY_EXTI_LINE) != RESET)
  {
  	int i = 0 ;
  	unsigned char data [16] ={0};
	
	ad_read(data,16);
	rt_kprintf("ad data:");
	for(i=0;i<16;i++){
		rt_kprintf("%02x",data[i]);
	}	
	rt_kprintf("\n");    
    
    /* Clear the EXTI Line 4 */
    EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);
  }
}

#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(ad_start, start ad7606 convert);
#endif

