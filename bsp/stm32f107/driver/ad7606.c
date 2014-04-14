#include <rtthread.h>  
#include "stm32f10x.h"
#include <stm32f10x_spi.h>
#include <stm32f10x_dma.h>
#include <finsh.h>
#include <stdio.h>

#define AD7606_SPI	SPI3
#define AD_SPI_RCC	RCC_APB1Periph_SPI3

#define DMA_SPI3

#ifdef DMA_SPI3
#define AD_TIMES	1	//缓冲区能存多少次数据
#define AD_CHS		16	//通道数

static __IO unsigned short ad_dma_buf[AD_TIMES*AD_CHS]={0};
static __IO unsigned short ad_dma_sd=0;
#endif

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

void ad_read(unsigned short * buf,int len)                                        //SPI读写数据函数
{
	int i = 0 ;	
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_RESET);
	for(i=0 ;i < len ;i++){
		//u8 retry=0;	
		//retdata <<= 8;
		//while (SPI_I2S_GetFlagStatus(AD7606_SPI, SPI_I2S_FLAG_TXE) == RESET)      //发送缓存标志位为空
		//{
		//	retry++;
		//	if(retry>200) {
		//		GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET);
		//		return ;
		//	}
		//}			  
		/* Send byte through the SPI1 peripheral */
		SPI_I2S_SendData(AD7606_SPI, 0);                                    //通过外设SPI1发送一个数据
		//retry=0;
		/* Wait to receive a byte */
		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(AD7606_SPI, SPI_I2S_FLAG_RXNE) == RESET);   //接收缓存标志位不为空
		
									    
		/* Return the byte read from the SPI bus */
		//retdata |=  SPI_I2S_ReceiveData(AD7606_SPI);                                 //通过SPI3返回接收数据				    
		buf[i] =  SPI_I2S_ReceiveData(AD7606_SPI);                                 //通过SPI3返回接收数据				    
		
	}
	
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET);	
	return ;
}
#ifdef DMA_SPI3
/*******************************************************************************
* Function Name  : SPI3_DMA_Configuration
* Description    : 配置SPI3_RX的DMA2通道1，SPI3_TX的DMA2通道2
* Input          : None
* Output         : None
* Return         : None
* Attention             : 
*******************************************************************************/
void DMA_SPI_Start( unsigned short * addr )
{
    DMA_InitTypeDef DMA_InitStructure;
    
    /* DMA2 Channel1 (triggered by SPI3 Rx event) Config */
	DMA_DeInit(DMA2_Channel1);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI3->DR;                  //设置 SPI1 发送外设(0x4001300C) 地址(目的地址)
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ad_dma_buf;                    //设置 SRAM 存储地址(目的地址)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                                //传输方向 外设-内存
	DMA_InitStructure.DMA_BufferSize = AD_TIMES * AD_CHS;                         //设置 SPI1 发送长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Channel1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Channel1, DMA_IT_HT, ENABLE); 
	DMA_ITConfig(DMA2_Channel1, DMA_IT_TE, ENABLE); 
	/* Enable SPI1 DMA RX request */					 
	SPI3->CR2 |= 1<<0;                                                                 //接收缓冲区DMA使能
	DMA_Cmd(DMA2_Channel1, ENABLE);
	
	
	/* DMA2 Channel2 (triggered by SPI3 Tx event) Config */
	DMA_DeInit(DMA2_Channel2);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI3->DR;                    //设置  接收外设(0x4001300C) 地址(源地址)
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ad_dma_sd;                    //设置 SRAM 存储地址(源地址)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                                //传输方向 内存-外设
	DMA_InitStructure.DMA_BufferSize = AD_TIMES * AD_CHS;                           //设置 SPI1 接收长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                  //外设地址增量(不变)
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;                           //内存地址增量(不变)
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;           //外设传输宽度(字节)
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                   //内存传输宽度(字节)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                     //传输方式,一次传输完停止,不重新加载
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                           //中断方式-高(三级)
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                      //内存到内存方式禁止
	DMA_Init(DMA2_Channel2, &DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Channel2, DMA_IT_TC, ENABLE);                                   //关闭 DMA2_Channel2 传输完成中断
	DMA_ITConfig(DMA2_Channel2, DMA_IT_TE, ENABLE);                                   //开启 DMA2_Channel2 传输错误中断
	DMA_ITConfig(DMA2_Channel2, DMA_IT_HT, ENABLE);                                   //开启 DMA2_Channel2 半数传输中断
	/* Enable SPI1 DMA TX request */
	SPI3->CR2 |= 1<<1;                                                                //发送缓冲区DMA使能
	//DMA_Cmd(DMA2_Channel2, ENABLE);                                                  //开启 DMA 通道 DMA1_Channel3
}
#endif
void ad7606_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
	SPI_InitTypeDef  SPI_InitStructure;

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(AD_SPI_RCC, ENABLE);		//使能SPI3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);    //使能GPIO的时钟
#ifdef DMA_SPI3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

#endif
	GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);

	//输出信号
	GPIO_InitStructure.GPIO_Pin = AD_OS0_PIN;          //过采样 OS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AD_OS0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS1_PIN;          //过采样 OS1
	GPIO_Init(AD_OS1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_CVA_PIN;          //启动采样
	GPIO_Init(AD_CVA_PORT, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = AD_RST_PIN;          //AD重置
	GPIO_Init(AD_RST_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_CS_PIN;          //使能
	GPIO_Init(AD_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_SCK_PIN;          //SPI 时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(AD_SCK_PORT, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = AD_MISO_PIN;          //数据
	GPIO_Init(AD_MISO_PORT, &GPIO_InitStructure);


	//以下开始输入信号
	GPIO_InitStructure.GPIO_Pin = AD_BUSY_PIN;          //采样完成
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ; 
	GPIO_Init(AD_BUSY_PORT, &GPIO_InitStructure);
	GPIO_WriteBit(AD_BUSY_PORT,AD_BUSY_PIN,Bit_RESET);

	GPIO_InitStructure.GPIO_Pin = AD_FIRST_PIN;          //首字节
	GPIO_Init(AD_FIRST_PORT, &GPIO_InitStructure);

	//初始化数值 
	//GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);    //这里不置高是因为如果在此设置会导致上电就会进行一次AD采样
	GPIO_WriteBit(AD_CS_PORT, AD_CS_PIN, Bit_SET); 

	//没有过采样
	GPIO_WriteBit(AD_OS0_PORT, AD_OS0_PIN, Bit_RESET);
	GPIO_WriteBit(AD_OS1_PORT, AD_OS1_PIN, Bit_RESET); 

	GPIO_WriteBit(AD_RST_PORT, AD_RST_PIN, Bit_RESET);	 


	//以下开始SPI配置
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为一线单工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                //设置SPI为主模式
#ifdef DMA_SPI3
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		            //SPI发送接收8位帧结构
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //SPI波特率预分频值为8
#else
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		            //SPI发送接收16位帧结构
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //SPI波特率预分频值为2
#endif
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                    //串行时钟在不操作时，时钟为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	                    //第一个时钟沿开始采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                    //NSS信号由软件（使用SSI位）管理
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                //数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                        //CRC值计算的多项式

	SPI_Init(AD7606_SPI, &SPI_InitStructure);   //根据SPI_InitStruct中指定的参数初始化外设SPI3寄存器
	/* Enable SPI3  */
	SPI_Cmd(AD7606_SPI, ENABLE);                                      //使能SPI外设

	//中断初始化
	GPIO_EXTILineConfig(ADBUSY_PORTSOURCE, ADBUSY_PINSOURCE);
	EXTI_InitStructure.EXTI_Line = ADBUSY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	//中断向量初始化	
	EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

#ifdef DMA_SPI3
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
#endif

	NVIC_InitStructure.NVIC_IRQChannel = ADBUSY_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
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
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_RESET);
	rt_thread_delay(RT_TICK_PER_SECOND/1000);
	GPIO_WriteBit(AD_CVA_PORT, AD_CVA_PIN, Bit_SET);
	return 0;
}
#ifdef DMA_SPI3
void EXTI1_IRQHandler(void) /* AD Data ok */
{
  if(EXTI_GetITStatus(ADBUSY_EXTI_LINE) != RESET)
  {
  	DMA_SPI_Start(ad_dma_buf);
    /* Clear the EXTI Line 4 */
    EXTI_ClearITPendingBit(ADBUSY_EXTI_LINE);
  }
}

void DMA2_Channel2_IRQHandler(void)
{
 	if(DMA_GetFlagStatus(DMA2_FLAG_TC2) == SET){
	 	DMA_ClearITPendingBit(DMA2_IT_TC2);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_TE2) == SET){
	 	DMA_ClearITPendingBit(DMA2_IT_TE2);
	}
	if(DMA_GetFlagStatus(DMA2_FLAG_HT2) == SET){
	 	DMA_ClearITPendingBit(DMA2_IT_HT2);
	}
}

void DMA2_Channel1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_FLAG_TC1) == SET){
		int i = 0 ;
	  	//unsigned short data [8] ={0};
		
		//ad_read(data,8);
		rt_kprintf("ad data:\n");
		for(i=0;i<8;i++){
			printf("channel %d : %04x  %d %fV\n",i,ad_dma_buf[i],(short)ad_dma_buf[i],(((short)ad_dma_buf[i])*10.0/32768.0));
		}	
		//rt_kprintf("\n");    
		DMA_ClearITPendingBit(DMA2_IT_TC1);
	}
}

long ad_pt(void)
{
	int i = 0 ;
	rt_kprintf("ad data:\n");
	for(i=0;i<8;i++){
		printf("channel %d : %04x  %d %fV\n",i,ad_dma_buf[i],(short)ad_dma_buf[i],(((short)ad_dma_buf[i])*10.0/32768.0));
	}
	return 0;	
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
#ifdef DMA_SPI3
FINSH_FUNCTION_EXPORT(ad_pt, print ad7606 convert result);
#endif
#endif

