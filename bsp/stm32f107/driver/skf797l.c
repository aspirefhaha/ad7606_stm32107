#include <rtthread.h>
#include "stm32f10x.h"
#include <stdio.h>
#include <finsh.h>

#define SKF_PORT		GPIOB
#define SKF_PIN		    GPIO_Pin_5

void skf_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);    //使能GPIO的时钟


	GPIO_InitStructure.GPIO_Pin = SKF_PIN;          //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SKF_PORT, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}
static int dicount=0;
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
	dicount++;
	//rt_kprintf("once skf\n");   
    /* Clear the EXTI Line 5 */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}

long skf_st(void)
{
	printf("skf: %d",GPIO_ReadInputDataBit(SKF_PORT,SKF_PIN));
	return 0;
}

void io_count_entry(void * parameter)
{
 	while(1){
	 	rt_thread_delay(RT_TICK_PER_SECOND);
		printf("io count %d",dicount);
		dicount=0;
	}
}

long skf_th(void)
{
 	rt_thread_t io_th = rt_thread_create("skf_th",io_count_entry,RT_NULL,2048,10,20);
	if(io_th){
	 	rt_thread_startup(io_th);
	}
	else{
	 	rt_kprintf("create skf th failed!");
	}
	return 0;
}

#ifdef RT_USING_FINSH	   
FINSH_FUNCTION_EXPORT(skf_st, print skf state);

FINSH_FUNCTION_EXPORT(skf_th, start io count thread);
#endif
