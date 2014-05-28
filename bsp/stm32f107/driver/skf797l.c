#include <rtthread.h>
#include "stm32f10x.h"
#include <stdio.h>
#include <finsh.h>
#include "../led.h"

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
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
	rt_kprintf("once skf\n");  
	led_flash();
    /* Clear the EXTI Line 5 */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}

long skf_st(void)
{
	printf("skf: %d",GPIO_ReadInputDataBit(SKF_PORT,SKF_PIN));
	return 0;
}





#ifdef RT_USING_FINSH	   
FINSH_FUNCTION_EXPORT(skf_st, print skf state);

#endif
