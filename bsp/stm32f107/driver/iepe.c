
#include <rtthread.h>
#include "stm32f10x.h"
#include <stdio.h>
#include <finsh.h>

#define IEPE1_PORT		GPIOE
#define IEPE1_PIN	    GPIO_Pin_2
#define IEPE2_PORT		GPIOE
#define IEPE2_PIN		GPIO_Pin_3
#define IEPE3_PORT		GPIOE
#define IEPE3_PIN		GPIO_Pin_4
#define IEPE4_PORT		GPIOE
#define IEPE4_PIN		GPIO_Pin_5

void iepe_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);    //使能GPIO的时钟

	GPIO_InitStructure.GPIO_Pin = IEPE1_PIN | IEPE2_PIN | IEPE3_PIN | IEPE4_PIN;          //过采样 OS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IEPE1_PORT, &GPIO_InitStructure);
		
	GPIO_WriteBit(IEPE1_PORT, IEPE1_PIN, Bit_SET);
	GPIO_WriteBit(IEPE2_PORT, IEPE2_PIN, Bit_SET);
	GPIO_WriteBit(IEPE3_PORT, IEPE3_PIN, Bit_SET);
	GPIO_WriteBit(IEPE4_PORT, IEPE4_PIN, Bit_SET);
}


long iepe_set(int pin,int value)
{
	switch(pin){
	case 1:
		if(value)
			GPIO_WriteBit(IEPE1_PORT, IEPE1_PIN, Bit_SET);
		else
			GPIO_WriteBit(IEPE1_PORT, IEPE1_PIN, Bit_RESET);
		break;
	case 2:
		if(value)
			GPIO_WriteBit(IEPE2_PORT, IEPE2_PIN, Bit_SET);
		else
			GPIO_WriteBit(IEPE2_PORT, IEPE2_PIN, Bit_RESET);
		break;
	case 3:
		if(value)
			GPIO_WriteBit(IEPE3_PORT, IEPE3_PIN, Bit_SET);
		else
			GPIO_WriteBit(IEPE3_PORT, IEPE3_PIN, Bit_RESET);
		break;
	case 4:
		if(value)
			GPIO_WriteBit(IEPE4_PORT, IEPE4_PIN, Bit_SET);
		else
			GPIO_WriteBit(IEPE4_PORT, IEPE4_PIN, Bit_RESET);
		break;
	}
	return 0;
}

long iepe_st(void)
{
	printf("iepe status:\n");
	printf("\tiepe 1: %d",GPIO_ReadOutputDataBit(IEPE1_PORT,IEPE1_PIN));
	printf("\tiepe 2: %d",GPIO_ReadOutputDataBit(IEPE2_PORT,IEPE2_PIN));
	printf("\tiepe 3: %d",GPIO_ReadOutputDataBit(IEPE3_PORT,IEPE3_PIN));
	printf("\tiepe 4: %d\n",GPIO_ReadOutputDataBit(IEPE4_PORT,IEPE4_PIN));
	return 0;
}

#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(iepe_set, start set iepe status);	   
FINSH_FUNCTION_EXPORT(iepe_st, print iepe status);
#endif

