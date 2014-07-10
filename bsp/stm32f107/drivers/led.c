#include <stm32f10x.h>
#include <stm32f10x_conf.h>
#include <rtthread.h>
#include "led.h"

void led_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB, GPIOC and AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_GPIO_LED | RCC_APB2Periph_AFIO , ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_LED_ALL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
}

void led_turn_on_all(void)
{
	/* Turn On All LEDs */
    GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED_ALL);
}

void led_turn_off_all(void)
{
	/* Turn Off All LEDs */
    GPIO_SetBits(GPIO_LED_PORT, GPIO_LED_ALL);
}

void rt_hw_led_off(uint8_t n)
{
    switch (n)
    {
    case 0:
         GPIO_SetBits(GPIO_LED_PORT, GPIO_LED1);
        break;
    case 1:
         GPIO_SetBits(GPIO_LED_PORT, GPIO_LED2);
        break;
	case 2:
         GPIO_SetBits(GPIO_LED_PORT, GPIO_LED3);
        break;
	case 3:
         GPIO_SetBits(GPIO_LED_PORT, GPIO_LED4);
        break;
    default:
        break;
    }
}

void rt_hw_led_on(uint8_t n)
{
    switch (n)
    {
    case 0:
        GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED1);
        break;
    case 1:
        GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED2);
        break;
	case 2:
        GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED3);
        break;
	case 3:
        GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED4);
        break;
    default:
        break;
    }
}
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
long led_flash(void)
{
 	unsigned char light = GPIO_ReadOutputDataBit(GPIO_LED_PORT,GPIO_LED4);
	if(light){
		rt_hw_led_on(3);
	}
	else{
		rt_hw_led_off(3);
	}
	return 0 ;
}

#ifdef  USE_FULL_ASSERT

#endif

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(led_flash, flash led 2 )
#endif


