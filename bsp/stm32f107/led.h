#ifndef LEDH
#define LEDH
/*LED define*/
#define RCC_GPIO_LED                                 RCC_APB2Periph_GPIOD
#define GPIO_LED_PORT                                GPIOD    
#define GPIO_LED1                                    GPIO_Pin_12    
#define GPIO_LED2                                    GPIO_Pin_13    
#define GPIO_LED3                                    GPIO_Pin_14    
#define GPIO_LED4                                    GPIO_Pin_15
#define GPIO_LED_ALL                                 GPIO_LED1 |GPIO_LED2 |GPIO_LED3 |GPIO_LED4 

void led_config(void);
void led_turn_on_all(void);
void led_turn_off_all(void);
void rt_hw_led_on(unsigned char led);
void rt_hw_led_off(unsigned char n);
long led_flash(void);

#endif

