#include "stdint.h"
#include "gpio_driver.h"
#include "main.h"
#include "stm32f407xx.h"


void led_init(void){
	gpio_pin_conf_t led_pin_conf;
	_HAL_RCC_GPIOD_CLK_ENABLE();
	
	led_pin_conf.pin = LED_ORANGE;
	led_pin_conf.mode = GPIO_PIN_OUTPUT_MODE;
	led_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	led_pin_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	led_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_init(GPIOD, &led_pin_conf);
	
	led_pin_conf.pin = LED_BLUE;
	hal_gpio_init(GPIOD, &led_pin_conf);
	
	led_pin_conf.pin = LED_GREEN;
	hal_gpio_init(GPIOD, &led_pin_conf);
	
	led_pin_conf.pin = LED_RED;
	hal_gpio_init(GPIOD, &led_pin_conf);
}

void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin){
	hal_gpio_write_to_pin(GPIOx,pin,1);
}

void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin){
	hal_gpio_write_to_pin(GPIOx,pin,0);
}

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin){
	if(hal_gpio_read_from_pin(GPIOx,pin) == 0){
		hal_gpio_write_to_pin(GPIOx,pin,1);
	} else hal_gpio_write_to_pin(GPIOx,pin,0);
}

int main(void){
	
	
	led_init();
	
	_HAL_RCC_GPIOA_CLK_ENABLE();
//	GPIOA->MODER &= ~0x3;
//	GPIOA->PUPDR &= ~0x3;
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN,INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	
#if 0	
	
	uint32_t i;
	
	while(1){
		led_turn_on(GPIOD,LED_ORANGE);
		led_turn_on(GPIOD,LED_BLUE);
		
		for(i=0;i<500000;i++) __NOP();
		
		led_turn_off(GPIOD,LED_ORANGE);
		led_turn_off(GPIOD,LED_BLUE);
		
		for(i=0;i<500000;i++) __NOP();
		
	}
#endif

	while(1);
	
}

void EXTI0_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	led_toggle(GPIOD,LED_BLUE);
	led_toggle(GPIOD,LED_ORANGE);
	led_toggle(GPIOD,LED_RED);
	led_toggle(GPIOD,LED_GREEN);
}
