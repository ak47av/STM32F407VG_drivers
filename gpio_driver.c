#include "gpio_driver.h"

void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf)
{
	hal_gpio_configure_pin_mode(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->mode);
	hal_gpio_cofigure_pin_otype(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->op_type);
	hal_gpio_configure_pin_speed(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->speed);
	hal_gpio_configure_pin_pupd(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->pull);
	hal_gpio_set_alt_function(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->alternate);
}

void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode)
{
	GPIOx->MODER |= (mode << (2*pin_no));
}

void hal_gpio_cofigure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t op_type)
{
	GPIOx->OTYPER |= (op_type << pin_no);
}

void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed)
{
	GPIOx->OSPEEDR |= (speed << (2*pin_no));
}

void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd)
{
	GPIOx->PUPDR |= (pupd << (2*pin_no));
}

void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t alt_fun_value)
{
	if(pin_no <= 7){
		GPIOx->AFR[0] |= (alt_fun_value << ((pin_no)*4));
	} else {
		GPIOx->AFR[1] |= (alt_fun_value << ((pin_no%8)*4));
	}
}

uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no)
{
	uint8_t value;
	value = ((GPIOx->IDR >> pin_no) & 0x00000001);
	return value;
}

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val)
{
	if(val)
		GPIOx->ODR |= (1 << pin_no);
	else 
		GPIOx->ODR &= ~(1 << pin_no);
}


void hal_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel)
{
	if(edge_sel == INT_RISING_EDGE){
		EXTI->RTSR |= 1<<pin_no;
	}
	else if(edge_sel == INT_FALLING_EDGE){
		EXTI->FTSR |= 1<<pin_no;
	}
	else if(edge_sel == INT_RISING_FALLING_EDGE){
		EXTI->FTSR |= 1<<pin_no;
		EXTI->RTSR |= 1<<pin_no;
	} else ;
}

void hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no)
{
	EXTI->IMR |= 1 << pin_no;
	NVIC_EnableIRQ(irq_no);
}

void hal_gpio_clear_interrupt(uint16_t pin)
{
	if(EXTI->PR & (1<<pin)){
		EXTI->PR |= (1<<pin);
	}
}
	
	