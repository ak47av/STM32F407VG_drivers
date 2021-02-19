#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f407xx.h"


#define GPIO_PIN_INPUT_MODE 							((uint32_t) 0x00)
#define GPIO_PIN_OUTPUT_MODE 							((uint32_t) 0x01)
#define GPIO_PIN_ALT_FUN_MODE 						((uint32_t) 0x02)

#define GPIO_PIN_OP_TYPE_PUSHPULL					((uint32_t) 0x00)
#define GPIO_PIN_OP_TYPE_OPEN_DRAIN				((uint32_t) 0x01)

#define GPIO_PIN_SPEED_LOW 								((uint32_t) 0x00)
#define GPIO_PIN_SPEED_MEDIUM 						((uint32_t) 0x01)
#define GPIO_PIN_SPEED_HIGH 							((uint32_t) 0x02)
#define GPIO_PIN_SPEED_VERY_HIGH					((uint32_t) 0x03)

#define GPIO_PIN_NO_PULL_PUSH							((uint32_t) 0x00)
#define GPIO_PIN_PULL_UP									((uint32_t) 0x01)
#define GPIO_PIN_PULL_DOWN								((uint32_t) 0x02)

//port definitions

#define GPIO_PORT_A GPIOA
#define GPIO_PORT_B GPIOB
#define GPIO_PORT_C GPIOC
#define GPIO_PORT_D GPIOD
#define GPIO_PORT_E GPIOE
#define GPIO_PORT_F GPIOF
#define GPIO_PORT_G GPIOG
#define GPIO_PORT_H GPIOH
#define GPIO_PORT_I GPIOI

//enable clock

#define _HAL_RCC_GPIOA_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<0))
#define _HAL_RCC_GPIOB_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<1))
#define _HAL_RCC_GPIOC_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<2))
#define _HAL_RCC_GPIOD_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<3))
#define _HAL_RCC_GPIOE_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<4))
#define _HAL_RCC_GPIOF_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<5))
#define _HAL_RCC_GPIOG_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<6))
#define _HAL_RCC_GPIOH_CLK_ENABLE()				(RCC->AHB1ENR	|= (1<<7))

typedef struct {
	uint32_t pin;
	uint32_t mode;
	uint32_t op_type;
	uint32_t pull;
	uint32_t speed;
	uint32_t alternate;
}gpio_pin_conf_t;

typedef enum {
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;

//DRIVER EXPOSED APIs

void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf);

//helper functions

void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode);

void hal_gpio_cofigure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t op_type);
	
void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed);

void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd);

void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t alt_fun_value);

uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no);

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val);

//interrupt functions

void hal_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel);

void hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

void hal_gpio_clear_interrupt(uint16_t pin);

























#endif
