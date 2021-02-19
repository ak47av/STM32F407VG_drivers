#ifndef __HAL_SPI_DRIVER_H
#define __HAL_SPI_DRIVER_H

#include "stm32f407xx.h"


//Bit definitions for Control register 1
#define SPI_REG_CR1_BIDIMODE 						((uint32_t) 1 << 15)
#define SPI_ENABLE_2_LINE_UNI_DIR				0
#define SPI_ENABLE_1_LINE_BI_DIR				1

#define SPI_REG_CR1_DFF									((uint32_t) 1 << 11)
#define SPI_8bit_DF_ENABLE							0
#define SPI_16bit_DF_ENABLE							1

#define SPI_REG_CR1_SSM									((uint32_t) 1 << 9)
#define SPI_SSM_ENABLE									0
#define SPI_SSM_DISABLE									1

#define SPI_REG_CR1_SSI									((uint32_t) 1 << 8)

#define SPI_CR1_LSB_FIRST								((uint32_t) 1 << 7)
#define SPI_TX_MSBFIRST									0
#define SPI_TX_LSBFIRST									1

#define SPI_REG_CR1_SPE									((uint32_t) 1 << 6)

#define SPI_REG_CR1_BR_PCLK_DIV_2				((uint32_t) 0 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_4				((uint32_t) 1 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_8				((uint32_t) 2 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_16			((uint32_t) 3 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_32			((uint32_t) 4 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_64			((uint32_t) 5 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_128			((uint32_t) 6 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_256			((uint32_t) 7 << 3)

#define SPI_REG_CR1_MSTR								((uint32_t) 1 << 2)
#define SPI_MASTER_MODE_SEL							1
#define SPI_SLAVE_MODE_SEL							0

#define SPI_REG_CR1_CPOL								((uit32_t) 1 << 1)
#define SPI_CPOL_LOW										0
#define SPI_CPOL_HIGH										1

#define SPI_REG_CR1_CPHA								((uint32_t) 1 << 0)
#define SPI_FIRST_CLOCK TRANS						0
#define SPI_SECOND_CLOCK_TRANS					1

//Bit definitions for Control register 2
#define SPI_REG_CR2_TXEIE_ENABLE				((uint32_t) 1 << 7)
#define SPI_REG_CR2_RXNEIE_ENABLE				((uint32_t) 1 << 6)
#define SPI_REG_CR2_ERRIE_ENABLE				((uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT				((uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE								0
#define SPI_TI_MODE											1

#define SPI_REG_CR2_SSOE								((uint32_t) 1 << 2)


//Bit definitions for SPI status register
#define SPI_REG_SR_FRE_FLAG							((uint32_t) 1 << 8)
#define SPI_REG_SR_BUSY_FLAG						((uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG 						((uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG						((uint32_t) 1 << 0)


// SPI device base address
#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3

#define SPI_IS_BUSY											1
#define SPI_IS_NOT_BUSY 								0

#define _HAL_RCC_SPI1_CLK_ENABLE() 			( RCC->APB2ENR |= (1 << 12))
#define _HAL_RCC_SPI2_CLK_ENABLE() 			( RCC->APB1ENR |= (1 << 14))
#define _HAL_RCC_SPI3_CLK_ENABLE() 			( RCC->APB1ENR |= (1 << 15))


#define RESET 0
#define SET !RESET

/*  		
				DATA STRUCTURES USED BY SPI
																					*/
																					
typedef enum 
{
	HAL_SPI_STATE_RESET				= 0x00,
	HAL_SPI_STATE_READY				= 0x01,
	HAL_SPI_STATE_BUSY				=	0x02,
	HAL_SPI_STATE_BUSY_TX			= 0x12,
	HAL_SPI_STATE_BUSY_RX			= 0x22,
	HAL_SPI_STATE_BUSY_TX_RX	= 0x32,
	HAL_SPI_STATE_ERROR 			= 0x03
}hal_spi_state_t;																		
									

typedef struct
{
	uint32_t Mode;
	uint32_t Direction;
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;
	uint32_t BaudRatePrescaler;
	uint32_t FirstBit;
}spi_init_t;


typedef struct __spi_handle_t
{
	SPI_TypeDef			*Instance;
	spi_init_t			Init;
	uint8_t					*pTxBuffPtr;
	uint16_t 				TxXferSize;
	uint16_t 				TxXferCount;
	uint8_t 				*pRxBuffPtr;
	uint16_t 				RxXferSize;
	uint16_t				RxXferCount;
	hal_spi_state_t	State;
}spi_handle_t;


/* 
				Driver Exposed APIs
																*/

void hal_spi_init(spi_handle_t *spi_handle);

void hal_spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);

void hal_spi_slave_rx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);

void hal_spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);

void hal_spi_slave_tx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);

void hal_i2c_spi_irq_handler(spi_handle_t *hspi);


#endif
