/*
 * stm32f7xx_spi_driver.h
 *
 *  Created on: Aug 29, 2023
 *      Author: UF096MBE
 */

#ifndef INC_STM32F7XX_SPI_DRIVER_H_
#define INC_STM32F7XX_SPI_DRIVER_H_

#include "stm32f722xx.h"


/*
 *	configuration structure for a SPI pin
 */

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;

}SPI_Config_t;

/*
 * Handle structure for a SPI pin
 */

typedef struct
{
	SPI_RegDef_t	*pSPIx;				//holds the base address of the SPIx
	SPI_Config_t 	SPI_Config;			//holds the SPI configuration settings

}SPI_hundle_t;


/*
 * @DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 * @BusConfig
 */

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/*
 * @DFF
 */

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1


/*
 * @CPOL
 */

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 * @CPHA
 */

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * @SSM
 */

#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/**********************************************************************************************
 * 							APIs supported by this driver
 **********************************************************************************************/


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_hundle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data read and write
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_hundle_t *pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Bit position definition of SPI peripheral
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_BIDI		15
#define SPI_CR1_SPE			6
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9

#define SPI_CR2_SSOE		2


#endif /* INC_STM32F7XX_SPI_DRIVER_H_ */
