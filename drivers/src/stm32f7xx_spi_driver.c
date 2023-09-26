/*
 * stm32f7xx_spi_driver.c
 *
 *  Created on: Aug 29, 2023
 *      Author: UF096MBE
 */
#include "stm32f7xx_spi_driver.h"


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if(pSPIx == SPI4)
			SPI4_PCLK_EN();
		else if(pSPIx == SPI5)
			SPI5_PCLK_EN();
	}
	else
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if(pSPIx == SPI4)
			SPI4_PCLK_DI();
		else if(pSPIx == SPI5)
			SPI5_PCLK_DI();
	}
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_hundle_t *pSPIHandle)
{
	// enable the GPIO clock
		SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t temp = 0;

	// configure the device mode
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		temp &= ~(1 << SPI_CR1_BIDI);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		temp |= (1 << SPI_CR1_BIDI);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		// RXONLY bit must be set
		temp &= ~(1 << SPI_CR1_BIDI);
		temp |= (1 << SPI_CR1_RXONLY);
	}

	// configure the spi serial clock speed (baudrate)

	temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// configure the DEF

	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;
	// configure the CPOL

	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
	// configure the CPHA

	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// configure the SSM

	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 |= temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
	else if(pSPIx == SPI4)
		SPI4_REG_RESET();
	else if(pSPIx == SPI5)
		SPI5_REG_RESET();

}

/*
 * Data read and write
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	//polling based code (blocking mode)
	while(Len > 0)
	{
		// wait until TXE is set
		while (!(pSPIx->SR & (1 << 1)));

		// check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

	//while (!(pSPIx->SR & (1 << 1)));
	//while ((pSPIx->SR & (1 << 7)));
	//uint8_t temp = pSPIx->DR;
	//temp = pSPIx->SR;

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	//polling based code (blocking mode)
	while(Len > 0)
	{

		while ((pSPIx->SR & (1 << 7)));
		pSPIx->DR = 0;

		// wait until RXNE is set
		while (!(pSPIx->SR & (1 << 0)));

		// check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{


}
void SPI_IRQHandling(SPI_hundle_t *pSPIHandle)
{

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
