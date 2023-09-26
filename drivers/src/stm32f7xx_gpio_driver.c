/*
 * stm32f7xx_gpio_driver.c
 *
 *  Created on: Aug 25, 2023
 *      Author: UF096MBE
 */

#include "stm32f7xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/***********************************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- this function enables or disables peripheral clock for a given GPO port
 *
 * @param[in]			- base address of the GPIO port
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * @return				- none
 *
 * @note				- none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DI();
	}
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_hundle_t *pGPIOHandle)
{
	// enable the GPIO clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	} else
	{
		// interrupt mode
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure the FSTR
			EXTI->FSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RSTR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure the RSTR
			EXTI->RSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FSTR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure the FSTR and RSTR
			EXTI->FSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// configure the GPIO port in SYSCFG_EXTICR
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		// enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function registers
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(( pGPIOx->IDR >> PinNumber ) & 0x1);

	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)( pGPIOx->IDR  & 0xFFFF);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));

		}
	}else
	{
		if(IRQNumber < 32)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 96 && IRQNumber < 128)
		{
			//program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));

		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp1, temp2;
	temp1 = IRQNumber / 4;
	temp2 = IRQNumber % 4;

	uint8_t shift_amount = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + temp1) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR register
	if(EXTI->PR & (1 <<PinNumber))
	{
		EXTI->PR |= (1 <<PinNumber);
	}

}


