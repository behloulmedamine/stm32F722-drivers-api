/*
 * stm32f722xx.h
 *
 *  Created on: Aug 24, 2023
 *      Author: UF096MBE
 */

#ifndef INC_STM32F722XX_H_
#define INC_STM32F722XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Processor specific details ARM Cortex M7 NVIC ISERx and ICERx and Priority register addresses
 */
#define NVIC_ISER0				( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (__vo uint32_t*)0xE000E10C )

#define NVIC_ICER0				( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1				( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2				( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3				( (__vo uint32_t*)0XE000E18C )

#define NVIC_PR_BASEADDR		( (__vo uint32_t*)0xE000E400 )

// ARM Cortex M7 number of priority bits implemented in prio register
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000UL	// 512 KB
#define ROM						0x00100000UL	// 30 KB
#define SRAM1_BASEADDR			0x20010000UL	// 176 KB
#define SRAM2_BASEADDR			0x2003C000UL	// 16 KB
#define DTCM_BASEADDR			0x20000000UL	// 64 KB
#define SRAM 					SRAM1_BASEADDR

/*
 * AHB and APB bus peripheral base addresses
 */

#define	PERIPH_BASE				0x40000000UL
#define APB1_PERIPH_BASE		PERIPH_BASE
#define APB2_PERIPH_BASE		0x40010000UL
#define AHB1_PERIPH_BASE		0x40020000UL
#define AHB2_PERIPH_BASE		0x50000000UL
#define AHB3_PERIPH_BASE		0xA0000000UL


/*
 * Base addresses of peripheral using AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1_PERIPH_BASE + 0x0000UL)
#define GPIOB_BASEADDR			(AHB1_PERIPH_BASE + 0x0400UL)
#define GPIOC_BASEADDR			(AHB1_PERIPH_BASE + 0x0800UL)
#define GPIOD_BASEADDR			(AHB1_PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASEADDR			(AHB1_PERIPH_BASE + 0x1000UL)
#define GPIOF_BASEADDR			(AHB1_PERIPH_BASE + 0x1400UL)
#define GPIOG_BASEADDR			(AHB1_PERIPH_BASE + 0x1800UL)
#define GPIOH_BASEADDR			(AHB1_PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASEADDR			(AHB1_PERIPH_BASE + 0x2000UL)
#define RCC_BASEADDR			(AHB1_PERIPH_BASE + 0x3800UL)
/*
 * Base addresses of peripheral using APB1 bus
 */

#define SPI2_BASEADDR			(APB1_PERIPH_BASE + 0x3800UL)
#define SPI3_BASEADDR			(APB1_PERIPH_BASE + 0x3C00UL)

#define USART2_BASEADDR			(APB1_PERIPH_BASE + 0x4400UL)
#define USART3_BASEADDR			(APB1_PERIPH_BASE + 0x4800UL)

#define UART4_BASEADDR			(APB1_PERIPH_BASE + 0x4C00UL)
#define UART5_BASEADDR			(APB1_PERIPH_BASE + 0x5000UL)
#define UART7_BASEADDR			(APB1_PERIPH_BASE + 0x7800UL)
#define UART8_BASEADDR			(APB1_PERIPH_BASE + 0x7C00UL)

#define I2C1_BASEADDR			(APB1_PERIPH_BASE + 0x5400UL)
#define I2C2_BASEADDR			(APB1_PERIPH_BASE + 0x5800UL)
#define I2C3_BASEADDR			(APB1_PERIPH_BASE + 0x5C00UL)

/*
 * Base addresses of peripheral using APB2 bus
 */

#define USART1_BASEADDR			(APB2_PERIPH_BASE + 0x1000UL)
#define USART6_BASEADDR			(APB1_PERIPH_BASE + 0x1400UL)

#define SPI1_BASEADDR			(APB2_PERIPH_BASE + 0x3000UL)
#define SPI4_BASEADDR			(APB2_PERIPH_BASE + 0x3400UL)
#define SPI5_BASEADDR			(APB2_PERIPH_BASE + 0x5000UL)

#define SYSCFG_BASEADDR			(APB2_PERIPH_BASE + 0x3800UL)
#define EXTI_BASEADDR			(APB2_PERIPH_BASE + 0x3C00UL)

/*
 * Registers definition of GPIO peripheral
 */

typedef struct
{
	__vo uint32_t MODER;		//GPIO port mode register
	__vo uint32_t OTYPER;		//GPIO port output type register
	__vo uint32_t OSPEEDR;		//GPIO port output speed register
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			//GPIO port input data register
	__vo uint32_t ODR;			//GPIO port output data register
	__vo uint32_t BSRR;			//GPIO port bit set/reset register
	__vo uint32_t LCKR;			//GPIO port configuration lock register
	__vo uint32_t AFR[2];		//GPIO alternate function low+hign register

}GPIO_RegDef_t;


/*
 * Registers definition of EXTI peripheral
 */

typedef struct
{
	__vo uint32_t IMR;				//Interrupt mask register
	__vo uint32_t EMR;				//Event mask register
	__vo uint32_t RSTR;				//Rising trigger selection register
	__vo uint32_t FSTR;				//Falling trigger selection register
	__vo uint32_t SWIER;			//Software interrupt event register
	__vo uint32_t PR;			 	//Pending register

}EXTI_RegDef_t;



/*
 * Registers definition of SYSCFG peripheral
 */

typedef struct
{
	__vo uint32_t MEMRMP;			//SYSCFG memory remap register
	__vo uint32_t PMC;				//SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];		//SYSCFG external interrupt configuration registers
	uint32_t reserved[2];		 	//reserved
	__vo uint32_t CMPCR;		 	//Compensation cell control register
}SYSCFG_RegDef_t;


/*
 * Registers definition of RCC peripheral
 */

typedef struct
{
	__vo uint32_t CR;				//RCC clock control register
	__vo uint32_t RPLLCFGR;			//RCC PLL configuration register (
	__vo uint32_t CFGR;				//RCC clock configuration register
	__vo uint32_t CIR;				//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;			//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;			//RCC AHB3 peripheral reset register
	uint32_t RESERVED1;			//RESERVED
	__vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register
	uint32_t RESERVED2[2];			//RESERVED
	__vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock register
	__vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;			//RCC AHB3 peripheral clock enable register
	uint32_t RESERVED3;			//RESERVED
	__vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register
	uint32_t RESERVED4[2];			//RESERVED
	__vo uint32_t AHB1LPENR;		//RCC AHB1 peripheral clock enable in low-power mode register
	__vo uint32_t AHB2LPENR;		//RCC AHB2 peripheral clock enable in low-power mode register
	__vo uint32_t AHB3LPENR;		//RCC AHB3 peripheral clock enable in low-power mode register
	uint32_t RESERVED5;			//RESERVED
	__vo uint32_t APB1LPENR;		//RCC APB1 peripheral clock enable in low-power mode register
	__vo uint32_t APB2LPENR;		//RCC APB2 peripheral clock enabled in low-power mode register
	uint32_t RESERVED6[2];			//RESERVED
	__vo uint32_t RBDCR;				//RCC backup domain control register
	__vo uint32_t CSR;				//RCC clock control & status register
	uint32_t RESERVED7[2];			//RESERVED
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;		//RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR;		//RCC PLLSAI configuration register
	__vo uint32_t DCKCFGR1;			//RCC dedicated clocks configuration register
	__vo uint32_t DCKCFGR2;			//RCC dedicated clocks configuration register

}RCC_RegDef_t;

/*
 * Registers definition of SYSCFG peripheral
 */

typedef struct
{
	__vo uint32_t CR1;				//SPI control register 1
	__vo uint32_t CR2;				//SPI control register 2
	__vo uint32_t SR;				//SPI status register
	__vo uint32_t DR;				//SPI data register
	__vo uint32_t CRCPR;		 	//SPI CRC polynomial register
	__vo uint32_t RXCRCR;		 	//SPI Rx CRC register
	__vo uint32_t TXCRCR;		 	//SPI Tx CRC register
	__vo uint32_t I2SCFGR;		 	//SPIx_I2S configuration register
	__vo uint32_t I2SPR;		 	//SPIx_I2S prescaler register
}SPI_RegDef_t;


#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5					((SPI_RegDef_t*)SPI5_BASEADDR)

/*
 * Clock enable macros for GPIO peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/*
 * Clock enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for SPI peripherals
 */
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1<<20))

/*
 * Clock enable macros for USART peripherals
 */
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))

/*
 * Clock enable macros for UART peripherals
 */
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))
#define UART7_PCLK_EN()		(RCC->APB1ENR |= (1<<30))
#define UART8_PCLK_EN()		(RCC->APB1ENR |= (1<<31))

/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

/*
 * Clock disable macros for GPIO peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock disable macros for I2C peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1<<20))

/*
 * Clock disable macros for USART peripherals
 */
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock disable macros for UART peripherals
 */
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1<<20))
#define UART7_PCLK_DI()		(RCC->APB1ENR &= ~(1<<30))
#define UART8_PCLK_DI()		(RCC->APB1ENR &= ~(1<<31))

/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

/*
 * reset macros for GPIO peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)


/*
 * reset macros for spi peripheral
 */

#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13)); }while(0)
#define SPI5_REG_RESET()		do{ (RCC->APB2RSTR |= (1<<20)); (RCC->APB2RSTR &= ~(1<<20)); }while(0)



/*
 * return port code for a given GPIOx
 */
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : \
									(x == GPIOI) ? 8 : 0 )

/*
 * IRQ (Interrupt request) numbers of the MCU
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*
 * some generic macros
 */

#define ENABLE				1
#define DISABLE				0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET

#include "stm32f7xx_gpio_driver.h"
#include "stm32f7xx_spi_driver.h"

#endif /* INC_STM32F722XX_H_ */
