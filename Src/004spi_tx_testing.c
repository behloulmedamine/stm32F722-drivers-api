/*
 * 004spi_tx_testing.c
 *
 *  Created on: Aug 29, 2023
 *      Author: UF096MBE
 */

#include "stm32f722xx.h"
#include <string.h>

/*
 *  SPI2_MOSI	PB15 AF5
 *	SPI2_MISO 	PB14 AF5
 * 	SPI2_SCLK	PB13 AF5
 * 	SPI2_NSS	PB12 AF5
 */

void SPI2_GPIOInit(void)
{
	GPIO_hundle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);
	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2Pins);
	//MISO
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2Pins);
	//NSS
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPI2Pins);
}

void SPI2_Init(void)
{
	SPI_hundle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN; // software slave management enabled

	SPI_Init(&SPI2handle);
}

void delay()
{
	for (uint32_t i = 0 ; i < 500000/4 ; i++);
}

int main(void)
{

	char user_data[] = "hello world";

	SPI2_GPIOInit();
	SPI2_Init();

	// this make NSS internally high and avoid MODF error (used when SSM is enabled)
	SPI_SSIConfig(SPI2, ENABLE);
	delay();
	// enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// wait until SPI not busy
	while ((SPI2->SR & (1 << 7)));

	// disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
