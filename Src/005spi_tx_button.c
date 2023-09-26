/*
 * 005spi_tx_button.c
 *
 *  Created on: Aug 29, 2023
 *      Author: UF096MBE
 */

#include "stm32f722xx.h"
#include <string.h>

#define COMMAND_LED_CONTROL	0x50

#define LED_ON	1
#define LED_OFF	0

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
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);
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
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit()
{
	GPIO_hundle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);

}

void delay()
{
	for (uint32_t i = 0 ; i < 500000 ; i++);
}

int main(void)
{

	char user_data[] = "AAAAA";

	GPIO_ButtonInit();

	SPI2_GPIOInit();
	SPI2_Init();

	// this make NSS internally high and avoid MODF error (used when SSM is enabled)
	//SPI_SSIConfig(SPI2, ENABLE);

	// this make NSS internally low  (used when SSM is disabled)
	SPI_SSOEConfig(SPI2, ENABLE);



	while(1)
	{

		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);


		uint8_t dataLen = strlen(user_data);

		SPI_SendData(SPI2, &dataLen, 1);
		//delay();

		SPI_SendData(SPI2, (uint8_t*)user_data, dataLen);
		//delay();

		// wait until SPI not busy
		while ((SPI2->SR & (1 << 7)));

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

