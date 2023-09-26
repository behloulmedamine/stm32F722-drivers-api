/*
 * 002led-button.c
 *
 *  Created on: Aug 28, 2023
 *      Author: UF096MBE
 */

#include "stm32f722xx.h"
#include "stm32f7xx_gpio_driver.h"

void delay()
{
	for (uint32_t i = 0 ; i < 500000/2 ; i++);
}

int main()
{
	GPIO_hundle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		}
	}
	return 0;

}
