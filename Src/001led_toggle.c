/*
 * 001led_toggle.c
 *
 *  Created on: Aug 25, 2023
 *      Author: UF096MBE
 */


#include "stm32f722xx.h"
#include "stm32f7xx_gpio_driver.h"

void delay()
{
	for (uint32_t i = 0 ; i < 500000 ; i++);
}

int main()
{
	GPIO_hundle_t GpioLed;

	GpioLed.pGPIOx = GPIOB
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		delay();
	}
	return 0;

}
