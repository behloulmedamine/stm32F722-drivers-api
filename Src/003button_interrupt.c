/*
 * 003button_interrupt.c
 *
 *  Created on: Aug 28, 2023
 *      Author: UF096MBE
 */


#include "stm32f722xx.h"


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
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;

}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
}
