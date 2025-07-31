/*
 * 001_Led_Toggle.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Yashwant N J
 */


#include "stm32f411xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{

	GPIO_PeriClkControl(GPIOA, ENABLE);


	GPIO_Handle_t gpioled;

	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_Init(&gpioled);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}

	return 0;
}

