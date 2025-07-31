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




	GPIO_Handle_t gpioled, gpiobtn;

	//	GPIO LED CONFIGURATION
	GPIO_PeriClkControl(GPIOA, ENABLE);
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioled);


	//	GPIO BUTTON CONFIGURATION
	GPIO_PeriClkControl(GPIOC, ENABLE);
	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	gpiobtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpiobtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		}

	}

	return 0;
}

