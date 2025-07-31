/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jul 27, 2025
 *      Author: Yashwant N J
 */

#include "stm32f411xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 *
 * @fn				- GPIO_PeriClkControl
 * @brief			- Enables or Disables peripheral clock for the given GPIO
 *
 * @param			- base address of GPIO
 * @param			- Enable or Disable macro
 *
 * */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t En_Di)
{
	if (En_Di == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();

		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();

		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();

		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();

		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();

		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else{

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();

		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();

		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();

		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();

		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();

		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}

}

/*
 * Init
 *
 * @fn				- GPIO_Init
 * @brief			- Initialize GPIO
 *
 * @param			- base address of GPIO
 *
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Configure GPIO pinmode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting

	}else
	{
		// 1. Intrerput mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==  GPIO_MODE_IN_RT)
		{
			// config rtsr - rising trigger selection reg
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clearing coresponding ftsr bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==  GPIO_MODE_IN_FT)
		{
			// config ftsr - rising trigger selection reg
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clearing coresponding rtsr bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==  GPIO_MODE_IN_RFT)
		{
			// config rtsr - rising trigger selection reg
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// config ftsr - rising trigger selection reg
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. config gpio port selection using SYSCFG_EXTICR

		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;
		uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//Configure Pin Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

	temp = 0;

	//Configure PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; // setting

	temp = 0;

	//Configure Pin Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; // setting

	temp = 0;

	//Configure Alt Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{

		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
		temp = (pGPIOHandle->GPIO_PinConfig .GPIO_PinAltFunMode <<(4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp; // setting

	temp = 0;
	}






}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET;

		}else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET;

		}else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET;

		}else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET;

		}else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET;

		}else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET;
		}
}

// Read Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR |= ~( 1<< PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}


// IRQ config and Handling
void GPIO_IRQInputConfig(uint8_t IRQNumber, uint8_t En_Di)
{

	if(En_Di == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0 reg (Interrupt set enable reg)
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 32 && IRQNumber < 64)
		{
			// program ISER1 reg (Interrupt set enable reg)
			*NVIC_ISER1 |= 1 << (IRQNumber % 32);

		}else if(IRQNumber >= 65 && IRQNumber < 96)
		{
			// program ISER2 reg (Interrupt set enable reg)
			*NVIC_ISER2 |= 1 << (IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			// program ICER0 reg (Interrupt clear enable reg)
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 32 && IRQNumber < 64)
		{
			// program ICER1 reg (Interrupt clear enable reg)
			*NVIC_ICER1 |= 1 << (IRQNumber % 32);

		}else if(IRQNumber >= 65 && IRQNumber < 96)
		{
			// program ICER0 reg (Interrupt clear enable reg)
			*NVIC_ICER2 |= 1 << (IRQNumber % 64);
		}

	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift = (iprx_section * 8) + (8 - 	NO_PR_BITS_IMPLEMENTED);

	(NVIC_IPR[iprx]) |= IRQPriority << shift;


}




void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti pr register corresponding to the pin number
	if(EXTI->PR && (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}








