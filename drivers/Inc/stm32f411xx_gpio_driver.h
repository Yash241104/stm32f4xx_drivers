/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: July 27, 2025
 *      Author: Yashwant N J
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_


#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;					// Gpio Pin number @GPIO_PIN
	uint8_t GPIO_PinMode; 					// possible values  @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					// Pin speeds @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;			// Gpio Pullup and Pulldown @GPIO_PUPD
	uint8_t GPIO_PinOPType; 				// Gpio output type @GPIO_OT_TYPE
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;




/*
 	 Handle Structure
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;		//this hold the base address of the port the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;		//this hold the pin cnofig settings

}GPIO_Handle_t;


//Peripheral Clock Setup
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t En_Di);

// Init and DeInit
void GPIO_Init(GPIO_Handle_t *GPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Read Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ config and Handling
void GPIO_IRQInputConfig(uint8_t IRQNumber, uint8_t En_Di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


/*
 * @GPIO_PIN
 *	GPIO Pin Numbers
*/

#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13		13
#define GPIO_PIN_14 	14
#define GPIO_PIN_15 	15





/*
 *  @GPIO_PIN_MODES
 *	GPIO Pin possible Modes
*/

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IN_FT		4
#define GPIO_MODE_IN_RT		5
#define GPIO_MODE_IN_RFT	6


/*
 * 	@GPIO_OT_TYPE
 *	GPIO Pin possible Output Types
*/

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/*
 *  @GPIO_PIN_SPEED
 *	GPIO Pin possible Output Speeds
*/

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * 	@GPIO_PUPD
 *	GPIO Pin Pullup and Pulldown config
*/

#define GPIO_NO_PUPD		0
#define GPIO_PU			1
#define GPIO_PD			2




#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
