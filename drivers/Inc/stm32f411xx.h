/*
 * stm32f411xx.h
 *
 *  Created on: Jul 27, 2025
 *      Author: Yashwant N J
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define __vo volatile




/************************************** PROCESSOR SPECIFIC DETAILS **************************************/
/*
 * ARM CORTEX M4 NVIC ISERx register addresses
 */

#define NVIC_ISER0  			((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1  			((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2  			((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3  			((__vo uint32_t*) 0xE000E10C)


/*
 * ARM CORTEX M4 NVIC ICERx register addresses
 */

#define NVIC_ICER0  			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1  			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  			((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3  			((__vo uint32_t*)0XE000E18C)


/*
 * ARM CORTEX M4 NVIC IPRx register addresses
 */

#define NVIC_IPR 				((__vo uint32_t*) 0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4










/*
	Base addresses of Flash and SRAM
*/

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U //112KB
#define SRAM2_BASEADDR				0x20001C00U // SRAM1_BASEADDR + 112kB
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM						SRAM1_BASEADDR

/*
	AHBx and APBx Bus Peripheral base addresses
*/

#define APB1PERIPH_BASEADDR			0x40000000U
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U


/*
	Base addresses of peripherals hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/*
	Base addresses of peripherals hanging on APB1 bus
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)

/*
	Base addresses of peripherals hanging on APB2 bus
 */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)


#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)


/*************************** Peripheral Register Definition ***************************/

typedef struct
{
	__vo uint32_t MODER;				//Address Offset : 0x00
	__vo uint32_t OTYPER;				//Address Offset : 0x04
	__vo uint32_t OSPEEDR;				//Address Offset : 0x08
	__vo uint32_t PUPDR;				//Address Offset : 0x0C
	__vo uint32_t IDR;					//Address Offset : 0x10
	__vo uint32_t ODR;					//Address Offset : 0x14
	__vo uint32_t BSRR;					//Address Offset : 0x18
	__vo uint32_t LCKR;					//Address Offset : 0x1C
	__vo uint32_t AFR[2];				//Address Offset : 0x20 - 0X24

}GPIO_RegDef_t;

/*************************** RCC Structure ***************************/

typedef struct
{
	__vo uint32_t CR;						// Control Register  							Address Offset : 0x00
	__vo uint32_t PLLCFGR;					// PLL configuration register  					Address Offset : 0x04
	__vo uint32_t CFGR;						// clock configuration register  				Address Offset : 0x08
	__vo uint32_t CIR;						// clock interrupt register 					Address Offset : 0x0C
	__vo uint32_t AHB1RSTR;					// AHB1 peripheral reset register	 			Address Offset : 0x10
	__vo uint32_t AHB2RSTR;					// AHB2 peripheral reset register 	 			Address Offset : 0x14
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;					// APB1 peripheral reset register 				Address Offset : 0x20
	__vo uint32_t APB2RSTR;					// APB2 peripheral reset register 				Address Offset : 0x24
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;					// AHB1 peripheral clock enable register  		Address Offset : 0x30
	__vo uint32_t AHB2ENR;					// AHB2 peripheral clock enable register  		Address Offset : 0x34
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;					// APB1 peripheral clock enable register  		Address Offset : 0x40
	__vo uint32_t APB2ENR;					// APB2 peripheral clock enable register  		Address Offset : 0x44
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;				// AHB1 peripheral clock enable in low power mode register  	Address Offset : 0x50
	__vo uint32_t AHB2LPENR;				// AHB2 peripheral clock enable in low power mode register  	Address Offset : 0x54
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;				// APB1 peripheral clock enable in low power mode register 		Address Offset : 0x60
	__vo uint32_t APB2LPENR;				// APB2 peripheral clock enable in low power mode register 		Address Offset : 0x64
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;				// Backup domain control register		 			Address Offset : 0x70
	__vo uint32_t CSR;				// Clock control & status register		 			Address Offset : 0x74
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;			// Spread spectrum clock generation register		Address Offset : 0x80
	__vo uint32_t PLLI2SCFGR;				// PLLI2S configuration register 		 		Address Offset : 0x84
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;				// Dedicated Clocks Configuration Register		 		Address Offset : 0x8C

}RCC_RegDef_t;

/*************************** EXTI Structure ***************************/

typedef struct
{
	__vo uint32_t IMR;				//Address Offset : 0x00
	__vo uint32_t EMR;				//Address Offset : 0x04
	__vo uint32_t RTSR;				//Address Offset : 0x08
	__vo uint32_t FTSR;				//Address Offset : 0x0C
	__vo uint32_t SWIER;			//Address Offset : 0x10
	__vo uint32_t PR;				//Address Offset : 0x14

}EXTI_RegDef_t;


/*************************** SYSCFG Structure ***************************/

typedef struct
{
	__vo uint32_t MEMRMP;				//Address Offset : 0x00
	__vo uint32_t PMC;				//Address Offset : 0x04
	__vo uint32_t EXTICR[4];			//Address Offset : 0x08 - 14
	__vo uint32_t CMPCR;				//Address Offset : 0x14

}SYSCFG_RegDef_t;




/*
 	 Peripheral Base Address typecasted to xxxx_RegDef_t
 */


#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)


/**************************  ENABLE  **************************/
/*
 	 Clock Enable Macros for GPIOx
*/

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 	 Clock Enable Macros for I2Cx
*/

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 	 Clock Enable Macros for SPIx
*/

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))

/*
 	 Clock Enable Macros for USARTx
*/

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 	 Clock Enable Macros for SYSCFG
*/

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))


/**************************  DISABLE  **************************/
/*
 	 Clock Disable Macros for GPIOx
*/

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 	 Clock Disable Macros for I2Cx
*/

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 	 Clock Disable Macros for SPIx
*/

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))

/*
 	 Clock Disable Macros for USARTx
*/

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

/*
 	 Clock Disable Macros for SYSCFG
*/

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))



/*
 * Macros to Reset GPIOx Peripherals
 */

#define GPIOA_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET 				do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Macros For IRQ Number
 */

#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI9_5		23
#define IRQ_EXTI15_10	40






// Some Generic Macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET


#define GPIO_BASE_TO_CODE(x) 			( (x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
										(x == GPIOE) ? 4:\
										(x == GPIOH) ? 7: 0)



#include "stm32f411xx_gpio_driver.h"



#endif /* INC_STM32F411XX_H_ */
