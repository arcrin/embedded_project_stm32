//
// Created by andy- on 2021-10-24.
//

#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H

#include <stdint.h>

#define __vo        volatile

/********************************************START: Processor Specific Details***************************************/
/*
 *  ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_IESR0				((__vo uint32_t*) 0xE000E100)
#define NVIC_IESR1				((__vo uint32_t*) 0xE000E104)
#define NVIC_IESR2				((__vo uint32_t*) 0xE000E108)
#define NVIC_IESR3				((__vo uint32_t*) 0xE000E10c)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0				((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*) 0xE000E18c)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED	4
/*
 *  base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U	//main memory
#define SRAM1_BASEADDR			0x20000000U	//sram1 size 112k
#define SRAM2_BASEADDR 			0x2001c000U	//sram2 size 16k
#define ROM						0x1FFF0000U	//system memory
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define	PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define	APB2PERIPH_BASE		0x40010000U
#define	AHB1PERIPH_BASE		0x40020000U
#define	AHB2PERIPH_BASE		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0c00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1c00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5c00)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3c00)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4c00) //not capable producing synchronous communication
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 */

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR 		(APB2PERIPH_BASE + 0x3c00)

/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g: Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * check your Device RM
 */

typedef struct {
    //offset	description
    __vo uint32_t MODER;		//0x0000 	GPIO port mode register
    __vo uint32_t OTYPER;		//0x0004	GPIO port output type register
    __vo uint32_t OSPEEDR;		//0x0008 	GPIO port output speed register
    __vo uint32_t PUPDR;		//0x000c 	GPIO port pull-up/pull-down register
    __vo uint32_t IDR;			//0x0010	GPIO port input data register
    __vo uint32_t ODR;			//0x0014	GPIO port output data register
    __vo uint32_t BSRR;			//0x0018	GPIO port bit set/reset register
    __vo uint32_t LCKR;			//0x001c	GPIO port configuration lock register
    __vo uint32_t AFR[2];		//0x0020, 	GPIO alternate function low and high registers (64 bits)

}GPIO_RegDef_t;

typedef struct {
    //offset	description
    __vo uint32_t CR;				//0x00		control register
    __vo uint32_t PLLCFGR;			//0x04		PLL configuration register
    __vo uint32_t CFGR;				//0x08		clock configuration register
    __vo uint32_t CIR;				//0x0c		clock interrupt register
    __vo uint32_t AHB1RSTR;			//0x10		AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;			//0x14		AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;			//0x18		AHB3 peripheral reset register
    uint32_t RESERVED0;				//0x1c
    __vo uint32_t APB1RSTR;			//0x20		APB1 peripheral reset register
    __vo uint32_t APB2RSTR;			//0x24		APB2 peripheral reset register
    uint32_t RESERVED1[2];			//0x28, 0x2c
    __vo uint32_t AHB1ENR;			//0x30		AHB1 peripheral clock enable register
    __vo uint32_t AHB2ENR;			//0x34		AHB2 peripheral clock enable register
    __vo uint32_t AHB3ENR;			//0x38		AHB3 peripheral clock enable register
    uint32_t RESERVED2;				//0x3c
    __vo uint32_t APB1ENR;			//0x40		APB1 peripheral clock enable register
    __vo uint32_t APB2ENR;			//0x44		APB2 peripheral clock enable register
    uint32_t RESERVED3[2];			//0x48, 0x4c
    __vo uint32_t AHB1LPENR;		//0x50		AHB1 peripheral clock enable in low power mode register
    __vo uint32_t AHB2LPENR;		//0x54		AHB2 peripheral clock enable in low power mode register
    __vo uint32_t AHB3LPENR;		//0x58		AHB3 peripheral clock enable in low power mode register
    uint32_t RESERVED4;				//0x5c
    __vo uint32_t APB1LPENR;		//0x60		APB1 peripheral clock enable in low power mode register
    __vo uint32_t APB2LPENR;		//0x64		APB2 peripheral clock enable in low power mode register
    uint32_t RESERVED5[2];			//0x68, 0x6c
    __vo uint32_t BDCR;				//0x70		Backup domain control register
    __vo uint32_t CSR;				//0x74		Clock control and status register
    uint32_t RESERVED6[2];			//0x78, 0x7c
    __vo uint32_t SSCGR;			//0x80		Spread spectrum clock generation register
    __vo uint32_t PLLI2SCFGR;		//0x84		PLLI2S configuration register
}RCC_RegDef_t;


typedef struct {
    __vo uint32_t IMR;				//0x00		interrupt mask register
    __vo uint32_t EMR;				//0x04		event mask register
    __vo uint32_t RTSR;				//0x08		rising trigger selection register
    __vo uint32_t FTSR;				//0x0c		falling trigger selection register
    __vo uint32_t SWIER;			//0x10		software interrupt event register
    __vo uint32_t PR;				//0x14		pending register
}EXTI_RegDef_t;

typedef struct {
    __vo uint32_t MEMRMP;			//0x00		memory remap register
    __vo uint32_t PMC;				//0x04		peripheral mode configuration register
    __vo uint32_t EXTICR[4];		//0x08-0x14	external interrupt configuration register 1
    uint32_t RESERVED1[2];			//0x18-0x1c
    __vo uint32_t CMPCR;			//0x20		compensation cell control register
}SYSCFG_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC -> APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()	(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC -> APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC -> APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC -> APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC -> APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC -> APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC -> APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC -> APB2ENR |= (1 << 5))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFGR_PCLK_EN()	(RCC -> APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 8))

/*
 * Clock Reset Macros for GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 *  returns port code for given GPIOx base address
 */
#define GPIO_BASE_TO_CODE(x)		((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOB) ? 2 :\
									(x == GPIOB) ? 3 :\
									(x == GPIOB) ? 4 :\
									(x == GPIOB) ? 5 :\
									(x == GPIOB) ? 6 :\
									(x == GPIOB) ? 7 : 0)


/*
 * IRQ numbers (positions) of stm32f407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1 			7
#define IRQ_NO_EXTI2 			8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10 		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2         	36
#define IRQ_NO_SPI3         	51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     		31
#define IRQ_NO_I2C1_ER     		32
#define IRQ_NO_USART1	    	37
#define IRQ_NO_USART2	    	38
#define IRQ_NO_USART3	    	39
#define IRQ_NO_UART4	    	52
#define IRQ_NO_UART5	    	53
#define IRQ_NO_USART6	    	71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  	(RCC -> APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  	(RCC -> APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFGR_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 14))

/*
 * some generic macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif //MCU1_STM32F407XX_H
