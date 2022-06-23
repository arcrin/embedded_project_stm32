//
// Created by andy- on 2021-10-26.
//



#ifndef MCU1_STM32F407XX_H
#define MCU1_STM32F407XX_H

#include <cstdint>
#include <cstddef>
#include <cstring>
//#include <cstdio>

#define _vo     volatile
#define _ro     volatile const
#define __weak  __attribute__((weak))
/*
 * Base memory address
 */
#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM                SRAM1_BASEADDR
#define SRAM2_BASEADDR      (SRAM1_BASEADDR + 0x0001c000)
#define ROM_BASEADDR        0x1FFF0000U  // System memory

/**********************************************************************
 * Processor specific details
 **********************************************************************/
/*
 * NVIC registers
 */
#define NVIC_ISER0      ((_vo uint32_t *)0xE000E100) // TODO: why are they pointers? because it is address of the retisger
#define NVIC_ISER1      ((_vo uint32_t *)0xE000E104)
#define NVIC_ISER2      ((_vo uint32_t *)0xE000E108)
#define NVIC_ISER3      ((_vo uint32_t *)0xE000E10C)
#define NVIC_ISER4      ((_vo uint32_t *)0xE000E110)
#define NVIC_ISER5      ((_vo uint32_t *)0xE000E114)
#define NVIC_ISER6      ((_vo uint32_t *)0xE000E118)
#define NVIC_ISER7      ((_vo uint32_t *)0xE000E11C)

#define NVIC_ICER0      ((_vo uint32_t *)0xE000E180)
#define NVIC_ICER1      ((_vo uint32_t *)0xE000E184)
#define NVIC_ICER2      ((_vo uint32_t *)0xE000E188)
#define NVIC_ICER3      ((_vo uint32_t *)0xE000E18C)
#define NVIC_ICER4      ((_vo uint32_t *)0xE000E190)
#define NVIC_ICER5      ((_vo uint32_t *)0xE000E194)
#define NVIC_ICER6      ((_vo uint32_t *)0xE000E198)
#define NVIC_ICER7      ((_vo uint32_t *)0xE000E19C)

#define NVIC_IPR_BASEADDR       ((_vo uint32_t*)0xE000E400)

#define NVIC_PRIORITY_BITS   4



/**********************************************************************
 * Peripheral specific details
 **********************************************************************/

/*
 * Peripheral bus base addresses
 */
#define PERIPH_BASE         0x40000000
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     0x40010000
#define AHB1PERIPH_BASE     0x40020000
#define AHB2PERIPH_BASE     0x50000000


/*
 * Peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR      (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR      (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR      (AHB1PERIPH_BASE + 0x2800)

/*
 * Peripherals on  ABP1 bus
 */
#define SPI2_BASEADDR       (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR       (APB1PERIPH_BASE + 0x3C00)

#define TIM6_BASEADDR       (APB1PERIPH_BASE + 0x1000)

/*
 * Peripherals on ABP2 bus
 */
#define SPI1_BASEADDR       (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR     (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR       (APB2PERIPH_BASE + 0x3C00)

/*
 * RCC base address
 */
#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x3800)

/*
 * GPIO register structure
 */
typedef struct{
    _vo int32_t MODER;
    _vo int32_t OTYPER;
    _vo int32_t OSPEEDR;
    _vo int32_t PUPDR;
    _vo int32_t IDR;
    _vo int32_t ODR;
    _vo int32_t BSRR;
    _vo int32_t LCKR;
    _vo int32_t AFR[2];
} GPIO_RegDef_t, *pGPIO_RegDef_t;

/*
 * GPIO definitions
 */
#define GPIOA   ((pGPIO_RegDef_t) GPIOA_BASEADDR)
#define GPIOB   ((pGPIO_RegDef_t) GPIOB_BASEADDR)
#define GPIOC   ((pGPIO_RegDef_t) GPIOC_BASEADDR)
#define GPIOD   ((pGPIO_RegDef_t) GPIOD_BASEADDR)
#define GPIOE   ((pGPIO_RegDef_t) GPIOE_BASEADDR)
#define GPIOF   ((pGPIO_RegDef_t) GPIOF_BASEADDR)
#define GPIOG   ((pGPIO_RegDef_t) GPIOG_BASEADDR)
#define GPIOH   ((pGPIO_RegDef_t) GPIOH_BASEADDR)
#define GPIOI   ((pGPIO_RegDef_t) GPIOI_BASEADDR)
#define GPIOJ   ((pGPIO_RegDef_t) GPIOJ_BASEADDR)
#define GPIOK   ((pGPIO_RegDef_t) GPIOK_BASEADDR)

#define GET_GPIO_PORT_CODE(pPort)    (((pPort) == GPIOA)?0:\
                                     ((pPort) == GPIOB)?1:\
                                     ((pPort) == GPIOC)?2:\
                                     ((pPort) == GPIOD)?3:\
                                     ((pPort) == GPIOE)?4:\
                                     ((pPort) == GPIOF)?5:\
                                     ((pPort) == GPIOG)?6:\
                                     ((pPort) == GPIOH)?7:\
                                     ((pPort) == GPIOI)?8:\
                                     ((pPort) == GPIOJ)?9:\
                                     ((pPort) == GPIOK)?10:0)

/*
 * GPIO clock macros
 */
#define GPIOA_PCLK_EN()     RCC->AHB1ENR |= 1 << 0
#define GPIOB_PCLK_EN()     RCC->AHB1ENR |= 1 << 1
#define GPIOC_PCLK_EN()     RCC->AHB1ENR |= 1 << 2
#define GPIOD_PCLK_EN()     RCC->AHB1ENR |= 1 << 3
#define GPIOE_PCLK_EN()     RCC->AHB1ENR |= 1 << 4
#define GPIOF_PCLK_EN()     RCC->AHB1ENR |= 1 << 5
#define GPIOG_PCLK_EN()     RCC->AHB1ENR |= 1 << 6
#define GPIOH_PCLK_EN()     RCC->AHB1ENR |= 1 << 7
#define GPIOI_PCLK_EN()     RCC->AHB1ENR |= 1 << 8
#define GPIOJ_PCLK_EN()     RCC->AHB1ENR |= 1 << 9
#define GPIOK_PCLK_EN()     RCC->AHB1ENR |= 1 << 10

#define GPIOA_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 8)
#define GPIOJ_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 9)
#define GPIOK_PCLK_DI()     RCC->AHB1ENR &= ~(1 << 10)

/*
 * GPIO reset macro
 */
#define GPIOA_REG_RESET()        do{RCC->AHB1RSTR |= (1<<0); RCC->AHB1RSTR &= ~(1<<0);} while(0) // set to 1, then clear to 0
#define GPIOB_REG_RESET()        do{RCC->AHB1RSTR |= (1<<1); RCC->AHB1RSTR &= ~(1<<1);} while(0)
#define GPIOC_REG_RESET()        do{RCC->AHB1RSTR |= (1<<2); RCC->AHB1RSTR &= ~(1<<2);} while(0)
#define GPIOD_REG_RESET()        do{RCC->AHB1RSTR |= (1<<3); RCC->AHB1RSTR &= ~(1<<3);} while(0)
#define GPIOE_REG_RESET()        do{RCC->AHB1RSTR |= (1<<4); RCC->AHB1RSTR &= ~(1<<4);} while(0)
#define GPIOF_REG_RESET()        do{RCC->AHB1RSTR |= (1<<5); RCC->AHB1RSTR &= ~(1<<5);} while(0)
#define GPIOG_REG_RESET()        do{RCC->AHB1RSTR |= (1<<6); RCC->AHB1RSTR &= ~(1<<6);} while(0)
#define GPIOH_REG_RESET()        do{RCC->AHB1RSTR |= (1<<7); RCC->AHB1RSTR &= ~(1<<7);} while(0)
#define GPIOI_REG_RESET()        do{RCC->AHB1RSTR |= (1<<8); RCC->AHB1RSTR &= ~(1<<8);} while(0)
#define GPIOJ_REG_RESET()        do{RCC->AHB1RSTR |= (1<<9); RCC->AHB1RSTR &= ~(1<<9);} while(0)
#define GPIOK_REG_RESET()        do{RCC->AHB1RSTR |= (1<<10); RCC->AHB1RSTR &= ~(1<<10);} while(0)

/*
 * RCC register structure
 */
typedef struct {
    _vo int32_t CR;
    _vo int32_t PLLCFGR;
    _vo int32_t CFGR;
    _vo int32_t CIR;
    _vo int32_t AHB1RSTR;
    _vo int32_t AHB2RSTR;
    _vo int32_t AHB3RSTR;
    int32_t Reserved1;
    _vo int32_t APB1RSTR;
    _vo int32_t APB2RSTR;
    int32_t Reserved2[2];
    _vo int32_t AHB1ENR;
    _vo int32_t AHB2ENR;
    _vo int32_t AHB3ENR;
    int32_t Reserved3;
    _vo int32_t APB1ENR;
    _vo int32_t APB2ENR;
    int32_t Reserved4[2];
    _vo int32_t AHB1LPENR;
    _vo int32_t AHB2LPENR;
    _vo int32_t AHB3LPENR;
    int32_t Reserved5;
    _vo int32_t APB1LPENR;
    _vo int32_t APB2LPENR;
    int32_t Reserved6[2];
    _vo int32_t BDCR;
    _vo int32_t CSR;
    int32_t Reserved7[2];
    _vo int32_t SSCGR;
    _vo int32_t PLLI2SCFGR;
    _vo int32_t PLLSAICFGR;
    _vo int32_t DCKCFGR;
}RCC_RegDef_t, *pRCC_RegDef_t;

/*
 * RCC definition
 */
#define RCC     ((pRCC_RegDef_t) RCC_BASEADDR)


/*
 * EXTI register structure
 */

typedef struct {
    _vo uint32_t IMR;
    _vo uint32_t EMR;
    _vo uint32_t RTSR;
    _vo uint32_t FTSR;
    _vo uint32_t SWIER;
    _vo uint32_t PR;
}EXTI_RegDef_t, *pEXTI_RegDef_t;

/*
 * EXTI definition
 */
#define EXTI    ((pEXTI_RegDef_t) EXTI_BASEADDR)

/*
 * SYSCFG register structure
 */
typedef struct {
    _vo uint32_t MEMRMP;        // 0x00
    _vo uint32_t PMC;           // 0x04
    _vo uint32_t EXTICR[4];     // 0x08 ~ 0x14
    uint32_t RESERVED[2];       // 0x18 ~ 0x1C
    _vo uint32_t CMPCR;         // 0x20
}SYSCFG_RegDef_t, *pSYSCFG_RegDef_t ;

/*
 * SYSCFG definition
 */
#define SYSCFG  ((pSYSCFG_RegDef_t) SYSCFG_BASEADDR)
/*
 * System configuration controller clock macro
 */
#define SYSCFG_PCLK_EN()    RCC->APB2ENR |= (1 << 14)

#define SYSCFG_PCLK_DI()    RCC->APB2ENR &= ~(1 << 14)

/*
 * SPI register structure
 */
typedef struct{
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t CRCPR;
    _vo uint32_t RXCRCR;
    _vo uint32_t TXCRCR;
    _vo uint32_t I2SCFGR;
    _vo uint32_t I2SPR;
}SPI_RegDef_t, *pSPI_RegDef_t;
/**************************************************************************************
 * Bit position definitions for SPI peripheral
 **************************************************************************************/
/*
 * SPI control register_1 bit definition macros
 */
#define SPI_CR1_CPHA        0 // clock phase
#define SPI_CR1_CPOL        1 // clock polarity
#define SPI_CR1_MSTR        2 // master selection
#define SPI_CR1_BR          3 // [2:0] baud rate
#define SPI_CR1_SPE         6 // SPI enable
#define SPI_CR1_LSBFIRST    7 // frame format MSB or LSB first
#define SPI_CR1_SSI         8 // internall slave select, force its valie on NSS in software slave mode
#define SPI_CR1_SSM         9 // software slabe managemet
#define SPI_CR1_RXONLY      10 // receive only
#define SPI_CR1_DFF         11 //data frame format (8-bit of 16-bit)
#define SPI_CR1_CRCNEXT     12 // CRC transfer next
#define SPI_CR1_CRCEN       13 // hardware CRC calculation enable
#define SPI_CR1_BIDIOE      14 // output enable in bidirectional mode
#define SPI_CR1_BIDI        15 // bidirectional data mode enable

/*
 * SPI control register_2 bit definition macros
 */
#define SPI_CR2_RXDMAEN     0 // rx buffer DMA(direct memory access) enable
#define SPI_CR2_TXDMAEN     1 // tx buffer DMA enable
#define SPI_CR2_SSOE        2 // SS ourput enable TODO: ?
#define SPI_CR2_FRF         4 // frame format
#define SPI_CR2_ERRIE       5 // error interrupt enable
#define SPI_CR2_RXNEIE      6 // rx_buffer_not_empty_interrupt enable
#define SPI_CR2_TXEIE       7 // tx_buffer_empty_interrupt enable

/*
 * SPI status register bit definition macros
 */
#define SPI_SR_RXNE         0 // rx buffer not empty
#define SPI_SR_TXE          1 // tx buffer empty
#define SPI_SR_CHSIDE       2 // channel side TODO: ?
#define SPI_SR_UDR          3 // underrun flag TODO: ?
#define SPI_SR_CRCEER       4 // CRC error flag
#define SPI_SR_MODF         5 // mode fault TODO: ?
#define SPI_SR_OVR          6 // overrun flag TODO: ?
#define SPI_SR_BUSY          7 // busy flag
#define SPI_SR_FRE          8 // frame format error

/*
 * SPI definition
 */
#define SPI1    ((pSPI_RegDef_t) SPI1_BASEADDR)
#define SPI2    ((pSPI_RegDef_t) SPI2_BASEADDR)
#define SPI3    ((pSPI_RegDef_t) SPI3_BASEADDR)

/*
 * SPI clock macros
 */
#define SPI1_PCLK_EN()      RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN()      RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN()      RCC->APB1ENR |= (1 << 15)

#define SPI1_PCLK_DI()      RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()      RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI()      RCC->APB1ENR &= ~(1 << 15)

/*
 * SPI reset macros
 */
#define SPI1_REG_RESET()        do{RCC->APB2RSTR |= (1<<12); RCC->APB2RSTR &= ~(1<<12);} while(0)
#define SPI2_REG_RESET()        do{RCC->APB1RSTR |= (1<<14); RCC->APB1RSTR &= ~(1<<14);} while(0)
#define SPI3_REG_RESET()        do{RCC->APB1RSTR |= (1<<15); RCC->APB1RSTR &= ~(1<<15);} while(0)

/*
 * Basic timer register structure
 */
typedef struct {
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    uint32_t RESERVED;
    _vo uint32_t DIER;
    _vo uint32_t SR;
    _vo uint32_t EGR;
    uint32_t RESERVED1[3];
    _vo uint32_t CNT;
    _vo uint32_t PSC;
    _vo uint32_t ARR;
}Basic_TIM_RegDef_t, *pBasic_TIM_RegDef_t;

/*
 * USART register definition
 */
typedef struct {
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t BRR;
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t CR3;
    _vo uint32_t GTPR;
}USART_RegDef_t, *pUSART_RegDef_t;

/********************************************************************************************
 * Bit position definitions of USART peripheral
 ********************************************************************************************/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9



#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400)

#define USART1  (pUSART_RegDef_t) USART1_BASEADDR
#define USART2  (pUSART_RegDef_t) USART2_BASEADDR
#define USART3  (pUSART_RegDef_t) USART3_BASEADDR
#define UART4   (pUSART_RegDef_t) UART4_BASEADDR
#define UART5   (pUSART_RegDef_t) UART5_BASEADDR
#define USART6  (pUSART_RegDef_t) USART6_BASEADDR

/*
 * Clock Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()         (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()         (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << 5))


#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 5))


/******************************************************************************
 * I2C
 * ****************************************************************************/
#define I2C1_BASEADDR               (APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASE + 0x5c00)

typedef struct {
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t OAR1;
    _vo uint32_t OAR2;
    _vo uint32_t DR;
    _vo uint32_t SR1;
    _vo uint32_t SR2;
    _vo uint32_t CCR;
    _vo uint32_t TRISE;
    _vo uint32_t FLTR;
} I2C_RegDef_t;

#define I2C1        ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2        ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3        ((I2C_RegDef_t*) I2C3_BASEADDR)

#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1 << 23))

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS


/*****************************************************************
 * TIMER
 *****************************************************************/
/*
 * Timer clock macros
 */
#define TIM6_PCLK_EN()      RCC->APB1ENR |= (1 << 4)

#define TIM6_PCLK_DI()      RCC->APB1ENR &= ~(1 << 4)

/*
 * Timer reset macro
 */
#define TIM6_REG_RESET()        do{RCC->APB1RSTR |= (1<<4); RCC->APB1RSTR &= ~(1<<4);} while(0)

/*
 * IRQ numbers
 */
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40
#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3
#define NVIC_IRQ_PRI4       4
#define NVIC_IRQ_PRI5       5
#define NVIC_IRQ_PRI6       6
#define NVIC_IRQ_PRI7       7
#define NVIC_IRQ_PRI8       8
#define NVIC_IRQ_PRI9       9
#define NVIC_IRQ_PRI10      10
#define NVIC_IRQ_PRI11      11
#define NVIC_IRQ_PRI12      12
#define NVIC_IRQ_PRI13      13
#define NVIC_IRQ_PRI14      14
#define NVIC_IRQ_PRI15      15

/*
 * NVIC register structure
 */
typedef struct {
    _vo uint32_t ISER[8];
    uint32_t RESERVED0[24];
    _vo uint32_t ICER[8];
    uint32_t RESERVED1[24];
    _vo uint32_t ISPR[8];
    uint32_t RESERVED2[24];
    _vo uint32_t ICPR[8];
    uint32_t RESERVED3[24];
    _vo uint32_t IABR[8];
    uint32_t RESERVED4[56];
    _vo uint32_t IP[240];
    uint32_t RESERVED5[644];
    _vo uint32_t STIR;
}NVIC_RegDef_t, *pNVIC_RegDef_t;

#define NVIC        ((pNVIC_RegDef_t) 0xE000E100) // NOTE: Double parenthesis is NECESSARY

/*
 * NVIC functions
 */
void NVIC_SetPriority(int8_t IRQn, uint8_t priority);

/*
 * SCB register structure and related macros
 */
typedef struct {
    _ro uint32_t CPUID;
    _vo uint32_t ICSR;
    _vo uint32_t VTOR;
    _vo uint32_t AIRCR;
    _vo uint32_t SCR;
    _vo uint32_t CCR;
    _vo uint8_t SHPR[12]; /* !<Offset  0x18 (R/W) System Handlers Priority Registers (4-7, 8-11, 12-15)*/
    _vo uint32_t SHCSR;
    _vo uint32_t CFSR;
    _vo uint32_t HFSR;
    uint32_t RESERVED;
    _vo uint32_t MMAR;
    _vo uint32_t BFAR;
    _vo uint32_t AFSR;
}SCB_RegDef_t, *pSCB_RegDef_t;

#define SCB     ((pSCB_RegDef_t) 0xE000ED00)

#define Enable_SysTick()        SCB->SHCSR |= 0x1 << 11;

/*
 * Systick register structure
 */
typedef struct {
    _vo uint32_t CTRL;
    _vo uint32_t LOAD;
    _vo uint32_t VAL;
    _ro uint32_t CALIB;
}Systick_RegDef_t, *pSystick_RegDef_t;

#define SysTick     ((pSystick_RegDef_t) 0xE000E010)

extern volatile uint32_t sysTick_count;

void SysTick_Init(uint32_t load_value);

uint32_t get_tick();

void delay(uint32_t delay_in_ms);

/*
 * Generic macros
 */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET

#define disable_irq()       do{asm volatile("cpsid i");} while(0)
#define enable_irq()        do{asm volatile("cpsie i");} while(0)

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "epaper.h"
#include "GUI_Paint.h"
#include <cstdlib>

#endif //MCU1_STM32F407XX_H
