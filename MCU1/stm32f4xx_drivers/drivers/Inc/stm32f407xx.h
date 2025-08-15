/*
 * stm32f407xx.h
 *
 *  Created on: Jul 3, 2025
 *      Author: goato
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

/***********************************START:Processor Specific Details***********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER_BASE_ADDR	((__vo uint32_t*)0xE000E100)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER_BASE_ADDR	((__vo uint32_t*)0xE000E180)

/*
 * ARM Cortex Mx processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implement in priority register
 */
#define PR_BITS_IMPLEMENTED	4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR	0x08000000U	// flash memory base address
#define SRAM1_BASEADDR	0x20000000U	// SRAM1 memory base address
#define SRAM2_BASEADDR	0x20001C00U	// SRAM2 memory base address
#define ROM_BASEADDR	0x1FFF0000
#define SRAM			SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR		0x40000000U
#define APB1PERIPH_BASEADDR	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHB1PERIPH_BASEADDR	0x40020000U
#define AHB2PERIPH_BASEADDR	0X50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: complete for all other peripherals
 */

#define GPIOA_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x0000))
#define GPIOB_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x0400))
#define GPIOC_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x0800))
#define GPIOD_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x0C00))
#define GPIOE_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x1000))
#define GPIOF_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x1400))
#define GPIOG_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x1800))
#define GPIOH_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x1C00))
#define GPIOI_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x2000))

#define RCC_BASEADDR ((AHB1PERIPH_BASEADDR) + (0x3800))

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: complete for all other peripherals
 */

#define I2C1_BASEADDR	((APB1PERIPH_BASEADDR) + (0x5400))
#define I2C2_BASEADDR	((APB1PERIPH_BASEADDR) + (0x5800))
#define I2C3_BASEADDR	((APB1PERIPH_BASEADDR) + (0x5C00))

#define SPI2_BASEADDR	((APB1PERIPH_BASEADDR) + (0x3800))
#define SPI3_BASEADDR	((APB1PERIPH_BASEADDR) + (0x3C00))

#define USART2_BASEADDR	((APB1PERIPH_BASEADDR) + (0x4400))
#define USART3_BASEADDR	((APB1PERIPH_BASEADDR) + (0x4800))
#define UART4_BASEADDR	((APB1PERIPH_BASEADDR) + (0x4C00))
#define UART5_BASEADDR	((APB1PERIPH_BASEADDR) + (0x5000))

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: complete for all other peripherals
 */

#define EXTI_BASEADDR	((APB2PERIPH_BASEADDR) + (0X3C00))
#define SPI1_BASEADDR	((APB2PERIPH_BASEADDR) + (0x3000))
#define USART1_BASEADDR	((APB2PERIPH_BASEADDR) + (0x1000))
#define USART6_BASEADDR	((APB2PERIPH_BASEADDR) + (0x1400))
#define SYSCFG_BASEADDR	((APB2PERIPH_BASEADDR) + (0x3800))

/*
 * Peripheral register definition structures
 */

typedef struct
{
	__vo uint32_t MODER;	// GPIO port mode register, address offset: 0x00
	__vo uint32_t OTYPER;	// GPIO port output type register, address offset: 0x04
	__vo uint32_t OSPEEDR;	// GPIO port output speed register, address offset: 0x08
	__vo uint32_t PUPDR;	// GPIO port pull-up/pull-down register, address offset: 0x0C
	__vo uint32_t IDR;		// GPIO port input data register, address offset: 0x10
	__vo uint32_t ODR;		// GPIO port output data register, address offset: 0x14
	__vo uint32_t BSRR;		// GPIO port bit set/reset register, address offset: 0x18
	__vo uint32_t LCKR;		// GPIO port configuration lock register, address offset: 0x1C
	__vo uint32_t AFR[2];	// AFR[0]: GPIO alternate function low register address offset:0x20
							// AFR[0]: GPIO alternate function high register address offset:0x24
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;       // SPI control register 1,           address offset: 0x00
	__vo uint32_t CR2;       // SPI control register 2,           address offset: 0x04
	__vo uint32_t SR;        // SPI status register,              address offset: 0x08
	__vo uint32_t DR;        // SPI data register,                address offset: 0x0C
	__vo uint32_t CRCPR;     // SPI CRC polynomial register,      address offset: 0x10
	__vo uint32_t RXCRCR;    // SPI RX CRC register,              address offset: 0x14
	__vo uint32_t TXCRCR;    // SPI TX CRC register,              address offset: 0x18
	__vo uint32_t I2SCFGR;   // SPI_I2S configuration register,   address offset: 0x1C
	__vo uint32_t I2SPR;     // SPI_I2S prescaler register,       address offset: 0x20
} SPI_RegDef_t;

typedef struct
{
	__vo uint32_t CR;           // Clock control register, address offset: 0x00
	__vo uint32_t PLLCFGR;      // PLL configuration register, address offset: 0x04
	__vo uint32_t CFGR;         // Clock configuration register, address offset: 0x08
	__vo uint32_t CIR;          // Clock interrupt register, address offset: 0x0C
	__vo uint32_t AHB1RSTR;     // AHB1 peripheral reset register, address offset: 0x10
	__vo uint32_t AHB2RSTR;     // AHB2 peripheral reset register, address offset: 0x14
	__vo uint32_t AHB3RSTR;     // AHB3 peripheral reset register, address offset: 0x18
	uint32_t reserved1;         // Reserved, address offset: 0x1C
	__vo uint32_t APB1RSTR;     // APB1 peripheral reset register, address offset: 0x20
	__vo uint32_t APB2RSTR;     // APB2 peripheral reset register, address offset: 0x24
	uint32_t reserved2[2];      // Reserved, address offset: 0x28-0x2C
	__vo uint32_t AHB1ENR;      // AHB1 peripheral clock enable register, address offset: 0x30
	__vo uint32_t AHB2ENR;      // AHB2 peripheral clock enable register, address offset: 0x34
	__vo uint32_t AHB3ENR;      // AHB3 peripheral clock enable register, address offset: 0x38
	__vo uint32_t reserved3;    // Reserved, address offset: 0x3C
	__vo uint32_t APB1ENR;      // APB1 peripheral clock enable register, address offset: 0x40
	__vo uint32_t APB2ENR;      // APB2 peripheral clock enable register, address offset: 0x44
	uint32_t reserved4[2];      // Reserved, address offset: 0x48-0x4C
	__vo uint32_t AHB1LPENR;    // AHB1 peripheral clock enable in low power mode register, address offset: 0x50
	__vo uint32_t AHB2LPENR;    // AHB2 peripheral clock enable in low power mode register, address offset: 0x54
	__vo uint32_t AHB3LPENR;    // AHB3 peripheral clock enable in low power mode register, address offset: 0x58
	uint32_t reserved5;         // Reserved, address offset: 0x5C
	__vo uint32_t APB1LPENR;    // APB1 peripheral clock enable in low power mode register, address offset: 0x60
	__vo uint32_t APB2LPENR;    // APB2 peripheral clock enable in low power mode register, address offset: 0x64
	uint32_t reserved6[2];      // Reserved, address offset: 0x68-0x6C
	__vo uint32_t BDCR;         // Backup dDDDomain control register, address offset: 0x70
} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;    // Interrupt mask register, address offset: 0x00
	__vo uint32_t EMR;    // Event mask register, address offset: 0x04
	__vo uint32_t RTSR;   // Rising trigger selection register, address offset: 0x08
	__vo uint32_t FTSR;   // Falling trigger selection register, address offset: 0x0C
	__vo uint32_t SWIER;  // Software interrupt event register, address offset: 0x10
	__vo uint32_t PR;     // Pending register, address offset: 0x14
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;    // Interrupt mask register, address offset: 0x00
	__vo uint32_t PMC;    // Event mask register, address offset: 0x04
	__vo uint32_t EXTICR[4];   // Rising trigger selection register, address offset: 0x08
		 uint32_t reserved[2];
	__vo uint32_t CMPCR;   // Falling trigger selection register, address offset: 0x0C
	} SYSCFG_RegDef_t;


/*
 * peripheral definition (Peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DIS()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Macros to reset SPIx peripheral
 */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)


/*
 * Return port code for given GPIOx base address
 */
/*
 * This marco returns a code(between 0-7) for a a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 : \
                                    (x == GPIOB) ? 1 : \
                                    (x == GPIOC) ? 2 : \
                                    (x == GPIOD) ? 3 : \
                                    (x == GPIOE) ? 4 : \
                                    (x == GPIOF) ? 5 : \
                                    (x == GPIOG) ? 6 : \
                                    (x == GPIOH) ? 7 : 0)

/*
 * IRQ(Interrupt Request) Number of STM32F407x MCU
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

/*
 * macro for all possible interrupt priority levels
 */
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI1	1
#define NVIC_IRQ_PRI2	2
#define NVIC_IRQ_PRI3	3
#define NVIC_IRQ_PRI4	4
#define NVIC_IRQ_PRI5	5
#define NVIC_IRQ_PRI6	6
#define NVIC_IRQ_PRI7	7
#define NVIC_IRQ_PRI8	8
#define NVIC_IRQ_PRI9	9
#define NVIC_IRQ_PRI10	10
#define NVIC_IRQ_PRI11	11
#define NVIC_IRQ_PRI12	12
#define NVIC_IRQ_PRI13	13
#define NVIC_IRQ_PRI14	14
#define NVIC_IRQ_PRI15	15

//some generic macros

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET 		SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
