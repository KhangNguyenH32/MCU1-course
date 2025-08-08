/*
 * 006spi_tx_testing.c
 *
 *  Created on: Aug 4, 2025
 *      Author: goato
 */

#include <string.h>
#include "stm32f407xx.h"

/*
 * PB12: SPI2_NSS
 * PB13: SPI2_SLCK
 * PB14: SPI2_MISO
 * PB15: SPI2_MOSI
 * Alter function mode: 5
 */

void delay(void)
{
	for(int32_t i = 0; i < 200000; i++);
}

void GPIO_ButtonInits(void)
{
		GPIO_handle_t gpioButton;

		gpioButton.pGPIOx = GPIOA;
		gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
		gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&gpioButton);

	    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
}

void SPI2_GPIOInits(void)
{
	GPIO_handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates sclk of 8Mhz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

int main(void)
{

	//initialize user button and configure interrupt EXTI
	GPIO_ButtonInits();

	//this function is used to initialize the GPIO pins to behave as PSI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//this make NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();

	//clear pending bit
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	char user_data[] = "Hello World";

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);
}
