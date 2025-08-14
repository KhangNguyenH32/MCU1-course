/*
 * 100slavemodeconfig.c
 *
 *  Created on: Aug 10, 2025
 *      Author: goato
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

char databuff[100];

void SPI1_GPIOInits()
{
	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOA;
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIpins);

	//MISO
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIpins);

	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIpins);

}

void SPI1_Inits()
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DIS;

	SPI_Init(&SPI1Handle);
}

int main(void)
{

	SPI1_GPIOInits();
	SPI1_Inits();

	SPI_PeripheralControl(SPI1, ENABLE);

	while(1)
	{

	printf("Slave Waiting for ss to go low \n");

	while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_4));

	uint8_t DataLen;
	SPI_ReceiveData(SPI1, &DataLen, 1);

	SPI_ReceiveData(SPI1, (uint8_t*)databuff, DataLen);


	databuff[DataLen] = '\0';

	printf("Rcvd: %s\n", databuff);
	printf("Length: %d\n", DataLen);
	}

}
