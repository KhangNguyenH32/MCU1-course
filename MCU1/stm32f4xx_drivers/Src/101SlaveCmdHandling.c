/*
 * 002SlaveCmdHandling.c
 *
 *  Created on: Aug 12, 2025
 *      Author: goato
 */


#include "stm32f407xx.h"
#include "string.h"
#include <stdio.h>

uint8_t led = 12;		// Slave LED digital I/O pin.

uint8_t dataBuff[255];

uint8_t board_id[10] = "A";

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

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

void GPIO_Inits()
{
	GPIO_Handle_t LED2;
	LED2.pGPIOx = GPIOC;
	LED2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	LED2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LED2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&LED2);
}

uint8_t checkData(uint8_t command)
{
  switch (command) {
    case COMMAND_ID_READ:
    case COMMAND_LED_READ:
    case COMMAND_LED_CTRL:
    case COMMAND_PRINT:
    case COMMAND_SENSOR_READ:
      return ACK;
    default:
      return NACK;
  }
}

// The loop function runs continuously after setup().
int main(void)
{
	GPIO_Inits();
	SPI1_GPIOInits();
	SPI1_Inits();
	SPI_PeripheralControl(SPI1, ENABLE);

	uint8_t pin, value, command, dummy, ackornack=NACK;
	while(1)
	{

		printf("Slave Waiting for ss to go low \n");

		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_4));

		SPI_ReceiveData(SPI1, &command, 1);

		ackornack = checkData(command);

		SPI_SendData(SPI1, &ackornack, 1);

		SPI_ReceiveData(SPI1, &dummy, 1);

		if(command == COMMAND_LED_CTRL)
		{
			SPI_ReceiveData(SPI1, &pin, 1);
			SPI_ReceiveData(SPI1, &value, 1);
			GPIO_WriteToOutputPin(GPIOC, pin, value);
		}
	}
}
