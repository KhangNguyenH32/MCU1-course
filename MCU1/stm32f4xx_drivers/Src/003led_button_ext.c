/*
 * 003led_button_ext.c
 *
 *  Created on: Jul 13, 2025
 *      Author: goato
 */


#include"stm32f407xx.h"

#define LOW 0
#define HIGH 1
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void)
{
	GPIO_handle_t gpioLED, gpioButton;

	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpioButton.pGPIOx = GPIOB;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&gpioLED);
	GPIO_Init(&gpioButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_11) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}
}
