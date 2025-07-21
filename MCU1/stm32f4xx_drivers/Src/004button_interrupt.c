/*
 * 004button_interrupt.c
 *
 *  Created on: Jul 19, 2025
 *      Author: goato
 */

#include <string.h>
#include"stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{
	GPIO_handle_t GPIOLED, GPIOButton;

	GPIOLED.pGPIOx = GPIOD;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIOButton.pGPIOx = GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLED);
	GPIO_Init(&GPIOButton);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
