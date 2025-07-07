/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 3, 2025
 *      Author: goato
 */


#include"stm32f407xx.h"
#include"stm32f407xx_gpio_driver.h"


/**
 * @fn          void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
 *
 * @brief       Enables or disables the peripheral clock for the given GPIO port.
 *
 * @param[in]   pGPIOx     Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB).
 * @param[in]   EnOrDis    Enable or disable the peripheral clock.
 *                         Use macros: ENABLE or DISABLE.
 *
 * @return      None
 *
 * @note        This function must be called before using any GPIO peripheral functions,
 *              otherwise the GPIO registers will not be accessible.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
    if(EnOrDis == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else // DISABLE
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DIS();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DIS();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DIS();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DIS();
        }
    }
}

/**
 * @fn          void GPIO_Init(GPIO_handle_t *pGPIOHandle)
 *
 * @brief       Initializes the GPIO peripheral according to the specified parameters
 *              in the GPIO handle structure.
 *
 * @param[in]   pGPIOHandle   Pointer to GPIO_handle_t structure that contains
 *                            the configuration information for the specified GPIO pin.
 *
 * @return      None
 *
 * @note        - This function configures mode, speed, pull-up/down, output type, and alternate function.
 *              - Make sure the peripheral clock is enabled before calling this function.
 */
void GPIO_Init(GPIO_handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{

	}
	temp = 0;

	//2. configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pupd setting
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the optype
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/**
 * @fn          void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
 *
 * @brief       Resets the specified GPIO peripheral registers to their default values.
 *
 * @param[in]   pGPIOx     Pointer to GPIO base address (e.g., GPIOA, GPIOB, ...).
 *
 * @return      None
 *
 * @note        Internally uses the RCC AHB1RSTR register to reset the corresponding GPIO block.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/**
 * @fn          uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 *
 * @brief       Reads the value (logic level) of a specific input pin from the given GPIO port.
 *
 * @param[in]   pGPIOx      Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB, ...).
 * @param[in]   PinNumber   GPIO pin number (0 to 15) to read.
 *
 * @return      uint8_t     Returns 0 if pin is low, 1 if pin is high.
 *
 * @note        Make sure the pin is configured as input or alternate function with input mode.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/**
 * @fn          uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
 *
 * @brief       Reads the entire 16-bit input port value of the specified GPIO port.
 *
 * @param[in]   pGPIOx      Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB, ...).
 *
 * @return      uint16_t    16-bit value representing logic level of all 16 pins.
 *
 * @note        Each bit in the returned value corresponds to one pin (bit 0 = pin 0, ..., bit 15 = pin 15).
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/**
 * @fn          void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
 *
 * @brief       Sets or clears the specified GPIO output pin.
 *
 * @param[in]   pGPIOx      Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB, ...).
 * @param[in]   PinNumber   GPIO pin number (0 to 15) to write.
 * @param[in]   Value       Logic level to write:
 *                          - 0: Clear pin (logic low)
 *                          - 1: Set pin (logic high)
 *
 * @return      None
 *
 * @note        The pin must be configured as output or alternate function (push-pull/open-drain).
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**
 * @fn          void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
 *
 * @brief       Writes a 16-bit value directly to the output data register (ODR) of the GPIO port.
 *
 * @param[in]   pGPIOx      Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB, ...).
 * @param[in]   Value       16-bit value to write to the port.
 *
 * @return      None
 *
 * @note        This operation overwrites all output pins of the port simultaneously.
 *              Be careful not to affect unintended pins.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**
 * @fn          void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 *
 * @brief       Toggles the current logic state of the specified output pin.
 *
 * @param[in]   pGPIOx      Pointer to GPIO peripheral base address (e.g., GPIOA, GPIOB, ...).
 * @param[in]   PinNumber   GPIO pin number (0 to 15) to toggle.
 *
 * @return      None
 *
 * @note        This is useful for applications such as blinking LEDs or generating square waves.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandling(uint8_t PinNumber);
