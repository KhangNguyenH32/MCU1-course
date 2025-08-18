/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 17, 2025
 *      Author: goato
 */

#include <stm32f407xx_i2c_driver.h>

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8 ,16};

/**********************************************************
 * @fn		- I2C_PeripheralControl
 *
 * @brief	- Enable or Disable I2Cx peripheral
 *
 * @param[in] pI2Cx		- Pointer to I2C base address (e.g., I2C1, I2C2, I2C3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the I2C peripheral ON or OFF
 *
 * @return	- none
 *
 * @note	- you invoking this function before use I2C
 **********************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR1 |= (1U << I2C_CR1_PE);
	}else //DISABLE
	{
		pI2Cx->CR1 &= ~(1U << I2C_CR1_PE);
	}
}

/**********************************************************
 * @fn		- I2C_PeriClockControl
 *
 * @brief	- Enable or Disable the peripheral clock for the given I2C module
 *
 * @param[in] pI2Cx		- Pointer to I2C peripheral base address (e.g., I2C1, I2C2, I2C3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the clock ON or OFF
 *
 * @return	- none
 *
 * @note	- This function should be call before using any I2C peripheral function
 **********************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else // DISABLE
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}

/**********************************************************
 * @fn		- I2C_Init
 *
 * @brief	- Configures and initializes the specified I2C peripheral
 *
 * @param[in] pI2CHandle - Pointer to I2C_Handle_t structure that contains
 * 						   the configuration information
 *
 * @return	- none
 *
 * @note	- Call I2C_PeriClockControl() before invoking this function
 **********************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

uint32_t RCC_GetPLLOutputClock()
{

}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, ahbp, apb1p, temp;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p  = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}


	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}
