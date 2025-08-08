/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 29, 2025
 *      Author: goato
 */

#include"stm32f407xx_spi_driver.h"

/**********************************************************
 * @fn		- SPI_PeriClockControl
 *
 * @brief	- Enable or Disable the peripheral clock for the given SPI module
 *
 * @param[in] pSPIx		- Pointer to SPI peripheral base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the clock ON or OFF
 *
 * @return	- none
 *
 * @note	- This function should be call before using any SPI peripheral function
 **********************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else // DISABLE
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DIS();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_DIS();
		}
	}
}

/**********************************************************
 * @fn		- SPI_Init
 *
 * @brief	- Configures and initializes the specified SPI peripheral
 *
 * @param[in] pSPIHandle - Pointer to SPI_Handle_t structure that contains
 * 						   the configuration information
 *
 * @return	- none
 *
 * @note	- Call SPI_PeriClockControl() before invoking this function
 **********************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//configure the SPI_CR1
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be clear
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**********************************************************
 * @fn		- SPI_Deinit
 *
 * @brief	- Resets the specified SPI peripheral registers to their default reset values.
 *
 * @param[in] pSPIx - Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 *
 * @return	- none
 *
 * @note	- Typically uses the RCC reset register (if available)
 **********************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**********************************************************
 * @fn		- SPI_SendData
 *
 * @brief	- Send data over the SPI peripheral to a connected device
 *
 * @param[in] pSPIx		- Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] pTxbuffer	- Pointer to the data buffer to transmit
 * @param[in] Len		- Length of the data to be sent (in byte)
 *
 * @return	- none
 *
 * @note	- This is a blocking call and waits until all the bytes are transmitted
 **********************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	 while(Len > 0)
	 {
		 //1. wait until TXE is set
		 while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		 //2. check the DFF bit in CR1
		 if(pSPIx->CR1 & (1U << SPI_CR1_DFF))
		 {
			 //16 bit DFF
			 //1. load the data in to the DR
			 pSPIx->DR = *((uint16_t*)pTxBuffer);
			 Len -= 2;
			 (uint16_t*)pTxBuffer++;
		 }else
		 {
			 //8 bit DFF
			 pSPIx->DR = *pTxBuffer;
			 Len--;
			 pTxBuffer++;
		 }
	 }
}

/**********************************************************
 * @fn		- SPI_ReceiveData
 *
 * @brief	- Receive data over the SPI peripheral to a connected device
 *
 * @param[in] pSPIx		- Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] pTxbuffer	- Pointer to the data buffer to transmit
 * @param[in] Len		- Length of the data to be Received (in byte)
 *
 * @return	- none
 *
 * @note	- This is a blocking call and waits until all the bytes are transmitted
 **********************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**********************************************************
 * @fn		- SPI_PeripheralControl
 *
 * @brief	- Enable or Disable SPIx peripheral
 *
 * @param[in] pSPIx		- Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the SPI peripheral ON or OFF
 *
 * @return	- none
 *
 * @note	- you invoking this function before use SPI
 **********************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	}else //DISABLE
	{
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}

/**********************************************************
 * @fn		- SPI_SSIConfig
 *
 * @brief	- configure SSI bit of SPIx
 *
 * @param[in] pSPIx		- Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the SSI bit ON or OFF
 *
 * @return	- none
 *
 * @note	- use for master mode when SSM = 1 to avoid MODF error
 **********************************************************/
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/**********************************************************
 * @fn		- SPI_SSOEConfig
 *
 * @brief	- configure SSOE bit of SPIx
 *
 * @param[in] pSPIx		- Pointer to SPI base address (e.g., SPI1, SPI2, SPI3)
 * @param[in] EnOrDis	- ENABLE or DISABLE macro to turn the SSOE bit ON or OFF
 *
 * @return	- none
 *
 * @note	- Only relevant in master mode when SSM = 1.
 **********************************************************/
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}
