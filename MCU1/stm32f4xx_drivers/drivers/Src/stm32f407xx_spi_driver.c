/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 29, 2025
 *      Author: goato
 */

#include <stm32f407xx_spi_driver.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	 while(Len > 0)
	 {
		 //1. wait until RXE is set
		 while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		 //2. check the DFF bit in CR1
		 if(pSPIx->CR1 & (1U << SPI_CR1_DFF))
		 {
			 //16 bit DFF
			 //1. load the data form DR to RXbuffer address
			 *((uint16_t*)pRxBuffer) = pSPIx->DR;
			 Len -= 2;
			 (uint16_t*)pRxBuffer++;
		 }else
		 {
			 //8 bit DFF
			 *pRxBuffer = pSPIx->DR;
			 Len--;
			 pRxBuffer++;
		 }
	 }
}

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

/**
 * @fn		void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
 *
 * @brief	Configures the interrupt for a specific IRQ line in the NVIC.
 *
 * @param	IRQNumber   The IRQ number (0 to 239 depending on MCU) to configure.
 * @param	IRQPriority The priority level for the IRQ (0 = highest, 15 = lowest).
 *
 * @note	This function:
 *			- Calculates the appropriate ISER/ICER register and bit for the IRQ
 *			- Enables or disables the IRQ in NVIC based on EnOrDis
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{
	uint8_t reg_idx = IRQNumber / 32;
	uint8_t bit_pos = IRQNumber % 32;

	if(reg_idx >= 8) return;

	if(EnOrDis == ENABLE)
	{
		NVIC_ISER_BASE_ADDR[reg_idx] |= (1U << bit_pos);
	}
	else // DISBALE
	{
		NVIC_ICER_BASE_ADDR[reg_idx] |= (1U << bit_pos);
	}
}

/**
 * @fn		void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
 *
 * @brief	Configures the Priority for a specific IRQ line in the NVIC.
 *
 * @param	IRQNumber   The IRQ number (0 to 239 depending on MCU) to configure.
 * @param	IRQPriority The priority level for the IRQ (0 = highest, 15 = lowest).
 *
 * @note	This function sets the priority in the NVIC_IPR register
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift = (iprx_section * 8) + (8 - PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift);
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_RX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/**
 * @fn		void SPI_IRQHandling(uint8_t PinNumber)
 *
 * @brief	Handles the interrupt event for a specific SPI pins.
 *
 * @param	PinNumber	The SPI pin number corresponding to the EXTI line.
 *
 * @note	This function clears the pending bit in the EXTI PR register for the given pin.
 * 			It should be called inside the corresponding EXTI interrupt handler.
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR1 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}
	//check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	 // check the DFF bit in CR1
	 if(pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	 {
		 //16 bit DFF
		 //1. load the data in to the DR
		 pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		 pSPIHandle->TxLen -= 2;
		 pSPIHandle->pTxBuffer += 2;
	 }else
	 {
		 //8 bit DFF
		 pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		 pSPIHandle->TxLen--;
		 pSPIHandle->pTxBuffer++;
	 }

	 if(!pSPIHandle->TxLen)
	 {
		 //Txlen is 0, close the spi communication and imform the application that Tx is over.
		 //this prevents interrupt from setting up of TXE flag
		 SPI_CloseTransmisson(pSPIHandle);
		 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	 }

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	 // check the DFF bit in CR1
	 if(pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	 {
		 //16 bit DFF
		 //1. load the data in to the DR
		 pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
		 pSPIHandle->RxLen -= 2;
		 pSPIHandle->pRxBuffer += 2;
	 }else
	 {
		 //8 bit DFF
		 pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
		 pSPIHandle->RxLen--;
		 pSPIHandle->pRxBuffer++;
	 }

	 if(!pSPIHandle->TxLen)
	 {
		 //Txlen is 0, close the spi communication and imform the application that Tx is over.
		 //this prevents interrupt from setting up of TXE flag
		 SPI_CloseReception(pSPIHandle);
		 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	 }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	 (void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	 pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	 pSPIHandle->pTxBuffer = NULL;
	 pSPIHandle->TxLen = 0;
	 pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	 pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	 pSPIHandle->pRxBuffer = NULL;
	 pSPIHandle->RxLen = 0;
	 pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implement. The application may override this function.
}
