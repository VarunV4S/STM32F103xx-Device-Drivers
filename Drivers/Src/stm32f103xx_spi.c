/*
 * stm32f103xx_spi.c
 *
 *  Created on: 10-Aug-2023
 *      Author: Varun
 */

#include "stm32f103xx_spi.h"


void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS){

	if(ENorDIS == ENABLE){

		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{

		if(pSPIx == SPI1){
			SPI1_PCLK_DIS();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DIS();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DIS();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPI_Handle){

	pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_MSTR);
	pSPI_Handle->pSPIx->CR[0] |= ((uint32_t)pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD
			|| pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_TXONLY){

		pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_BIDIMODE);
		pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_RXONLY);
	}
	else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD_RXONLY){

		pSPI_Handle->pSPIx->CR[0] |= ( 1UL << SPI_CR1_BIDIMODE);
		pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_BIDIOE);
	}
	else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD_TXONLY){

		pSPI_Handle->pSPIx->CR[0] |= ( 1UL << SPI_CR1_BIDIMODE);
		pSPI_Handle->pSPIx->CR[0] |= ( 1UL << SPI_CR1_BIDIOE);
	}
	else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY){

		pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_BIDIMODE);
		pSPI_Handle->pSPIx->CR[0] |= ( 1UL << SPI_CR1_RXONLY);
	}

	pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_DFF);
	pSPI_Handle->pSPIx->CR[0] |= ( (uint32_t)pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_CPHA);
	pSPI_Handle->pSPIx->CR[0] |= ( (uint32_t)pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_CPOL);
	pSPI_Handle->pSPIx->CR[0] |= ( (uint32_t)pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	pSPI_Handle->pSPIx->CR[0] &= ~( 1UL << SPI_CR1_SSM);
	pSPI_Handle->pSPIx->CR[0] |= ( (uint32_t)pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->CR[0] &= ~( 7UL << SPI_CR1_BR_BASE);
	pSPI_Handle->pSPIx->CR[0] |= ( (uint32_t)pSPI_Handle->SPI_Config.SPI_Speed << SPI_CR1_BR_BASE);
}


void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length){

	while(Length > 0){

		//while(!(pSPIx->SR & ( 1UL << SPI_SR_TXE_FLAG)));
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG) == FLAG_RESET);

		if(pSPIx->CR[0] & ( 1UL << SPI_CR1_DFF)){

			pSPIx->DR = *((uint16_t*)pTxBuffer);
			(uint16_t*)pTxBuffer++;
			Length -= 2;
		}
		else{

			pSPIx->DR = *((uint8_t*)pTxBuffer);
			pTxBuffer++;
			Length--;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length){

	while(Length > 0){

		while(!(pSPIx->SR & ( 1UL << 0)));

		if(pSPIx->CR[0] & ( 1UL << 11)){

			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			Length -= 2;
		}
		else{

			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			Length--;
		}
	}
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS){

	if(ENorDIS == ENABLE){

		pSPIx->CR[0] |= ( 1UL << SPI_CR1_SPE);
	}
	else{

		pSPIx->CR[0] &= ~( 1UL << SPI_CR1_SPE);
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName){

	if(pSPIx->SR & ( 1UL << flagName)){

		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS){

	if(ENorDIS == ENABLE){

		pSPIx->CR[0] |= ( 1UL << SPI_CR1_SSI);
	}
	else{

		pSPIx->CR[0] &= ~( 1UL << SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS){

	if(ENorDIS == ENABLE){

		pSPIx->CR[1] |= ( 1UL << SPI_CR2_SSOE);
	}
	else{

		pSPIx->CR[1] &= ~( 1UL << SPI_CR2_SSOE);
	}
}


void SPI_MasterModeConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS){

	if(ENorDIS == ENABLE){

		pSPIx->CR[0] |= ( 1UL << SPI_CR1_MSTR);
	}
	else{

		pSPIx->CR[0] &= ~( 1UL << SPI_CR1_MSTR);
	}
}
