/*
 * stm32f103xx_spi.h
 *
 *  Created on: 10-Aug-2023
 *      Author: Varun
 */

#ifndef INC_STM32F103XX_SPI_H_
#define INC_STM32F103XX_SPI_H_

#include "stm32f103xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;

}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;

}SPI_Handle_t;

void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);

void SPI_IRQConfig(void);
void SPI_IRQPriorityConfig(void);
void SPI_IRQHandle(void);


/***************************************************SPI MACROS**************************************************/

#define SPI_MODE_SLAVE                      0
#define SPI_MODE_MASTER                     1

#define SPI_BUS_CONFIG_FD                   0
#define SPI_BUS_CONFIG_HD_RXONLY            1
#define SPI_BUS_CONFIG_HD_TXONLY            2
#define SPI_BUS_CONFIG_SIMP_RXONLY          3
#define SPI_BUS_CONFIG_SIMP_TXONLY          4

#define SPI_DFF_8BIT                        0
#define SPI_DFF_16BIT                       1

#define SPI_CPOL_LOW                        0
#define SPI_CPOL_HIGH                       1

#define SPI_CPHA_1ST_CLK                    0
#define SPI_CPHA_2ND_CLK                    1

#define SPI_SPEED_DIV2                      0
#define SPI_SPEED_DIV4                      1
#define SPI_SPEED_DIV8                      2
#define SPI_SPEED_DIV16                     3
#define SPI_SPEED_DIV32                     4
#define SPI_SPEED_DIV64                     5
#define SPI_SPEED_DIV128                    6
#define SPI_SPEED_DIV256                    7

#define SPI_SOFTSLAVE_EN                    1
#define SPI_SOFTSLAVE_DIS                   0

/*
 * SPI REGISTER BIT FIELDS
 * */

#define SPI_CR1_CPHA                        0
#define SPI_CR1_CPOL                        1
#define SPI_CR1_MSTR                        2
#define SPI_CR1_BR_BASE                     3
#define SPI_CR1_SPE                         6
#define SPI_CR1_LSBFIRST                    7
#define SPI_CR1_SSI                         8
#define SPI_CR1_SSM                         9
#define SPI_CR1_RXONLY                      10
#define SPI_CR1_DFF                         11
#define SPI_CR1_CRCNEXT                     12
#define SPI_CR1_CRCEN                       13
#define SPI_CR1_BIDIOE                      14
#define SPI_CR1_BIDIMODE                    15

#define SPI_CR2_RXDMAEN                     0
#define SPI_CR2_TXDMAEN                     1
#define SPI_CR2_SSOE                        2
#define SPI_CR2_ERRIE                       5
#define SPI_CR2_RXNEIE                      6
#define SPI_CR2_TXNEIE                      7

#define SPI_SR_RXNE_FLAG                    0
#define SPI_SR_TXE_FLAG                     1
#define SPI_SR_CHSIDE_FLAG                  2
#define SPI_SR_UDE_FLAG                     3
#define SPI_SR_CRCERR_FLAG                  4
#define SPI_SR_MODF_FLAG                    5
#define SPI_SR_OVR_FLAG                     6
#define SPI_SR_BSY_FLAG                     7


#endif /* INC_STM32F103XX_SPI_H_ */
