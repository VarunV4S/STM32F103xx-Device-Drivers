/*
 * stm32f103xx_gpio.h
 *
 *  Created on: 18-Jul-2023
 *      Author: Varun
 */

#ifndef INC_STM32F103XX_GPIO_H_
#define INC_STM32F103XX_GPIO_H_


#include "stm32f103xx.h"


typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinIPType;
	uint8_t GPIO_PinAltFunMode;
	uint8_t GPIO_PinIntType;

}GPIO_Config_t;


typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_Config_t GPIO_PinConfig;

}GPIO_Handle_t;


void GPIO_Init(GPIO_Handle_t * pGPIO_Handle);
void GPIO_DeInitPort(GPIO_RegDef_t *pGPIOx);
void GPIO_DeInitPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_PCLKControl(GPIO_RegDef_t* pGPIOx, uint8_t ENorDIS);

uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t * pGPIO_Handle);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);

void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIO_Handle, uint8_t SetOrReset);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data);
void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIO_Handle);

void GPIO_IRQConfig(uint32_t IRQNumber, uint8_t ENorDIS);
void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandle(uint8_t PinNumber);

/********************************************************GPIO MACROS**********************************************************/


#define GPIO_MODE_INPUT        0
#define GPIO_MODE_OUTPUT       1
#define GPIO_MODE_INT          2

#define GPIO_SPEED_LOW         1
#define GPIO_SPEED_MED         2
#define GPIO_SPEED_HIGH        3

#define GPIO_OPTYPE_PP         0
#define GPIO_OPTYPE_OD         1

#define GPIO_IPTYPE_AN         0
#define GPIO_IPTYPE_FL         1
#define GPIO_IPTYPE_PUPD       2

#define GPIO_ALTFUN_EN         2
#define GPIO_ALTFUN_DIS        0

#define GPIO_INTTYPE_FT        0
#define GPIO_INTTYPE_RT        1
#define GPIO_INTTYPE_FRT       2

#define GPIO_PIN_0             0
#define GPIO_PIN_1             1
#define GPIO_PIN_2             2
#define GPIO_PIN_3             3
#define GPIO_PIN_4             4
#define GPIO_PIN_5             5
#define GPIO_PIN_6             6
#define GPIO_PIN_7             7
#define GPIO_PIN_8             8
#define GPIO_PIN_9             9
#define GPIO_PIN_10            10
#define GPIO_PIN_11            11
#define GPIO_PIN_12            12
#define GPIO_PIN_13            13
#define GPIO_PIN_14            14
#define GPIO_PIN_15            15


#endif /* INC_STM32F103XX_GPIO_H_ */
