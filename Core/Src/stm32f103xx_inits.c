/*
 * stm32f103xx_inits.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Varun
 */

#include "main.h"

extern GPIO_Handle_t PC13;


void BM_GPIO_Init(){

	PC13.pGPIOx = GPIOC;
	PC13.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	PC13.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	PC13.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	PC13.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MED;
	PC13.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFUN_DIS;
	PC13.GPIO_PinConfig.GPIO_PinIPType = GPIO_IPTYPE_PUPD;
	PC13.GPIO_PinConfig.GPIO_PinIntType = GPIO_INTTYPE_FT;

	GPIO_PCLKControl(GPIOC, ENABLE);
	GPIO_Init(&PC13);
}


