/*
 * stm32f103xx_gpio.c
 *
 *  Created on: 18-Jul-2023
 *      Author: Varun
 */

#include "stm32f103xx_gpio.h"


void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDIS){

	if(pGPIOx == GPIOA){
		ENorDIS ? GPIOA_PCLK_EN() : GPIOA_PCLK_DIS();
	}
	else if(pGPIOx == GPIOB){
		ENorDIS ? GPIOB_PCLK_EN() : GPIOB_PCLK_DIS();
		}
	else if(pGPIOx == GPIOC){
		ENorDIS ? GPIOC_PCLK_EN() : GPIOC_PCLK_DIS();
		}
	else if(pGPIOx == GPIOD){
		ENorDIS ? GPIOD_PCLK_EN() : GPIOD_PCLK_DIS();
		}
	else if(pGPIOx == GPIOE){
		ENorDIS ? GPIOE_PCLK_EN() : GPIOE_PCLK_DIS();
		}
	else if(pGPIOx == GPIOF){
		ENorDIS ? GPIOF_PCLK_EN() : GPIOF_PCLK_DIS();
		}
	else if(pGPIOx == GPIOG){
		ENorDIS ? GPIOG_PCLK_EN() : GPIOG_PCLK_DIS();
		}

}


void GPIO_Init(GPIO_Handle_t *pGPIO_Handle){

	uint8_t GPIO_PinNumber = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;

	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT){

		uint8_t GPIO_PinSpeed = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed;
		uint8_t GPIO_PinAltFunMode = pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode;
		uint8_t GPIO_PinOPType = pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType;

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << (4 * (GPIO_PinNumber % 8)));
		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] |= ((uint32_t)GPIO_PinSpeed << (4 * (GPIO_PinNumber % 8)));

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << ((4 * (GPIO_PinNumber % 8)) + 2));
		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] |= (((uint32_t)GPIO_PinOPType + GPIO_PinAltFunMode) << ((4 * (GPIO_PinNumber % 8)) + 2));

	}
	else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT){

		uint8_t GPIO_PinIPType = pGPIO_Handle->GPIO_PinConfig.GPIO_PinIPType;

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << (4 * (GPIO_PinNumber % 8)));

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << (4 * (GPIO_PinNumber % 8) + 2));
		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] |= ((uint32_t)GPIO_PinIPType << (4 * (GPIO_PinNumber % 8) + 2));

	}
	else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT){

		uint8_t GPIO_PinIPType = pGPIO_Handle->GPIO_PinConfig.GPIO_PinIPType;

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << (4 * (GPIO_PinNumber % 8)));

		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] &= ~( 3UL << (4 * (GPIO_PinNumber % 8) + 2));
		pGPIO_Handle->pGPIOx->CR[GPIO_PinNumber/8] |= ((uint32_t)GPIO_PinIPType << (4 * (GPIO_PinNumber % 8) + 2));

		AFIO_CLK_EN();

		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinIntType == GPIO_INTTYPE_FT){

			EXTI->RTSR &= ~( 1UL << GPIO_PinNumber);
			EXTI->FTSR |= ( 1UL << GPIO_PinNumber);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinIntType == GPIO_INTTYPE_RT){

			EXTI->FTSR &= ~( 1UL << GPIO_PinNumber);
			EXTI->RTSR |= ( 1UL << GPIO_PinNumber);

		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinIntType == GPIO_INTTYPE_FRT){

			EXTI->FTSR |= ( 1UL << GPIO_PinNumber);
			EXTI->RTSR |= ( 1UL << GPIO_PinNumber);

		}

		EXTI->IMR |= ( 1UL << GPIO_PinNumber);

		uint8_t GPIO_PORT_NUM = (pGPIO_Handle->pGPIOx == GPIOA) ? 0 :
				(pGPIO_Handle->pGPIOx == GPIOB) ? 1 :
				(pGPIO_Handle->pGPIOx == GPIOC) ? 2 :
				(pGPIO_Handle->pGPIOx == GPIOD) ? 3 :
				(pGPIO_Handle->pGPIOx == GPIOE) ? 4 :
				(pGPIO_Handle->pGPIOx == GPIOG) ? 5 : 6;

		AFIO->EXTICR[GPIO_PinNumber/4] |= ((uint32_t)GPIO_PORT_NUM << 4 * (GPIO_PinNumber % 4));
	}
}


void GPIO_DeInitPort(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
}

void GPIO_DeInitPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->CR[PinNumber/8] &= ~( 3UL << 4 * (PinNumber % 8));

	pGPIOx->CR[PinNumber/8] &= ~( 3UL << (4 * (PinNumber % 8) + 2));
	pGPIOx->CR[PinNumber/8] |= ( 1UL << (4 * (PinNumber % 8) + 3));

	uint32_t GPIO_PORT_NUM = (pGPIOx == GPIOA) ? 0 :
					(pGPIOx == GPIOB) ? 1 :
					(pGPIOx == GPIOC) ? 2 :
					(pGPIOx == GPIOD) ? 3 :
					(pGPIOx == GPIOE) ? 4 :
					(pGPIOx == GPIOG) ? 5 : 6;

	if((EXTI->IMR & ( 1UL << PinNumber))
			&& ((AFIO->EXTICR[PinNumber/4] & ( 5UL << 4 * (PinNumber % 4))) == (GPIO_PORT_NUM << 4 * (PinNumber % 4)))){

		EXTI->IMR &= ~( 1UL << PinNumber);

		AFIO->EXTICR[PinNumber/4] &= ~( 5UL << 4 * (PinNumber % 4));

		EXTI->FTSR &= ~( 1UL << PinNumber);
		EXTI->RTSR &= ~( 1UL << PinNumber);
	}
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value = (uint16_t)pGPIOx->IDR;
	return value;
}


uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIO_Handle){

	uint16_t tempValue = (uint16_t)pGPIO_Handle->pGPIOx->IDR;
	tempValue &= ( 1UL << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	return tempValue ? 1 : 0;
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data){

	pGPIOx->ODR = data;
}


void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIO_Handle, uint8_t SetOrReset){

	pGPIO_Handle->pGPIOx->ODR &= ~( 1UL << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->ODR |= ((uint32_t)SetOrReset << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
}


void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIO_Handle){

	pGPIO_Handle->pGPIOx->ODR ^= ( 1UL << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
}

void GPIO_IRQConfig(uint32_t IRQNumber, uint8_t ENorDIS){

	if(ENorDIS){

		if(IRQNumber <= 31){

			(*NVIC_ISER0) |= ( 1UL << IRQNumber % 32);
			(*NVIC_ICER0) &= ~( 1UL << IRQNumber % 32);

		}
		else if(IRQNumber >= 32 && IRQNumber <= 63){

			(*NVIC_ISER1) |= ( 1UL << IRQNumber % 32 );
			(*NVIC_ICER1) &= ~( 1UL << IRQNumber % 32);
		}
	}
	else{

		if(IRQNumber <= 31){

			(*NVIC_ICER0) |= ( 1UL << IRQNumber % 32);
			(*NVIC_ISER0) &= ~( 1UL << IRQNumber % 32);
		}
		else if(IRQNumber >= 32 && IRQNumber <= 63){

			(*NVIC_ISER1) |= ( 1UL << IRQNumber % 32 );
			(*NVIC_ISER1) &= ~( 1UL << IRQNumber % 32);
		}
	}
}


void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority){

	*(NVIC_IPR_BASEADDR + IRQNumber/4) &= ~( 15UL << (8 * (IRQNumber % 4) + 4));
	*(NVIC_IPR_BASEADDR + IRQNumber/4) |= ((uint32_t)IRQPriority << (8 * (IRQNumber % 4) + 4));
}


void GPIO_IRQHandle(uint8_t PinNumber){

	if(EXTI->PR & (1UL  << PinNumber)){

		EXTI->PR |= ( 1UL << PinNumber);
	}
}
