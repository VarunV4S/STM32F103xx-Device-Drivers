#include "stm32f103xx_gpio.h"
#include "stm32f103xx_irqhandlers.h"

void EXTI0_IRQHandler(void){
	GPIO_IRQHandle(GPIO_PIN_0);
}
