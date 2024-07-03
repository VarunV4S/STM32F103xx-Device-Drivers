/*
 * stm32f130xx.h
 *
 *  Created on: 17-Jul-2023
 *      Author: Varun
 */


#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_


#include <stdint.h>

#define _IO                       volatile


/*****************************************Arm Cortex M3 Specific Macros*****************************************************/

/*
 * NVIC Register Macros
 * */

#define NVIC_ISER0                       ((_IO uint32_t*)0xE000E100UL)
#define NVIC_ISER1                       ((_IO uint32_t*)0xE000E104UL)
#define NVIC_ISER2                       ((_IO uint32_t*)0xE000E108UL)

#define NVIC_ICER0                       ((_IO uint32_t*)0xE000E180UL)
#define NVIC_ICER1                       ((_IO uint32_t*)0xE000E184UL)
#define NVIC_ICER2                       ((_IO uint32_t*)0xE000E188UL)

#define NVIC_IPR_BASEADDR                ((_IO uint32_t*)0xE000E400UL)


/**************************************************Base Address Macros*******************************************************/

/*
 * Base Address of all Memories
 * */

#define FLASH_BASEADDR            0x08000000UL        /* Flash Memory Base Address - 0x08000000 */
#define SRAM_BASEADDR             0x20000000UL        /* SRAM Base Address - 0x20000000 */
#define ROM_BASEADDR              0x1FFFF000UL        /* System Memory (ROM) Base Address - 0x1FFFF000 */
#define SRAM                      SRAM_BASEADDR      /* SRAM Name Changed Here */


/*
 * Base Address of all APB and AHB Bus
 * */

#define PERIPH_BASE               0x40000000UL        /* Overall Peripheral Base Address */
#define APB1PERIPH_BASE           PERIPH_BASE        /* APB1  Peripheral Base Address - 0x40000000 */
#define APB2PERPH_BASE            0x40010000UL        /* APB2  Peripheral Base Address - 0x40010000 */
#define AHBPERIPH_BASE			  0x40018000UL        /* AHB  Peripheral Base Address - 0x40018000 */


/*
 * Base Address of all peripherals in APB2 Bus
 * */

#define GPIOA_BASEADDR           (APB2PERPH_BASE + 0x0800UL)
#define GPIOB_BASEADDR           (APB2PERPH_BASE + 0x0C00UL)
#define GPIOC_BASEADDR           (APB2PERPH_BASE + 0x1000UL)
#define GPIOD_BASEADDR           (APB2PERPH_BASE + 0x1400UL)
#define GPIOE_BASEADDR           (APB2PERPH_BASE + 0x1800UL)
#define GPIOF_BASEADDR           (APB2PERPH_BASE + 0x1C00UL)
#define GPIOG_BASEADDR           (APB2PERPH_BASE + 0x2000UL)

#define AFIO_BASEADDR            (APB2PERPH_BASE + 0x0000UL)

#define EXTI_BASEADDR            (APB2PERPH_BASE + 0x0400UL)

#define ADC1_BASEADDR            (APB2PERPH_BASE + 0x2400UL)
#define ADC2_BASEADDR            (APB2PERPH_BASE + 0x2800UL)
#define ADC3_BASEADDR            (APB2PERPH_BASE + 0x3C00UL)

#define TIM1_BASEADDR            (APB2PERPH_BASE + 0x2400UL)
#define TIM8_BASEADDR            (APB2PERPH_BASE + 0x2400UL)
#define TIM9_BASEADDR            (APB2PERPH_BASE + 0x2400UL)
#define TIM10_BASEADDR           (APB2PERPH_BASE + 0x2400UL)
#define TIM11_BASEADDR           (APB2PERPH_BASE + 0x2400UL)

#define USART1_BASEADDR          (APB2PERPH_BASE + 0x3800UL)

#define SPI1_BASEADDR            (APB2PERPH_BASE + 0x3000UL)


/*
 * Base Address of all peripherals in APB1 Bus
 * */

#define SPI2_BASEADDR           (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASEADDR           (APB1PERIPH_BASE + 0x3C00UL)

//#define TIM2_BASEADDR
//#define TIM3_BASEADDR
//#define TIM4_BASEADDR
//#define TIM5_BASEADDR
//#define TIM6_BASEADDR
//#define TIM7_BASEADDR
//#define TIM12_BASEADDR
//#define TIM13_BASEADDR
//#define TIM14_BASEADDR
//
//#define RTC_BASEADDR
//
//#define WWDG_BASEADDR
//#define IWDG_BASEADDR
//
//#define USART2_BASEADDR
//#define USART3_BASEADDR
//#define UART4_BASEADDR
//#define UART5_BASEADDR
//
//#define I2C1_BASEADDR
//#define I2C2_BASEADDR


/*
 * Base Address of all peripherals in AHB Bus
 * */

#define RCC_BASEADDR             0x40021000UL


/*
 * All APB2 peripherals Base Address Type cast
 * */

#define GPIOA                    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                    ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                    ((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define AFIO                     ((AFIO_RegDef_t*)AFIO_BASEADDR)

#define EXTI                     ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SPI1                     ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                     ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                     ((SPI_RegDef_t*)SPI3_BASEADDR)


/*
 * All AHB peripherals Base Address Type cast
 * */

#define RCC                      ((RCC_RegDef_t*)RCC_BASEADDR)


/**********************************************Peripherals Register Structures***********************************************/

/*
 *GPIO Register Structure
 */

typedef struct {
	_IO uint32_t CR[2];
	_IO uint32_t IDR;
	_IO uint32_t ODR;
	_IO uint32_t BSRR;
	_IO uint32_t BSR;
	_IO uint32_t LCKR;

}GPIO_RegDef_t;


/*
 * AFIO Register Structure
 * */

typedef struct {
	_IO uint32_t EVCR;
	_IO uint32_t MAPR;
	_IO uint32_t EXTICR[4];
	_IO uint32_t RESERVED0;
	_IO uint32_t MAPR1;

}AFIO_RegDef_t;


/*
 * RCC Register Structure
 * */

typedef struct {
	_IO uint32_t CR;
	_IO uint32_t CFGR;
	_IO uint32_t CIR;
	_IO uint32_t APB2RSTR;
	_IO uint32_t APB1RSTR;
	_IO uint32_t AHBENR;
	_IO uint32_t APB2ENR;
	_IO uint32_t APB1ENR;
	_IO uint32_t BDCR;
	_IO uint32_t CSR;
	_IO uint32_t AHBSTR;
	_IO uint32_t CFGR2;

}RCC_RegDef_t;


/*
 * EXTI Register Structure
 * */

typedef struct{
	_IO uint32_t IMR;
	_IO uint32_t EMR;
	_IO uint32_t RTSR;
	_IO uint32_t FTSR;
	_IO uint32_t SWIER;
	_IO uint32_t PR;

}EXTI_RegDef_t;


/*
 * SPI Register Structure
 * */

typedef struct{
	_IO uint32_t CR[2];
	_IO uint32_t SR;
	_IO uint32_t DR;
	_IO uint32_t CRCPR;
	_IO uint32_t RXCRCR;
	_IO uint32_t TXCRCR;
	_IO uint32_t I2SCFGR;
	_IO uint32_t I2SPR;

}SPI_RegDef_t;


/******************************************************General Macros********************************************************/


#define ENABLE                           1
#define DISABLE                          0

#define SET                              1
#define RESET                            0

#define GPIO_PIN_SET                     1
#define GPIO_PIN_RESET                   0

#define HIGH                             1
#define LOW                              0

#define BIT_1                            1UL
#define BIT_2                            3UL
#define BIT_3                            7UL
#define BIT_4                            15UL
#define BIT_5                            31UL

#define FLAG_RESET                       0
#define FLAG_SET                         1



/**************************************************Peripheral Control Macros*************************************************/

/*
 * GPIOx Clock Enable
 * */

#define GPIOA_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 2))
#define GPIOB_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 3))
#define GPIOC_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 4))
#define GPIOD_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 5))
#define GPIOE_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 6))
#define GPIOF_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 7))
#define GPIOG_PCLK_EN()                  (RCC->APB2ENR |= (1UL << 8))

/*
 * GPIOx Clock Disable
 * */

#define GPIOA_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 2))
#define GPIOB_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 3))
#define GPIOC_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 4))
#define GPIOD_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 5))
#define GPIOE_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 6))
#define GPIOF_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 7))
#define GPIOG_PCLK_DIS()                 (RCC->APB2ENR &= ~(1UL << 8))

/*
 * GPIOx Register Reset
 * */

#define GPIOA_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 2)); (RCC->APB2RSTR &= ~(1UL << 2)); } while(0)
#define GPIOB_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 3)); (RCC->APB2RSTR &= ~(1UL << 3)); } while(0)
#define GPIOC_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 4)); (RCC->APB2RSTR &= ~(1UL << 4)); } while(0)
#define GPIOD_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 5)); (RCC->APB2RSTR &= ~(1UL << 5)); } while(0)
#define GPIOE_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 6)); (RCC->APB2RSTR &= ~(1UL << 6)); } while(0)
#define GPIOF_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 7)); (RCC->APB2RSTR &= ~(1UL << 7)); } while(0)
#define GPIOG_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 8)); (RCC->APB2RSTR &= ~(1UL << 8)); } while(0)

/*
 * AFIO Clock Enable
 * */

#define AFIO_CLK_EN()                    (RCC->APB2ENR |= (1UL << 0))

/*
 * SPI Clock Enable
 * */

#define SPI1_PCLK_EN()                   (RCC->APB2ENR |= (1UL << 12))
#define SPI2_PCLK_EN()                   (RCC->APB1ENR |= (1UL << 14))
#define SPI3_PCLK_EN()                   (RCC->APB1ENR |= (1UL << 15))

/*
 * SPI Clock Disable
 * */

#define SPI1_PCLK_DIS()                  (RCC->APB2ENR &= ~(1UL << 12))
#define SPI2_PCLK_DIS()                  (RCC->APB1ENR &= ~(1UL << 14))
#define SPI3_PCLK_DIS()                  (RCC->APB1ENR &= ~(1UL << 15))

/*
 * SPIx Register Reset
 * */

#define SPI1_REG_RESET()                do{ (RCC->APB2RSTR |= (1UL << 12)); (RCC->APB2RSTR &= ~(1UL << 12)); } while(0)
#define SPI2_REG_RESET()                do{ (RCC->APB1RSTR |= (1UL << 14)); (RCC->APB1RSTR &= ~(1UL << 14)); } while(0)
#define SPI3_REG_RESET()                do{ (RCC->APB1RSTR |= (1UL << 15)); (RCC->APB1RSTR &= ~(1UL << 15)); } while(0)


#endif /* INC_STM32F103XX_H_ */
