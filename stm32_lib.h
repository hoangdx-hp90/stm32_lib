/*
 * stm32_helper.h
 *
 *  Created on: Aug 21, 2022
 *      Author: kyle
 */

#ifndef STM32_HELPER_H_
#define STM32_HELPER_H_
#include "stm32_lib_config.h"
#include "stdint.h"

void update_irq_handler_function(int32_t IRQn, void fcn(void));
void system_tick_init();







//====================================================
// MAP FOR STM32H743 device
#ifdef STM32H743xx
#include "stm32h743xx.h"


//Predefine MACRO
#define IWDG_ReloadCounter() IWDG1->KR= 0x0000AAAAU
#endif
//====================================================
#if 0




#ifdef STM32F40_41xxx
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

typedef struct{
	USART_TypeDef 		* USARTx;
	DMA_Stream_TypeDef *tx_stream;
	uint32_t			tx_channel;
	uint32_t volatile * tx_clear_reg;
	uint32_t			tx_clear_msk;
	DMA_Stream_TypeDef *rx_stream;
	uint32_t			rx_channel;
	uint32_t volatile *	rx_clear_reg;
	uint32_t			rx_clear_msk;
	int					IRQn;
	uint32_t			AFIO_type;
}uart_hw_info_typedef;


#define GPIO_ENABLE_CLOCK(GPIO) 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA <<(((uint32_t)(GPIO) -(uint32_t)GPIOA )>>10), ENABLE);
#define GPIO_SET_BIT(GPIO,bit)   	(GPIO)->BSRRL = 1<<(bit)
#define GPIO_CLEAR_BIT(GPIO,bit)    (GPIO)->BSRRH = 1<<(bit)
#define GPIO_OUT_BIT(GPIO,bit,val) \
		do {\
			if((val)) 	(GPIO)->BSRRL = 1<<(bit);\
			else 		(GPIO)->BSRRH = 1<<(bit);\
		}while(0);
#define GPIO_READ_BIT(GPIO,bit)	(((GPIO)->IDR&(1<<(bit)))?1:0)
#endif

#if defined (STM32F10X_HD) ||  defined (STM32F10X_CL)
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#define GPIO_ENABLE_CLOCK(GPIO) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA <<(((uint32_t)(GPIO) -(uint32_t)GPIOA )/((uint32_t)(GPIOB) -(uint32_t)GPIOA)), ENABLE);
#define GPIO_SET_BIT(GPIO,bit)   	((GPIO_TypeDef *)GPIO)->BSRR = 1<<(bit)
#define GPIO_CLEAR_BIT(GPIO,bit)   	((GPIO_TypeDef *)GPIO)->BRR = 1<<(bit)
#define GPIO_CLEAR_PIN(GPIO,pin)	((GPIO_TypeDef *)GPIO)->BRR = (pin)
#define GPIO_SET_PIN(GPIO,bit)   	((GPIO_TypeDef *)GPIO)->BSRR = (pin)
#define GPIO_CLR_BIT				GPIO_CLEAR_BIT
//Data type mapping

#if defined( STM32F10X_HD) ||  defined(STM32F10X_CL)
typedef  struct {
	__IO uint32_t CR;
	__IO uint32_t NDTR;
	__IO uint32_t CPAR;
	__IO uint32_t CM0AR;
}DMA_Stream_TypeDef;

#define DMA1_Stream1       ((DMA_Stream_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Stream2       ((DMA_Stream_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Stream3       ((DMA_Stream_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Stream4       ((DMA_Stream_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Stream5       ((DMA_Stream_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Stream6       ((DMA_Stream_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Stream7       ((DMA_Stream_TypeDef *) DMA1_Channel7_BASE)
#define DMA2_Stream1       ((DMA_Stream_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Stream2       ((DMA_Stream_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Stream3       ((DMA_Stream_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Stream4       ((DMA_Stream_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Stream5       ((DMA_Stream_TypeDef *) DMA2_Channel5_BASE)
#endif
//====================================================
#define GPIO_OUT_BIT(GPIO,bit,val) \
		do {\
			if((val)) 	(GPIO)->BSRR = 1<<(bit);\
			else 		(GPIO)->BRR  = 1<<(bit);\
		}while(0);
#define GPIO_READ_BIT(GPIO,bit)	(((GPIO)->IDR&(1<<(bit)))?1:0)
#endif
//====================================================
#define GPIO_WRITE_BIT 	GPIO_OUT_BIT



void GPIO_Mode_output_PushPull(GPIO_TypeDef * GPIO, uint32_t bit);

void GPIO_Mode_input_PullUp(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_input_PullDown(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_input_Floating(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_Analog(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_Alternate(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_AlternatePullUp(GPIO_TypeDef * GPIO, uint32_t bit);
void GPIO_Mode_AlternatePullDown(GPIO_TypeDef * GPIO, uint32_t bit);

void GPIO_SetMode_output_PushPull_using_pin_mask(GPIO_TypeDef * GPIO, uint32_t pin);
void GPIO_Mode_Alternate_using_pin_mask(GPIO_TypeDef * GPIO, uint32_t pin);

void USART_SetBaudrate(USART_TypeDef * USARTx, uint32_t baud);
#ifdef STM32F40_41xxx
const uart_hw_info_typedef* USART_GetHardwareInfo(USART_TypeDef * USARTx);
#endif
void USART_ClockCmd( USART_TypeDef * USARTx, FunctionalState state);
void TIMER_ClockCmd( TIM_TypeDef * TIMx, FunctionalState state);
void TIMER_StartIRQ(TIM_TypeDef * TIMx, uint32_t tick_rate_hz);

#ifdef STM32F40_41xxx
void DMA_ClockCmd( DMA_Stream_TypeDef * stream, FunctionalState state);
#endif

#endif





void system_timer_init(void* base);
void delay_ms(uint32_t time);
void delay_us(uint32_t time);
#endif /* STM32_HELPER_H_ */
