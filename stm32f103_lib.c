#include "stm32_lib.h"
#if USE_STM32F103_GENERAL_LIB
#include "stm32f10x_flash.h"
#include <string.h>

//=========================================================================
void GPIO_Mode_output_PushPull(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;
	GPIO_ENABLE_CLOCK(GPIO);
	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_SetMode_output_PushPull_using_pin_mask(GPIO_TypeDef * GPIO, uint32_t pin){
	GPIO_InitTypeDef           GPIO_InitStructure;
	GPIO_ENABLE_CLOCK(GPIO);
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_input_PullUp(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;

	GPIO_ENABLE_CLOCK(GPIO);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_input_PullDown(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;

	GPIO_ENABLE_CLOCK(GPIO);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_input_Floating(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;

	GPIO_ENABLE_CLOCK(GPIO);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_Analog(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;
	GPIO_ENABLE_CLOCK(GPIO);
	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_Alternate(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;
	GPIO_ENABLE_CLOCK(GPIO);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_Alternate_using_pin_mask(GPIO_TypeDef * GPIO, uint32_t pin){
	GPIO_InitTypeDef           GPIO_InitStructure;
	GPIO_ENABLE_CLOCK(GPIO);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_AlternatePullUp(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;

	GPIO_ENABLE_CLOCK(GPIO);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//=========================================================================
void GPIO_Mode_AlternatePullDown(GPIO_TypeDef * GPIO, uint32_t bit){
	GPIO_InitTypeDef           GPIO_InitStructure;

	GPIO_ENABLE_CLOCK(GPIO);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = 1<<(bit);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);
}
//===============================================================================
void USART_SetBaudrate(USART_TypeDef * USARTx, uint32_t baud){
	uint32_t apbclock = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	/*---------------------------- USART BRR Configuration -----------------------*/
	/* Configure the USART Baud Rate */
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	if (USARTx == USART1)
	{
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	}
	else
	{
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}
	USARTx->BRR = (apbclock + (baud>>1))/baud;
}
//==============================================================================
void FLASH_WRITE(uint32_t base, uint8_t *data, uint16_t len){
	FLASH_Unlock();
	FLASH_ErasePage(base);
	while(len>4){
		uint32_t tmp32 = *(uint32_t*)data;
		len-=4; data+=4;
		FLASH_ProgramWord(base,tmp32);base+=4;
	}
	while(len >=2){
		uint16_t tmp16 = *(uint16_t*)data;
		len-=2; data+=2;
		FLASH_ProgramHalfWord(base,tmp16);base+=2;
	}
	if(len){
		uint16_t tmp16 = *data;
		FLASH_ProgramHalfWord(base,tmp16);
	}
	FLASH_Lock();
}
//=============================================================================
void FLASH_READ(uint32_t base, uint8_t *data, uint16_t len){
	uint32_t offset = 0 ;
	uint8_t * p8 = (uint8_t*)base;
	if(data == NULL) return ;
	while(len--) *data++ = p8[offset++];
}
//=============================================================================
void TIMER_ClockCmd( TIM_TypeDef * TIMx, FunctionalState state){
	switch((uint32_t)TIMx){
	case (uint32_t)TIM1:RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, state); break;
	case (uint32_t)TIM2:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, state); break;
	case (uint32_t)TIM3:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, state); break;
	case (uint32_t)TIM4:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, state); break;
	case (uint32_t)TIM5:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, state); break;
	case (uint32_t)TIM6:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, state); break;
	case (uint32_t)TIM7:RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, state); break;
	case (uint32_t)TIM8:RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, state); break;
	default: break;
	}
}
//===============================================================================
void TIMER_StartIRQ(TIM_TypeDef * TIMx, uint32_t tick_rate_hz){
	if(TIMx != NULL && tick_rate_hz >0){
		TIMx->CCR1 = 0;
		uint32_t arr = SystemCoreClock/ tick_rate_hz;
		uint32_t psc = 1;
		while(arr >= 65536){
			psc = psc*10;
			arr = arr/10;
		}
		TIMx->PSC = psc-1;
		TIMx->ARR = arr-1;
		TIMx->RCR = 0;
		TIMx->DIER = TIM_DIER_UIE;
		TIMx->CR1 |= TIM_CR1_CEN;
	}
}
//===============================================================================
void USART_ClockCmd( USART_TypeDef * USARTx, FunctionalState state){
	switch((uint32_t)USARTx){
	case (uint32_t)USART1:RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, state); break;
	case (uint32_t)USART2:RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, state); break;
	case (uint32_t)USART3:RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, state); break;
	case (uint32_t)UART4 :RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 , state); break;
	case (uint32_t)UART5 :RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5 , state); break;
	default: break;
	}
}
//===============================================================================
void DMA_ClockCmd( DMA_Stream_TypeDef * stream, FunctionalState state){
	switch((uint32_t)stream){
	case (uint32_t)DMA1_Stream1:
	case (uint32_t)DMA1_Stream2:
	case (uint32_t)DMA1_Stream3:
	case (uint32_t)DMA1_Stream4:
	case (uint32_t)DMA1_Stream5:
	case (uint32_t)DMA1_Stream6:
	case (uint32_t)DMA1_Stream7:
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,state);
	break;
	case (uint32_t)DMA2_Stream1:
	case (uint32_t)DMA2_Stream2:
	case (uint32_t)DMA2_Stream3:
	case (uint32_t)DMA2_Stream4:
	case (uint32_t)DMA2_Stream5:
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,state);
	break;
	}
}
//***************************************
void delay_us(uint32_t time){
	int volatile  i;
	while (time--){
		for(i=0;i<10;i++);
	}
}
#endif
