#include "stm32_lib.h"

#if USE_STM32_LOOP
#include "stdint.h"
#include "string.h"
#include "stm32f10x.h"
#include "misc.h"


#if USE_STM32_LOOP
#define MAX_TICK_CALLBACK_FCN					16
#define STDOUT_BUF_SIZE							(1024)

static void (*tick_callback_list[MAX_TICK_CALLBACK_FCN])(void) = {NULL};
static uint32_t volatile time_msec=0,time_msec_sub=0,time_sec=0;
static uint8_t volatile init_state = 0;
//========================================================================
#define STDOUT_BUF_SIZE_MSK 	(STDOUT_BUF_SIZE-1)
#if (STDOUT_BUF_SIZE & STDOUT_BUF_SIZE_MSK)
#error "STDOUT_BUF_SIZE must bew 2^N"
#endif
//========================================================================
static uint32_t swo_is_ready(){
	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)  &&      /* Trace enabled */
			(ITM->TCR & ITM_TCR_ITMENA_Msk)         	&&      /* ITM enabled */
			(ITM->TER & (1ul << 0)        )          	)     /* ITM Port #0 enabled */
	{
		if (ITM->PORT[0].u32 == 0) return 0;
		return 1;
	}
	return 0;
}
static uint8_t swo_buf[STDOUT_BUF_SIZE];
static uint32_t volatile swo_wr_add=0,swo_rd_add=0,swo_lock=0;
//========================================================================
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
	swo_lock = 1;
	if(swo_wr_add >=STDOUT_BUF_SIZE ) swo_wr_add = 0;
	uint32_t wr_next = swo_wr_add+1;
	if(wr_next >= STDOUT_BUF_SIZE) wr_next = 0;
	if(wr_next != swo_rd_add ){
		swo_buf[swo_wr_add] = ch;
		swo_wr_add = wr_next;
	}
	swo_lock =0;
	return ch;
}
//========================================================================
void  outbyte(uint8_t character){
	__io_putchar(character);
}
//========================================================================
static void system_process(){

}
//========================================================================
static void system_timer_handler(void){
	time_msec ++;
	time_msec_sub++;
	if(time_msec_sub >=1000){
		time_msec_sub =0;
		time_sec ++;
	}
	for(int i=0;i< MAX_TICK_CALLBACK_FCN;i++){
		if(tick_callback_list[i] != NULL) tick_callback_list[i]();
	}
	//===SWO flush
	if(swo_lock ==0){
		if(swo_rd_add != swo_wr_add){
			if(swo_is_ready()){
				ITM->PORT[0].u8 = (uint8_t) swo_buf[swo_rd_add];
				swo_rd_add = (swo_rd_add+1)&STDOUT_BUF_SIZE_MSK;
			}
		}
	}
}
//========================================================================
void system_timer_init(void* base){
	if(init_state) return;//Only init one time
	uint32_t is_normal_timer = 0;

	if((uint32_t)base == SysTick_BASE){
		update_irq_handler_function(SysTick_IRQn,system_timer_handler);
		SysTick_Config(SystemCoreClock/1000);
		NVIC_SetPriority(SysTick_IRQn, 15);
	}
	else if((uint32_t)base == TIM1_BASE){
		TIMER_ClockCmd(TIM1,ENABLE);
		update_irq_handler_function(TIM1_UP_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM1_UP_IRQn,15);
		NVIC_EnableIRQ(TIM1_UP_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM2_BASE){
		TIMER_ClockCmd(TIM2,ENABLE);
		update_irq_handler_function(TIM2_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM2_IRQn,15);
		NVIC_EnableIRQ(TIM2_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM3_BASE){
		TIMER_ClockCmd(TIM3,ENABLE);
		update_irq_handler_function(TIM3_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM3_IRQn,15);
		NVIC_EnableIRQ(TIM3_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM4_BASE){
		TIMER_ClockCmd(TIM4,ENABLE);
		update_irq_handler_function(TIM4_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM4_IRQn,15);
		NVIC_EnableIRQ(TIM4_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM5_BASE){
		TIMER_ClockCmd(TIM5,ENABLE);
		update_irq_handler_function(TIM5_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM5_IRQn,15);
		NVIC_EnableIRQ(TIM5_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM6_BASE){
		TIMER_ClockCmd(TIM6,ENABLE);
		update_irq_handler_function(TIM6_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM6_IRQn,15);
		NVIC_EnableIRQ(TIM6_IRQn);
		is_normal_timer = 1;
	}
	else if((uint32_t)base == TIM7_BASE){
		TIMER_ClockCmd(TIM7,ENABLE);
		update_irq_handler_function(TIM7_IRQn, system_timer_handler);
		NVIC_SetPriority(TIM7_IRQn,15);
		NVIC_EnableIRQ(TIM7_IRQn);
		is_normal_timer = 1;
	}
	if(is_normal_timer){
		TIMER_StartIRQ( (TIM_TypeDef *)base,1000);
	}
	init_state =1;
}
//========================================================================
uint32_t system_get_time_msec() {
	return time_msec;
}
//========================================================================
uint32_t system_get_time_sec() {
	return time_sec;
}
//========================================================================
void system_delay_ms(uint32_t msec){
	if(init_state){
		uint32_t start_time = time_msec;
		while((uint32_t)(time_msec - start_time ) < msec);
	}
	else{
		int volatile i;
		while (msec--){
			for(i=0;i<10000;i++);
		}
	}
}
//========================================================================
void system_delay_us(uint32_t usec){
	uint32_t msec = usec/1000;
	system_delay_ms(msec);
	usec-= msec*1000;
	while (usec--){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}

#endif
#endif
