/*
 * soft_reset_vector_table.c
 *
 *  Created on: Aug 4, 2022
 *      Author: kyle
 */
#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#endif

#if defined(STM32F10X_HD) ||  defined(STM32F10X_CL)
#include "stm32f10x.h"
#endif
#include <stdint.h>

static uint32_t volatile user_irq_vector_table[0x200>>2] __attribute__((aligned(0x200)));
static uint32_t volatile init_status = 0;
//=========================================================================================
void copy_irq_vector_to_ram(){
	uint32_t *p32 = (uint32_t*)SCB->VTOR;
	for(uint32_t i=0;i< (sizeof(user_irq_vector_table)>>2);i++){
		user_irq_vector_table[i] = p32[i];
	}
	NVIC_SetVectorTable(NVIC_VectTab_RAM, (uint32_t)user_irq_vector_table - SRAM_BASE);
	init_status = sizeof(user_irq_vector_table)>>2;
}
//=========================================================================================
void update_irq_handler_function(int32_t IRQn, void fcn(void)){
	if(init_status ==0) copy_irq_vector_to_ram();
	if((IRQn >=-14)  &&  IRQn < (((int32_t)sizeof(user_irq_vector_table)>>2)-16)){
		user_irq_vector_table[IRQn+16] = (uint32_t)fcn;
	}
}
