/*
 * stm32_no_os.h
 *
 *  Created on: Apr 24, 2024
 *      Author: kyle
 */

#ifndef STM32_LIB_STM32_NO_OS_H_
#define STM32_LIB_STM32_NO_OS_H_
#include "stm32_lib.h"

#if USE_STM32_LOOP
#include "stdint.h"
#include "string.h"


void system_timer_init(void* base);
uint32_t system_get_time_msec();
uint32_t system_get_time_sec();
void system_delay_ms(uint32_t msec);
void system_delay_us(uint32_t usec);
#endif

#endif /* STM32_LIB_STM32_NO_OS_H_ */
