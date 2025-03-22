/*
 * stm32_lib_config.h
 *
 *  Created on: Oct 17, 2024
 *      Author: kyle
 */

#ifndef STM32_LIB_STM32_LIB_CONFIG_H_
#define STM32_LIB_STM32_LIB_CONFIG_H_

//#define USE_STM32_LOOP					1
//#define USE_STM32F103_GENERAL_LIB		1
//#define USE_STM32F4XX_GENERAL_LIB		1



//========================================================
#ifdef STM32F40_41xxx
#define USE_STM32F4XX_GENERAL_LIB		1
#endif
//========================================================
#ifdef STM32H743xx
#define USE_STM32H743_GENERAL_LIB		1
#endif

#endif /* STM32_LIB_STM32_LIB_CONFIG_H_ */
