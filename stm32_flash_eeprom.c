/*
 * flash_eeprom.c
 *
 *  Created on: Aug 31, 2022
 *      Author: kyle
 */
//=========================================================================
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern int h_printf( const char *format_const, ...);
//===STM32F407 FLASH MACRO ============================================
#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
/*------------------------------------------------------------------------------------------
| STT |  Sector name       | start_address |  end_address  |sector size(KB)  | offset(KB)   |
|-----+--------------------+---------------+---------------+-----------------+--------------|
| 0   |  FLASH_Sector_0    |   08000000    |    08003FFF   |       16        |     0        |
| 1   |  FLASH_Sector_1    |   08004000    |    08007FFF   |       16        |     16       |
| 2   |  FLASH_Sector_2    |   08008000    |    0800BFFF   |       16        |     32       |
| 3   |  FLASH_Sector_3    |   0800C000    |    0800FFFF   |       16        |     48       |
| 4   |  FLASH_Sector_4    |   08010000    |    0801FFFF   |       64        |     64       |
| 5   |  FLASH_Sector_5    |   08020000    |    0803FFFF   |       128       |     128      |
| 6   |  FLASH_Sector_6    |   08040000    |    0805FFFF   |       128       |     256      |
| 7   |  FLASH_Sector_7    |   08060000    |    0807FFFF   |       128       |     384      |
| 8   |  FLASH_Sector_8    |   08080000    |    0809FFFF   |       128       |     512      |
| 9   |  FLASH_Sector_9    |   080A0000    |    080BFFFF   |       128       |     640      |
| 10  |  FLASH_Sector_10   |   080C0000    |    080DFFFF   |       128       |     768      |
| 11  |  FLASH_Sector_11   |   080E0000    |    080FFFFF   |       128       |     896      |
| 12  |  FLASH_Sector_12   |   08100000    |    08103FFF   |       16        |     1024     |
| 13  |  FLASH_Sector_13   |   08104000    |    08107FFF   |       16        |     1040     |
| 14  |  FLASH_Sector_14   |   08108000    |    0810BFFF   |       16        |     1056     |
| 15  |  FLASH_Sector_15   |   0810C000    |    0810FFFF   |       16        |     1072     |
| 16  |  FLASH_Sector_16   |   08110000    |    0811FFFF   |       64        |     1088     |
| 17  |  FLASH_Sector_17   |   08120000    |    0813FFFF   |       128       |     1152     |
| 18  |  FLASH_Sector_18   |   08140000    |    0815FFFF   |       128       |     1280     |
| 19  |  FLASH_Sector_19   |   08160000    |    0817FFFF   |       128       |     1408     |
| 20  |  FLASH_Sector_20   |   08180000    |    0819FFFF   |       128       |     1536     |
| 21  |  FLASH_Sector_21   |   081A0000    |    081BFFFF   |       128       |     1664     |
| 22  |  FLASH_Sector_22   |   081C0000    |    081DFFFF   |       128       |     1792     |
| 23  |  FLASH_Sector_23   |   081E0000    |    081FFFFF   |       128       |     1920     |
-------------------------------------------------------------------------------------------*/
#define FLASH_EEPROM_BASE_ADD	0x08040000
#define FLASH_EEPROM_SIZE		(1024*2)
#define FLASH_EEPROM_SECTOR		FLASH_Sector_6
#define USE_FLASH_BUFFER		1

#define FLASH_PROGRAM_HALF_WORD(add,val16) do{\
		FLASH_ClearFlag(FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_RDERR);\
		FLASH_Unlock();\
		FLASH_ProgramHalfWord((add),(val16));\
		FLASH_Lock();\
}while(0)
#define FLASH_PROGRAM_WORD(add,val32) do{\
		FLASH_ClearFlag(FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_RDERR);\
		FLASH_Unlock();\
		FLASH_ProgramWord((add),(val32));\
		FLASH_Lock();\
}while(0)
#define FLASH_ERASE_SECTOR(add) do{\
		FLASH_ClearFlag(FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_RDERR);\
		FLASH_Unlock();\
		FLASH_EraseSector(FLASH_SECTOR,VoltageRange_3);\
		FLASH_Lock();\
}while(0)
//==============================================================================
#if USE_FLASH_BUFFER
static uint8_t flash_buffer[FLASH_EEPROM_SIZE];
#endif
//=============================================================================
void FLASH_EEPROM_READ(uint32_t offset, uint8_t *data, uint16_t len){
	uint8_t * p8 = (uint8_t*)(FLASH_EEPROM_BASE_ADD+offset);
	if(data == NULL) return ;
	for(uint32_t i=0;i<len && offset < FLASH_EEPROM_SIZE;i++,offset++){
		data[i] = p8[offset];
	}
}
//===============================================================================
void FLASH_EEPROM_WRITE(uint32_t offset, uint8_t *data, uint16_t len){
#if USE_FLASH_BUFFER
	//Read data to buffer
	FLASH_EEPROM_READ(0, flash_buffer, FLASH_EEPROM_SIZE);
	//modify data
	for(uint32_t i= 0;i<len && i+ offset< FLASH_EEPROM_SIZE;i++){
		flash_buffer[i+offset] = data[i];
	}
	//Unlock flash
	FLASH_ClearFlag(FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_RDERR);
	FLASH_Unlock();
	//Erase flash
	FLASH_EraseSector(FLASH_EEPROM_SECTOR, VoltageRange_3);
	//Write flash
	while(len>3){
		uint32_t tmp32 = *(uint32_t*)data;
		FLASH_PROGRAM_WORD(FLASH_EEPROM_BASE_ADD+offset,tmp32);
		offset+=4;
		len-=4; data+=4;
	}
	if(len){
		uint32_t tmp32 = *(uint32_t*)data;
		FLASH_PROGRAM_WORD(FLASH_EEPROM_BASE_ADD+offset,tmp32);
	}
#else
	if(len +offset > FLASH_EEPROM_SIZE) return;
	FLASH_ClearFlag(FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_RDERR);
	FLASH_Unlock();
	//Erase flash
	FLASH_EraseSector(FLASH_EEPROM_SECTOR, VoltageRange_3);
	for(uint16_t i=0; i<len;i+=4){
		uint32_t tmp32 = *data++;
		tmp32 = (tmp32<<8) | *data++;
		tmp32 = (tmp32<<8) | *data++;
		tmp32 = (tmp32<<8) | *data++;
		tmp32 = (tmp32<<8) | *data++;
		FLASH_PROGRAM_WORD(FLASH_EEPROM_BASE_ADD+i,tmp32);
	}
#endif
}

#endif

#if defined(STM32F10X_HD) ||  defined(STM32F10X_CL)
#include <stm32f10x.h>
#include <stm32f10x_flash.h>



#define FLASH_SECTOR_SIZE			1024*2
#define FLASH_EEPROM_BASE_ADD		0x08060000
#define FLASH_EEPROM_SIZE			(1024*2048)
#define FLASH_HEADER				0x1234
#define FLASH_EEPROM_BASE_ADDRESS	( NVIC_VectTab_FLASH + 127*2048)

#define FLASH_UNLOCK()		FLASH_Unlock()
#define FLASH_LOCK()		FLASH_Lock()

#define FLASH_PROGRAM_HALF_WORD(add,val16) do{\
		FLASH_ClearFlag(FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);\
		FLASH_ProgramHalfWord((add),(val16));\
}while(0)

#define FLASH_ERASE_SECTOR(add) do{\
		FLASH_ClearFlag(FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);\
		FLASH_ErasePage(add);\
}while(0)


//==============================================================================
void FLASH_EEPROM_WRITE(uint32_t base, uint8_t *data, uint16_t len){
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
void FLASH_EEPROM_READ(uint32_t base, uint8_t *data, uint16_t len){
	uint32_t offset = 0 ;
	uint8_t * p8 = (uint8_t*)base;
	if(data == NULL) return ;
	while(len--) *data++ = p8[offset++];
}
//=============================================================================
#endif

