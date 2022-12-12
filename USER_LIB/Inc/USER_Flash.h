/*
 * flash.h
 *
 *  Created on: Dec 6, 2022
 *      Author: HPMC
 */

#ifndef INC_USER_FLASH_H_
#define INC_USER_FLASH_H_
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>
#pragma pack(1)
typedef struct
{
	uint8_t no;
	uint8_t ssid[30];
	uint8_t pass[30];
}wifi_info_t;
#pragma pack()

void USER_FLASH_ErasePage(FLASH_EraseInitTypeDef EraseInitStruct);
void USER_FLASH_Write(uint32_t Address, const uint8_t* data, int dataLength);
char* USER_FLASH_Read_DoubleWord(uint64_t  Address);

void USER_FLASH_Write_IntType(uint32_t Address, uint64_t value);
void USER_FLASH_Write_DoubleType(uint32_t Address,double value);
void USER_FLASH_Write_Array(uint32_t Address, uint8_t* Array, uint16_t Length);
void USER_FLASH_Write_Struct(uint32_t Address, wifi_info_t data);

uint64_t USER_FLASH_Read_IntType(uint32_t address);
double USER_FLASH_Read_double(uint32_t address);
void USER_FLASH_Read_Array(uint32_t address, uint8_t* Array, uint16_t Length);
void USER_FLASH_Read_Struct(uint32_t address, wifi_info_t *data);

#endif /* INC_USER_FLASH_H_ */
