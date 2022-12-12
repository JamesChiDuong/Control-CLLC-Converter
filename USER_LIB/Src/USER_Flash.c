/*
 * flash.c
 *
 *  Created on: Dec 6, 2022
 *      Author: HPMC
 */
#include "USER_Flash.h"
void USER_FLASH_ErasePage(FLASH_EraseInitTypeDef EraseInitStruct)
{
	/* Unlock Mode*/
	HAL_FLASH_Unlock();

	//HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	FLASH_PageErase(EraseInitStruct.NbPages,EraseInitStruct.Banks);

	HAL_FLASH_Lock();

}
void USER_FLASH_Write(uint32_t Address, const uint8_t* data, int dataLength)
{
	//HAL_FLASH_Unlock();
	unsigned int dwLen = 0;

	if((dataLength % 8 ==0) && (dataLength >= 8))
	{
		dwLen = dataLength;
	}
	else if ((dataLength % 8 > 0) && (dataLength > 8))
	{
		dwLen = dataLength + (8 - (dataLength % 8));
	}
	else if (dataLength < 8)
	{
		dwLen = dataLength + (8 - (dataLength % 8));
	}
	uint8_t dataArr[dwLen +1];
	memset((char*) dataArr, 0xFF, dataLength +1);
	memcpy((char*)dataArr, (char*)data, dataLength);

	uint64_t __data = 0;
	for(uint32_t i = 0; i < dwLen ; i +=8)
	{
		__data |= dataArr[i + 7]; __data <<= 8;
		__data |= dataArr[i + 6]; __data <<= 8;
		__data |= dataArr[i + 5]; __data <<= 8;
		__data |= dataArr[i + 4]; __data <<= 8;
		__data |= dataArr[i + 3]; __data <<= 8;
		__data |= dataArr[i + 2]; __data <<= 8;
		__data |= dataArr[i + 1]; __data <<= 8;
		__data |= dataArr[i + 0];
//		HAL_FLASH_Unlock();
//		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,Address, __data);
//		HAL_FLASH_Lock();
		USER_FLASH_Write_IntType((Address+i), 1000);
		__data = 0x0000000000000000;
	}

}
char* USER_FLASH_Read_DoubleWord(uint64_t  Address)
{
	static char result[8 + 1];		/* Store 8 byte data with 1 byte character end*/
	char temp[8] = {0};

	memset(result, 0x00, 8+1);
	memset(temp, 0xFF, 8);

	uint64_t data;
	data = *(__IO uint64_t *)Address;

	for(int i = 0; i < 8 ; i++)
	{
		temp[i] = data;
		data >>= 8;
	}
	memcpy(result, temp, 8);
	return result;
}
















void USER_FLASH_Write_IntType(uint32_t Address, uint64_t value)
{
	HAL_FLASH_Unlock();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, value);

	HAL_FLASH_Lock();
}
//void USER_FLASH_Write_DoubleType(uint32_t Address, double value)
//{
//	uint8_t data[8] = {0};
//	uint64_t var = 0;
//	*(double *)data = value;
//	var = *(uint64_t*)data;
//	HAL_FLASH_Unlock();
//
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, var);
//
//	HAL_FLASH_Lock();
//}
//void USER_FLASH_Write_Array(uint32_t Address, uint8_t* Array, uint16_t Length)
//{
//	HAL_FLASH_Unlock();
//	uint64_t *ptr = (uint64_t*)Array;
//	for (uint32_t i = 0; i< (Length+1)/4 ; i ++)
//	{
//		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (Address + (4*i)), *ptr);
//		ptr++;
//	}
//	HAL_FLASH_Lock();
//}
uint64_t USER_FLASH_Read_IntType(uint32_t address)
{
	return *(__IO uint64_t*)(address);
}
//double USER_FLASH_Read_double(uint32_t address)
//{
//	uint64_t data = *(__IO uint32_t*)(address);
//	return *(__IO double*)(&data);
//}
//void USER_FLASH_Read_Array(uint32_t address, uint8_t* Array, uint16_t Length)
//{
//	uint64_t *ptr = (uint64_t*)Array;
//	for (uint32_t i = 0; i< (Length+1)/4 ; i ++)
//	{
//		*ptr = *(__IO uint64_t*)(address + (4*i));
//		ptr++;
//	}
//}
