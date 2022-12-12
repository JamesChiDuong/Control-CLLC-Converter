#ifndef CALLBACK_FUNCTION_USER
#define CALLBACK_FUNCTION_USER

#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
/**
  * @brief  Funtion Init
  * @note   Parameters of this function
  *          - none
  */
void USER_CALLBACK_init(void);
/**
  * @brief  Function call back, when having event interrupt from UART
  * @note   Parameters of this function
  *          - huart: The type of UART
  */
void USER_CALLBACK_clearRxBuffer(void);
/**
  * @brief  Function to clear RxData
  * @note   Parameters of this function
  *          - none
  */
void USER_CALLBACK_clearRxData(void);

#endif
