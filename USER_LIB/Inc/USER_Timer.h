#ifndef TIMER_FUNCTION_USER
#define TIMER_FUNCTION_USER

#include <USER_CALLBACK_FUNCTION.h>
#include "stm32g4xx_hal.h"
typedef struct COMPONENT_TIMER
{
	float Duty;
	uint16_t NumberOfTimer;
	uint32_t Channel;
	uint32_t frequency;
}COMPONENT_TIMER;
/**
  * @brief  The Function to choose the property of PWM such as, Dutycycle, The number and channel of Timer
  * @note   Parameters of this function
  *          - DutyCycle: To choose the expected dutycyle
  *          - htim: The number of Timer
  *          - TimerChannel: The number channel of timer
  */
void USER_TIMER_setValueOfPWM(COMPONENT_TIMER TimerVariable);/************* Funtion to set PWM*/

/**
  * @brief  The Function to handler String From USART to put into the timer variable
  * @note   Parameters of this function
  *          - Buffer: Which is stored Data
  */
void USER_TIMER_setFrequency(uint32_t ExpectedFrequency, uint16_t NumberOfTimer,uint16_t TimerChannel);

COMPONENT_TIMER USER_TIMER_handleString(char* Buffer);
/**
  * @brief  To get value from ARR Register
  * @note   Parameters of this function
  *          - None
  */
uint32_t USER_TIMER_getValueARR(void);
/**
  * @brief  To get value from CRR Register
  * @note   Parameters of this function
  *          - None
  */
uint32_t USER_TIMER_getValueCCR(void);
/**
  * @brief  Convert ADC value to Dutycycle. Voltage 0 - 3.3V conver 0->100%
  * @note   Parameters of this function
  *          - ADC value
  */
float USER_TIMER_ConvertADCValueToDutyCycle(float ADCValue);
/**
  * @brief  To divied value CCR into 2 it will be served for trigger ADC at the middle square form
  * @note   Parameters of this function
  *          - Timer
  */
void USER_TIMER_DividedIntoTwoCCR(TIM_HandleTypeDef timer);
#endif
