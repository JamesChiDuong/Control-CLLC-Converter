/*
 * Timer_USER.c
 *
 *  Created on: Oct 18, 2022
 *      Author: PC
 */
#include <USER_Timer.h>
#include <assert.h>
/*Declare Variable */
uint32_t AHBFrequency = 150000000;
uint32_t ValueOfARR = 0;
uint32_t ValueOfCCR = 0;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/****Static Prototype *****/
static uint16_t __USER_TIMER_convertU16(int DutyCycle);

/****Define function********************/
static uint16_t __USER_TIMER_convertU16(int DutyCycle)
{
	return (uint16_t)DutyCycle;
}


void USER_TIMER_setValueOfPWM(COMPONENT_TIMER TimerVariable)
{
	TIM_HandleTypeDef htim;
	switch(TimerVariable.NumberOfTimer)
	{
		case 2:
			htim = htim2;
			break;
		case 3:
			htim = htim3;
			break;
	}
	switch (TimerVariable.Channel)
	{
		case 1: /* Channel to trigger ADC */
			TimerVariable.Channel = TIM_CHANNEL_1;
			break;
		case 2: /* Channel to PWM */
			TimerVariable.Channel = TIM_CHANNEL_2;
			break;
		case 3:
			TimerVariable.Channel = TIM_CHANNEL_3;
			break;
	}
	//assert((TimerVariable.frequency >=100000) || TimerVariable.frequency == 0);
	switch (TimerVariable.frequency)
	{
		case 0:
			ValueOfARR = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim);
			break;
		default:
			htim.Instance->ARR = ((uint32_t)((AHBFrequency)/(TimerVariable.frequency*(htim.Init.Prescaler+1))) -1);
			ValueOfARR = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim);
			break;
	}
	ValueOfCCR = (((uint32_t)TimerVariable.Duty*ValueOfARR)/100);
	__HAL_TIM_SET_COMPARE(&htim,TimerVariable.Channel,ValueOfCCR);
}
/*Syntax buffer TIMER-30-20000-2-2(TIMER-dutycycle-tanso-numberoftimer-channel) */
COMPONENT_TIMER USER_TIMER_handleString(char* Buffer)
{
	char* ptr = NULL;
	COMPONENT_TIMER var = {0.0,0,0,0};


	ptr = strtok(Buffer,"-");
	ptr = strtok(NULL,"-");
	var.Duty = __USER_TIMER_convertU16(atoi(ptr));
	ptr = strtok(NULL,"-");
	var.frequency = (uint32_t)atoi(ptr);
	ptr = strtok(NULL,"-");
	var.NumberOfTimer = __USER_TIMER_convertU16(atoi(ptr));
	ptr = strtok(NULL,"-");
	var.Channel = __USER_TIMER_convertU16(atoi(ptr));
	return var;
}
uint32_t USER_TIMER_getValueARR(void)
{
	return ValueOfARR;
}
uint32_t USER_TIMER_getValueCCR(void)
{
	return ValueOfCCR;
}

/**
  * @brief  Convert ADC value to Dutycycle. Voltage 0 - 3.3V conver 0->100%
  * @note   Parameters of this function
  *          - ADC value
  */
float USER_TIMER_ConvertADCValueToDutyCycle(float ADCValue)
{
	float tempValue = (ADCValue/1000);
	float DutyCycle = 0;

	DutyCycle = (tempValue*100)/3.3;
	return DutyCycle;
}

void USER_TIMER_DividedIntoTwoCCR(TIM_HandleTypeDef timer)
{
	timer.Instance->CCR1 = (timer.Instance->CCR2)/2;
}
