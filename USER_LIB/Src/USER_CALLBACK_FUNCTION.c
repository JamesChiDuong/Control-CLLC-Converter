/*
 * USER_CALLBACK_FUNTION.c
 *
 *  Created on: Oct 18, 2022
 *      Author: PC
 */
#include <USER_CALLBACK_FUNCTION.h>
/*Extern Variable*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart2;
/*Variable */
char Rx_data[2] ={0};/******************************************************** Rx Data*/
char Rx_Buffer[20] = {0};/**************************************************** Rx Buffer*/
uint8_t count = 0;/*********************************************************** Count Variable to count in to RxBuffer*/
uint8_t CheckFlagADC = 0;/**************************************************** Flag checking when ADC jump into CallBack Function*/
uint8_t CheckFlagUSART = 0;/*************************************************** Flag checking when UART jump into CallBack funtion*/
uint16_t Voltage_Variable = 0;/*********************************************** Voltage Variable*/
uint32_t ADCVar[3];

void USER_CALLBACK_init(void)
{
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	 // HAL_TIM_Base_Start_IT(&htim2);
	//  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);/******************************* To disable ADC*/
	  /*Start Interrupt Function*/
	  /****Uart Transfer*****/
	  HAL_UART_Receive_IT(&huart2,(uint8_t*)Rx_data, 1);
	  /**** ADC Start 1********/
	  //HAL_ADC_Start_IT(&hadc1);
	  /**** ADC 4 Start with DMA*****/
	  HAL_ADC_Start_DMA(&hadc3,ADCVar,3);
	  //HAL_ADC_Start(&hadc3);
	  // Use HAL_ADC_Start when want to start mesuare ADC
	  // HAL_ADC_Stop to stop mesuare
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		Rx_Buffer[count++] = Rx_data[0];
		if((Rx_data[0]) == '\r')
		{
			count = 0;
			CheckFlagUSART = 1;
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t*)Rx_data,1);
	}
}
/********Conversion complete callback in non-blocking mode***/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{
		CheckFlagADC = 1;
	}
	if(hadc->Instance == hadc3.Instance)
	{
		CheckFlagADC = 1;
	}
}

void USER_CALLBACK_DeInit(void)
{
	memset(Rx_Buffer,'\0',strlen(Rx_Buffer));
	memset(Rx_data,'\0',strlen(Rx_data));
	CheckFlagUSART = 0;
	CheckFlagADC = 0;
}
