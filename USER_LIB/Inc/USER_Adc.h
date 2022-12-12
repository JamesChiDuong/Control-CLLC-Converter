/*
 * USER_Adc.h
 *
 *  Created on: Oct 29, 2022
 *      Author: PC
 */

#ifndef INC_USER_ADC_H_
#define INC_USER_ADC_H_
#include <USER_CALLBACK_FUNCTION.h>
#include <USER_Filter.h>
typedef struct ADCFilter
{
	float cutoffFreq; //Hz
	float sampleTime; //Hz
	float output;
}COMPONENT_ADCFilter;

/**** This function for getting value from DMA ****/
/**
  * @brief  Get value from DMA
  * @note   Parameters of this function
  *          - para1: to choose the channel to get value
  *          - para2: The information of ADC
  * return value of output signal
  */
float USER_ADC_GetADCFilterValue(uint8_t channelADC);

/**
  * @brief  The Function to handler String From USART to put into the ADC
  * @note   Parameters of this function
  *          - Buffer: Which is stored Data
  */
COMPONENT_ADCFilter USER_ADC_handleString(char* Buffer);

/**
  * @brief  The Function to config the information of filter
  * @note   Parameters of this function
  *          - Buffer: Which is stored Data
  */
void USER_ADC_ConfigFilter(COMPONENT_ADCFilter FilterSignalConfig);

/**
  * @brief  The Function will return information of Filter
  * @note   Parameters of this function
  *          - none
  * return  information of Filter
  */
COMPONENT_ADCFilter USER_ADC_GetInforFilter(void);
#endif /* INC_USER_ADC_H_ */
