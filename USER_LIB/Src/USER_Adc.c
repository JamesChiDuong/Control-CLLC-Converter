/*
 * USER_Adc.c
 *
 *  Created on: Oct 29, 2022
 *      Author: PC
 */
#include <USER_Adc.h>
extern uint32_t ADCVar[3]; /********************************************* ADC variable for DMA*/

COMPONENT_ADCFilter FilterSignal ={50000,100000,0};
/*Static funtion*/

void USER_ADC_ConfigFilter(COMPONENT_ADCFilter FilterSignalConfig)
{
	FilterSignal.cutoffFreq = FilterSignalConfig.cutoffFreq;
	FilterSignal.sampleTime = FilterSignalConfig.sampleTime;
}
float USER_ADC_GetADCFilterValue(uint8_t channelADC)
{
	float rawSignal = 0;
	switch(channelADC)
	{
		case 1:
			rawSignal = (float)ADCVar[0];
			break;
		case 5:
			rawSignal = (float)ADCVar[1];
			break;
		case 12:
			rawSignal = (float)ADCVar[2];
			break;
	}
	FilterSignal.output = __USER_FILTER_LowPassFilter(rawSignal, FilterSignal.cutoffFreq, FilterSignal.sampleTime,channelADC);
	return FilterSignal.output;
}
/*Syntax buffer ADC-100-100000(ADC-cutoff-samplingtime) */
COMPONENT_ADCFilter USER_ADC_handleString(char* Buffer)
{
	COMPONENT_ADCFilter var;
	char* ptr = NULL;
	ptr = strtok(Buffer,"-");
	ptr = strtok(NULL,"-");
	var.cutoffFreq = atoff(ptr);
	ptr = strtok(NULL,"-");
	var.sampleTime = atoff(ptr);

	return var;
}

COMPONENT_ADCFilter USER_ADC_GetInforFilter(void)
{
	return FilterSignal;
}
