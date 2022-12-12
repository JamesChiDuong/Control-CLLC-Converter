/*
 * USER_Filter.c
 *
 *  Created on: Nov 11, 2022
 *      Author: HPMC
 */

#include <USER_Filter.h>

float __USER_FILTER_LowPassFilter(float Signal, float CutOffFreq,
		float SampleRate, uint8_t numberFilter)
{
	float RC = 0.0;
	float dt = 0.0;
	float alpha = 0.0;
	float y = 0.0;
	float w0 = 2*M_PI*CutOffFreq;

	static float ylast1 = 0.0; /********** To call ylast continue****/
	static float ylast2 = 0.0;
	static float ylast3 = 0.0;
	RC = 1.0/w0;
	dt = 1.0/SampleRate;
	alpha = dt/(RC+dt);

	switch(numberFilter)
	{
		case 1:
			y = ylast1 + alpha *(Signal - ylast1);
			ylast1 = y;
			break;
		case 5:
			y = ylast2 + alpha *(Signal - ylast2);
			ylast2 = y;
			break;
		case 12:
			y = ylast3 + alpha *(Signal - ylast3);
			ylast3 = y;
			break;
	}
	return y;
}
