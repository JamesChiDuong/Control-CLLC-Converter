/*
 * USER_Filter.h
 *
 *  Created on: Nov 11, 2022
 *      Author: HPMC
 */

#ifndef INC_USER_FILTER_H_
#define INC_USER_FILTER_H_

#include <math.h>
#include <stdint.h>
/**
  * @brief  The Function of LowPass Filter
  * @note   Parameters of this function
  *          - Signal: The raw of Signal
  *          - CutoffFrequency: The frequency need to cut off
  *          - SampleRate: The sampling frequency
  * return: Value of output Signal
  */
float __USER_FILTER_LowPassFilter(float Signal, float CutOffFreq, float SampleRate, uint8_t numberFilter);


#endif /* INC_USER_FILTER_H_ */
