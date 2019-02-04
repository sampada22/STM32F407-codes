/*
 * pwm.h
 * 
 * Created : 9/25/2018
 *  Author : NIS_ANE
 *   email : 073bex421.nischal@pcampus.edu.np
 */
 
#ifndef _PWM_H_
#define _PWM_H_
 
#include "main.h"
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "timer.h"

void set_DutyCycle (TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle);

 
#endif // _ADC_H_
