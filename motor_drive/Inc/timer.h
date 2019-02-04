/*
 * timer.h
 * 
 * Created : 9/24/2018
 *  Author : NIS_ANE
 *   email : 073bex421.nischal@pcampus.edu.np
 */
 
#ifndef _TIMER_H_
#define _TIMER_H_
 
#include "main.h"
#include "stm32f4xx_hal.h"
#include "defines.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;


void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
uint16_t time_period (uint16_t PWM_frequency);
 
#endif // _TIMER_H_
