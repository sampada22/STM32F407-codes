/*
 * pwm.c
 * 
 * Created : 9/25/2018
 *  Author : NIS_ANE
 *   email : 073bex421.nischal@pcampus.edu.np
 */
 
 #include "pwm.h"
 
 /* SET DUTYCYCLE function */
void set_DutyCycle (TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle)
{
		uint16_t mapped_value;
		mapped_value = (time_period(MOTOR_DRIVER_FREQUENCY)*dutyCycle)/65535;
		__HAL_TIM_SET_COMPARE(htim, Channel, mapped_value);
}
