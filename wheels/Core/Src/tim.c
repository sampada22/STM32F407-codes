/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

#include "defines.h"

uint16_t time_period(uint16_t PWM_frequency)
{
        if (PWM_frequency < 2000) { //MIN PWM_FREQUENCY = 1281.738 for time period to be 16 bit
                _Error_Handler(__FILE__, __LINE__);
        }
        return ((PWM_TIMER_FREQUENCY / 2) / PWM_frequency) - 1; // In Center aligned mode period doubles hence we divide by 2
}

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
        TIM_Encoder_InitTypeDef sConfig;
        TIM_MasterConfigTypeDef sMasterConfig;

        htim1.Instance = TIM1;
        htim1.Init.Prescaler = 0;
        htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim1.Init.Period = 0;
        htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim1.Init.RepetitionCounter = 0;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
        TIM_Encoder_InitTypeDef sConfig;
        TIM_MasterConfigTypeDef sMasterConfig;

        htim3.Instance = TIM3;
        htim3.Init.Prescaler = 0;
        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim3.Init.Period = 0;
        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }
}
/* TIM4 init function */
void MX_TIM4_Init(void)
{
        TIM_Encoder_InitTypeDef sConfig;
        TIM_MasterConfigTypeDef sMasterConfig;

        htim4.Instance = TIM4;
        htim4.Init.Prescaler = 0;
        htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim4.Init.Period = 0;
        htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }
}
/* TIM5 init function */
void MX_TIM5_Init(void)
{
        TIM_Encoder_InitTypeDef sConfig;
        TIM_MasterConfigTypeDef sMasterConfig;

        htim5.Instance = TIM5;
        htim5.Init.Prescaler = 0;
        htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim5.Init.Period = 0;
        htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }
}
/* TIM8 init function */
void MX_TIM8_Init(void)
{
        TIM_MasterConfigTypeDef sMasterConfig;
        TIM_OC_InitTypeDef sConfigOC;
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

        htim8.Instance = TIM8;
        htim8.Init.Prescaler = 0;
        htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
        htim8.Init.Period = time_period(MOTOR_DRIVER_FREQUENCY);
        htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim8.Init.RepetitionCounter = 0;
        if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 0;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
        sBreakDeadTimeConfig.DeadTime = 0;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
        {
                _Error_Handler(__FILE__, __LINE__);
        }

        HAL_TIM_MspPostInit(&htim8);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *tim_encoderHandle)
{

        GPIO_InitTypeDef GPIO_InitStruct;
        if (tim_encoderHandle->Instance == TIM1)
        {
                /* USER CODE BEGIN TIM1_MspInit 0 */

                /* USER CODE END TIM1_MspInit 0 */
                /* TIM1 clock enable */
                __HAL_RCC_TIM1_CLK_ENABLE();

                /**TIM1 GPIO Configuration    
                        PE9     ------> TIM1_CH1
                        PE11     ------> TIM1_CH2 
                        */
                GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

                /* USER CODE BEGIN TIM1_MspInit 1 */

                /* USER CODE END TIM1_MspInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM3)
        {
                /* USER CODE BEGIN TIM3_MspInit 0 */

                /* USER CODE END TIM3_MspInit 0 */
                /* TIM3 clock enable */
                __HAL_RCC_TIM3_CLK_ENABLE();

                /**TIM3 GPIO Configuration    
                        PA6     ------> TIM3_CH1
                        PA7     ------> TIM3_CH2 
                        */
                GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

                /* USER CODE BEGIN TIM3_MspInit 1 */

                /* USER CODE END TIM3_MspInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM4)
        {
                /* USER CODE BEGIN TIM4_MspInit 0 */

                /* USER CODE END TIM4_MspInit 0 */
                /* TIM4 clock enable */
                __HAL_RCC_TIM4_CLK_ENABLE();

                /**TIM4 GPIO Configuration    
                        PD12     ------> TIM4_CH1
                        PD13     ------> TIM4_CH2 
                        */
                GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
                HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

                /* USER CODE BEGIN TIM4_MspInit 1 */

                /* USER CODE END TIM4_MspInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM5)
        {
                /* USER CODE BEGIN TIM5_MspInit 0 */

                /* USER CODE END TIM5_MspInit 0 */
                /* TIM5 clock enable */
                __HAL_RCC_TIM5_CLK_ENABLE();

                /**TIM5 GPIO Configuration    
                        PA0-WKUP     ------> TIM5_CH1
                        PA1     ------> TIM5_CH2 
                        */
                GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

                /* USER CODE BEGIN TIM5_MspInit 1 */

                /* USER CODE END TIM5_MspInit 1 */
        }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{

        if (tim_pwmHandle->Instance == TIM8)
        {
                /* USER CODE BEGIN TIM8_MspInit 0 */

                /* USER CODE END TIM8_MspInit 0 */
                /* TIM8 clock enable */
                __HAL_RCC_TIM8_CLK_ENABLE();
                /* USER CODE BEGIN TIM8_MspInit 1 */

                /* USER CODE END TIM8_MspInit 1 */
        }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{

        GPIO_InitTypeDef GPIO_InitStruct;
        if (timHandle->Instance == TIM8)
        {
                /* USER CODE BEGIN TIM8_MspPostInit 0 */

                /* USER CODE END TIM8_MspPostInit 0 */

                /**TIM8 GPIO Configuration    
                        PC6     ------> TIM8_CH1
                        PC7     ------> TIM8_CH2
                        PC8     ------> TIM8_CH3
                        PC9     ------> TIM8_CH4 
                        */
                GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
                HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

                /* USER CODE BEGIN TIM8_MspPostInit 1 */

                /* USER CODE END TIM8_MspPostInit 1 */
        }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *tim_encoderHandle)
{

        if (tim_encoderHandle->Instance == TIM1)
        {
                /* USER CODE BEGIN TIM1_MspDeInit 0 */

                /* USER CODE END TIM1_MspDeInit 0 */
                /* Peripheral clock disable */
                __HAL_RCC_TIM1_CLK_DISABLE();

                /**TIM1 GPIO Configuration    
                        PE9     ------> TIM1_CH1
                        PE11     ------> TIM1_CH2 
                        */
                HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9 | GPIO_PIN_11);

                /* USER CODE BEGIN TIM1_MspDeInit 1 */

                /* USER CODE END TIM1_MspDeInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM3)
        {
                /* USER CODE BEGIN TIM3_MspDeInit 0 */

                /* USER CODE END TIM3_MspDeInit 0 */
                /* Peripheral clock disable */
                __HAL_RCC_TIM3_CLK_DISABLE();

                /**TIM3 GPIO Configuration    
                        PA6     ------> TIM3_CH1
                        PA7     ------> TIM3_CH2 
                        */
                HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);

                /* USER CODE BEGIN TIM3_MspDeInit 1 */

                /* USER CODE END TIM3_MspDeInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM4)
        {
                /* USER CODE BEGIN TIM4_MspDeInit 0 */

                /* USER CODE END TIM4_MspDeInit 0 */
                /* Peripheral clock disable */
                __HAL_RCC_TIM4_CLK_DISABLE();

                /**TIM4 GPIO Configuration    
                        PD12     ------> TIM4_CH1
                        PD13     ------> TIM4_CH2 
                        */
                HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13);

                /* USER CODE BEGIN TIM4_MspDeInit 1 */

                /* USER CODE END TIM4_MspDeInit 1 */
        }
        else if (tim_encoderHandle->Instance == TIM5)
        {
                /* USER CODE BEGIN TIM5_MspDeInit 0 */

                /* USER CODE END TIM5_MspDeInit 0 */
                /* Peripheral clock disable */
                __HAL_RCC_TIM5_CLK_DISABLE();

                /**TIM5 GPIO Configuration    
                        PA0-WKUP     ------> TIM5_CH1
                        PA1     ------> TIM5_CH2 
                        */
                HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

                /* USER CODE BEGIN TIM5_MspDeInit 1 */

                /* USER CODE END TIM5_MspDeInit 1 */
        }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle)
{

        if (tim_pwmHandle->Instance == TIM8)
        {
                /* USER CODE BEGIN TIM8_MspDeInit 0 */

                /* USER CODE END TIM8_MspDeInit 0 */
                /* Peripheral clock disable */
                __HAL_RCC_TIM8_CLK_DISABLE();
                /* USER CODE BEGIN TIM8_MspDeInit 1 */

                /* USER CODE END TIM8_MspDeInit 1 */
        }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
