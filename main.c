#include "stm32f4xx_hal.h"
void GPIO_Config(void);
void ADC_Config(void);
void TIM_Config(void);

ADC_HandleTypeDef myADChandle;
int main(){

while(1){


}
}

void GPIO_Config(void){
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef myPin;
	myPin.Pin = GPIO_PIN_1;
	myPin.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA,&myPin);
	
	myPin.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	myPin.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOD,&myPin);
	
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
	HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
}

void ADC_Config(void){
__HAL_RCC_ADC1_CLK_ENABLE();
	myADChandle.Instance = ADC1;
	myADChandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	myADChandle.Init.ContinuousConvMode = DISABLE;
	
	
	


}