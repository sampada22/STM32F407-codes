#include "stm32f4xx.h"
int main(void){
// ADC Calibration(For this the ADC should be disabled first)
	ADC1->CR2 &= ~(ADC_CR2_ADON);
	ADC1->CR1 |= (ADC_CR1_ADCAL);
	//Select a clock source for the ADC
	RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN);
	RCC -> CR |= (RCC_CR_HSION);
	while((RCC -> CR & RCC_CR_HSIRDY) == 0){
	
	}
	ADC1->ADC1
	//Enable the ADC
	
while(1){
//Start Conversion

}

}