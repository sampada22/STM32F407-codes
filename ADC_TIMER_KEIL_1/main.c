#define ANY_VALUE_YOU_WANT
#define ANY_DELAY_REQUIRED = 0x0FFF
#include"stm32f4xx.h"
 
	//TIMER TIME_BASE CONFIGURATION
	

	/*Hardware precision delay loop implementation using TIM6 timer peripheral.
	Any other STM32 timer can be used to fulfill this function, but TIM6 was chosen 
	as it has the less integration level. Other timer peripherals may be reserved for 
	more complicated tasks*/
	
	//Clear the update event flag
	TIM6->SR = 0;
	
	//set the required delay
	
	//The timer prescalar register reset value is 0. If a longer delay is required the
	//presalar register may be configured to
	TIM6 ->PSC = 0
	TIM6 ->ARR = ANY_DELAY_REQUIRED;
	
	//start the timer counter
	TIM6 ->CR1|= TIM_CR1_CEN;
	//loop until the update event flag is set
	while(!(TIM6 ->SR & TIM_SR_UIF));
	//the required time delay has been elapsed
	
	//user code can be executed
	
	
	//TIMER CHANNEL CONFIGURATION IN INPUT MODE
	
	//variable to store timestamp for the last detected active edge
	uint32_t TimeStamp;
	//the ARR register reset value is 0x0000FFFF for TIM3 timer. so it should be ok for this snippet
	//if you want to change it then, proceed as follows
	TIM3->ARR = ANY_VALUE_YOU_WANT;
//set the TIM3 timer channel as input
//CC1s bits are writable only when the channel1 is off
//After reset, all the timer channels are turned off
TIM3 -> CCMR1|= TIM_CCMR1_CC1S_0;
/*Enable the TIM3 channel1 and keep the default configuration(state after reset) for channel polarity*/
TIM3 -> CCER |= TIM_CCER_CC1E;
/* start the timer counter*/
TIM3 -> CR1 |= TIM_CR1_CEN;
/*Clear the capture event flag for channel1*/
TIM3->SR = ~TIM_SR_CC1IF;
/*Loop until the capture event flag is set*/
while(!(TIM3->SR&TIM_SR_CC1IF));
/*An active edge was detected so store the timestamp*/
TimeStamp = TIM3->CCR1;




TIMER CHANNEL IN OUTPUT MODE
/*The ARR register value is 0x0000FFFF for TIM3 timer. So it should be ok for this
snippet.If we want to change, proceed as the next line*/
TIM3 -> ARR = ANY_VALUE_YOU_WANT;
/* The TIM3 timer channel 1 after reset is configured as output*/
/* TIM3 ->CC1S reset value is 0;*/
/* To select PWM2 output mode set the OC1M control bit field to 111*/
TIM3 ->CCMR1 |= TIM_CCMR1_OC1M_0| TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;
/*Set the duty cycle to 50%*/
TIM3 ->CCR1 = TIM3->ARR/2;
/*By default, after reset, preload for channel 1 is turned off*/
/*To change it, proceed as follows*/
TIM3 -> CCMR1 |= TIM_CCMR1_OC1PE;
/*Enable the TIM3 channel1 and keep the default configuration(state after reset) for channel polarity*/
TIM3 ->CCER |= TIM_CCER_CC1E;
/*start the timer counter*/
TIM3 ->CR1|= TIM_CR1_CEN;




/*TIM2 Configuration*/
/*TIM2 clock enable*/
RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
/*Set the Timer prescalar to get 8Mhz as counter clock*/
Prescalar = (uint16_t)(SystemCoreClock/8000000)-1;
/*Reset the SMCR register*/
TIM2 ->SMCR = RESET;
#ifdef USE_ETR
/*Configure the ETR prescalar = 4*/
TIM2 ->SMCR |= TIM_ETRPRESCALAR_DIV4
/*Configure the polarity = Rising edge*/
TIM_ETRPOLARITY_NONINVERTED|
/*Configure the ETR clock ource*/
TIM_SMCR_ECE;
#else /*Internal clock source*/\
TIM2 -> SMCR &= ~ TIM_SMCR_SMS;
#endif /*USE_ETR*/
TIM2 -> CR1 &= ~(TIM1_CR1_DIR|TIM_CR1_CMS);
/*Select the up counter mode*/
TIM2 ->CR1 |= TIM_COUNTERMODE_UP;
TIM2 -> CR1 &= ~TIM_CR1_CKD;
/* Set the clock division to 1*/
TIM2 ->CR1 |= TIM_CLOCKDIVISION_DIV1;
/*Set the autoreload value*/
TIM2->ARR = PERIOD;
/*Set the Prescalar value*/
TIM2->PSC = (SystemCoreClock/8000000)-1;
/*Generate an update evnet to relaod the prescalar value immediately*/
TIM2 ->EGR = TIM_EGR_UG;
TIM2 -> CCMR1 &= ~TIM_CCMR1_CC2S;
/*Connect the timer input to IC2*/
TIM2 -> CCMR1 |= TIM_CCMR1_CC2S_0;


/*INPUT CAPTURE CONFIGURATION*/
/*Enable the DMA1 */
DMA1_Channel7 ->CCR |= DMA_CCR_EN;
/*Enable the TIM Capture/Compare2 DMA request*/
TIM2 ->DIER |= TIM_DMA_CC2;/*Enables the TIM capture compare channel 2*/
TIM2 ->CCER |= TIM_CCER_CC2E;
/*Enable the TIM2*/
TIM2 -> CR1 |= TIM_CR1_CEN;
/*wait until the transfer complete*/
while((DMA1 -> ISR &DMA_ISR_TCIF7)== RESET)()



/*N PULSE WAVEFORM GENERATION USING ONE PULSE MODE CONFIGURATION*/
/*Peripheral clock enable*/
RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
/*Set the Timer prescalar to get 1MHz as counter clock*/
Prescalar = (uint16_t)(SystemCoreClock/1000000)-1;
/*select the up counter mode*/
TIM1 ->CR1 &= ~(TIM_CR1_DIR|TIM_CR1_CMS);
TIM1 -> CR1 |= TIM_COUNTERMODE_UP;
/*set the clock division to 1*/
TIM1 ->CR1 &= ~TIM_CR1_CKD;
TIM1 -> CR1 |= TIM_CLOCKDIVISION_DIV1;
/*Set the autoreload value*/
TIM1 -> ARR = PERIOD;
/*set the Pulse value*/
TIM1 ->CCR1 = PULSE;
/*Set the prescalar value*/
TIM1 -> PSC = Prescalar;
/*Set the repitition counter value*/
TIM1 ->RCR = PULSE_NUMBER -1;
/*Generate an update event to reload the prescalar and the repition 
counter value immediately*/
TIM1 -> EGR = TIM_EGR_UG;
/*Configure the internal clock source*/
TIM1 ->SMCR = RESET;
/*Select the OPM Mode*/
TIM1 ->CR1 |= TIM_CR1_OPM;
TIM1 ->CCMR1 &=(uint16_t)~TIM_CCMR1_OC1M;
TIM1 ->CCMR2 &=(uint16_t)~TIM_CCMR1_CC1S;
TIM1 ->CCMR1 |= TIM_OCMODE_PWM2;
/*Select the channel 1 output compare and the mode*/
TIM1 ->CCER &= (uint16_t)~TIM_CCER_CC1P;
/*Set the output compare polarity to high*/
TIM1 ->CCER |= TIM_OCPOLARITY_HIGH;
/*Enable the compare output channel 1*/
TIM1 -> CCER |= TIM1_CCER_CC1E;
/*Enable the TIM main output*/
TIM1 -> BDTR |= TIM_BDTR_MOE;
/*Enable the TIM Peripheral*/
TIM1 ->CR1 |= TIM_CR1_CEN;



/*BREAK FUNCTION CONFIGURATION*/
/*TIM1 clock enable*/
RCC -> APB2ENR |=  RCC_APB2ENR_TIM1EN;
/*Set the timer prescalar to get 500 kHz as counter clock*/
Prescalar = (uint16_t)(SystemCoreClock/50000)-1;
/*Select the up counter mode*/
TIM1 ->CR1 &= ~(TIM_CR1_DIR | TIM1_CR1_CMS);
TIM1 -> CR1 |= TIM_COUNTERMODE_UP;
/* Set the clock division to 1*/
TIM1 -> CR1 &= ~TIM_CR1_CKD;
TIM1 -> CR1 |= TIM_CLOCKDIVISION_DIV1;
/*Set the autoreload value*/
TIM1 ->ARR = PERIOD;
/*Set the capture compare register value*/
TIM1 -> CCR1 = PUSLE;
/*set the prescalar value*/
TIM1 -> PSC =  Prescalar;
/*Generate an update event to reload the prescalar and the repition 
counter immendiately*/
TIM1 -> EGR = TIM_EGR_UG;
/*configure the internal clock source*/
TIM1 -> SMCR = RESET;
/*Clear the BDTR bits*/
TIM1  -> BDTR = RESET;
/*Set the Dead time value to 0*/
TIM1 ->BDTR |= DEAD_TIME;
/*Disable the lock level*/
TIM1 ->BDTR |= TIM_LOCKLEVEL_OFF;
/*Enable the output idle mode*/
TIM1 -> BDTR |= TIM_OSSI_ENABLE;
/*Disable the output run mode*/
TIM1 -> BDTR |= TIM_OSSR_DISABLE;
/*Enable the break input*/
TIM1 -> BDTR |= TIM_BREAK _ENABLE;
/*Set the polarity to high*/
TIM1 ->BDTR |=TIM_BREAKPOLARITY_HIGH;
/*Enable the automatic output*/
TIM1 ->BDTR |= TIM_AUTOMATICOUTPUT_ENABLE;
/*Selet channel 1 output comapre and mode*/
TIM1 ->CCMR1 &= ~TIM_CCMR1_OC1M;
TIM1 ->CCMR1 &= ~TIM_CCMR1_CC1S;
TIM1 ->CCMR1 |= TIM_OCMODE_PWM1;
/*Set the ouput compare polarity to high*/
TIM1 ->CCER &= ~TIM_CCER_CC1P;
TIM1 ->CCER |= TIM_OCPOLARITY_HIGH;
/*Enable the compare output channel 1*/
TIM1 -> CCER |= TIM_CCER_CC1E;
/*Enable the TIM peripheral*/
TIM1 ->CR1 |= TIM_CR1_CEN;


