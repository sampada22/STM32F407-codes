#include <stm32f4xx.h>
#include <stdio.h>

//initialize the timer variables
volatile uint8_t timespan = 0;
volatile uint8_t lastcounter = 0;
volatile uint8_t newcounter = 0;
volatile uint8_t overflow = 0;
volatile uint8_t PulseEnd = 0;

//Functions used in the program
void SysTick_Handler(void);
void SetHSI(void);
void LED_GPIO(void);
void TIM4_C1_Init(void);
void TIM2_C3_Init(void);
void TIM4_IRQHandler(void);
void LED(void);

void setSysTick(void){
//SysTick timer(1 ms)
	if (SysTick_Config(SystemCoreClock/1000)){
	while(1){
	}
	
	}
}

volatile uint32_t msTicks;//variable to count 1ms time ticks
	void SysTick_Handler(void){
	msTicks++;
	}

static void Delay(__IO uint32_t dlyTicks){
uint32_t curTicks = msTicks;
	while ((msTicks - curTicks)<dlyTicks){}
}

int main(void){

SysTick_Handler();
setSysTick();
SetHSI();
LED_GPIO();
TIM2_C3_Init();
TIM4_C1_Init();

while(1){
LED();
Delay(100);	
}	

}

void SetHSI(void){

RCC ->CR |= RCC_CR_HSION; //turn on HSI oscillator
while((RCC->CR &RCC_CR_HSIRDY)==0);
RCC ->CFGR &= ~RCC_CFGR_SW_HSI; //select HSI clock
RCC -> CFGR |= RCC_CFGR_SWS_HSI; 
	while((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_HSI);//wait for HSI stabilize

}
//Configure GPIO Port D
void LED_GPIO(void){
RCC ->AHB1ENR |= RCC_AHB1ENR_GPIODEN;// Enable GPIO clock
GPIOD->MODER |= GPIO_MODER_MODER12_0;	//general purpose input output mode
GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1;
	

}
//Configure TIM2 for sending output signal
void TIM2_C3_Init(void){
RCC ->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;//reset GPIOB clock
RCC ->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOBRST;//cler RESET
RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable the GPIOB clock
	
//PB10 Pulse Generating Pin
GPIOB -> MODER &= ~(GPIO_MODER_MODE10_0);	   //RESET the PINB10
GPIOB -> MODER &= ~(GPIO_MODER_MODE10_1);
GPIOB -> MODER |= (GPIO_MODER_MODE10_1);     //set PINB10 for alternate function mode

GPIOB ->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10_0);  //reset the speed register
GPIOB ->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10_1);
GPIOB ->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10_1);  // set the high speed 

GPIOB -> OTYPER &= ~(GPIO_OTYPER_OT10);       //push-pull mode

GPIOB ->PUPDR &= ~(GPIO_PUPDR_PUPD10_0);       //no pull up pull down
GPIOB ->PUPDR &= ~(GPIO_PUPDR_PUPD10_1);

GPIOB ->AFR[1]= (GPIO_AFRL_AFRL0_1 );         //set PB pin 10 as AF1 (TIM2_CH3) 

RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
TIM2 -> PSC = 15;
TIM2 ->ARR =0XFFFF;
	
TIM2 ->CCMR2 |=( TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);	//PWM mode 1
TIM2 ->CCMR2 |=(TIM_CCMR2_OC3PE);//CH3 output preload enable
TIM2 ->CR1 |= (TIM_CR1_ARPE);//auto reload preload enable
TIM2 ->CCER |= (TIM_CCER_CC3E);//enable output for channel 3
TIM2 ->EGR |= (TIM_EGR_UG);//force update
TIM2 ->SR &= ~(TIM_SR_UIF);//clear the update flag
TIM2 -> DIER |= (TIM_DIER_UIE); //enable interrupt on update
TIM2 -> CR1 |=(TIM_CR1_DIR);   //set downcounting counter direction
TIM2 ->CR1 |= (TIM_CR1_CEN);	//enable counter
}	

void TIM4_C1_Init(void){

RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable the GPIOB clock
	
//PB6 echo pin
GPIOB -> MODER &= ~( GPIO_MODER_MODE6_0);//reset the moder register for PINB6
GPIOB -> MODER &= ~( GPIO_MODER_MODE6_1);
GPIOB -> MODER |= (GPIO_MODER_MODE6_1); //set for alternate function mode

GPIOB ->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6_0 );//reset the speed register for PINB6
GPIOB ->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6_1 );	
GPIOB ->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6_1 );//set the speed of GPIOB for high speed
	
GPIOB ->OTYPER &= ~(GPIO_OTYPER_OT6);	    //set the PINE2 for push-pull mode

GPIOB ->PUPDR &= ~(GPIO_PUPDR_PUPD6_0);	 //no pull up pull down mode
GPIOB ->PUPDR &= ~(GPIO_PUPDR_PUPD6_1);	
	
GPIOB ->AFR [0]&= ~(GPIO_AFRL_AFRL0_1 );
GPIOB ->AFR [0]= (GPIO_AFRL_AFRL0_1 );      //set PB pin 6 as AF2 (TIM4_CH1) 

RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;//enable TIM4 clock
TIM4 ->PSC = 15; //set prescalar to 15
TIM4 ->ARR = 0XFFFF;//set max pulse width of 65536
TIM4 -> CCMR1 |= (TIM_CCMR1_CC1S_0|TIM_CCMR1_CC1S_1);//clear capture/compare register	
TIM4 -> CCMR1 |= 0x1;//select CH1 input capture
TIM4 ->CCMR1 &= ~(TIM_CCMR1_IC1F);//disable digital filtering
TIM4 -> CCER |= (TIM_CCER_CC1P);	//select both rising and falling edge
TIM4 -> CCMR1 &= ~(TIM_CCMR1_IC1PSC);//input prescalar to 0 to capture each valid edge
TIM4 ->CCER |= TIM_CCER_CC1E;   //enable counter capture
TIM4 ->DIER |= TIM_DIER_CC1IE;	//enable channel 1 capture/compare interrrupt
NVIC_SetPriority(TIM4_IRQn,1);//set priority to 1
NVIC_EnableIRQ(TIM4_IRQn);	//enable TIM4 interrut on NVIC
}		


void TIM4_IRQHandler(void){
if ((TIM4->SR & TIM_SR_UIF)!= 0){   //check the update event flag
overflow++;                         //if UIF = 1, increment overflow flag
TIM4 -> SR &= ~(TIM_SR_UIF);        //clear UIF
}
if((TIM4->SR & TIM_SR_CC1IF)!=0){    //check capture event flag
newcounter = TIM4->CCR1;             //read capture value,store as newcounter  
timespan = (newcounter - lastcounter)+65536*overflow;//calculate the total pusle width
lastcounter = newcounter;//save the value of new couter as lastcounter to be used for the nexr cycle
overflow = 0;         //clear overflow counter
}

}
void LED (void){
	
	double  Distance;
	Distance = (timespan/58.0);
	
	if(Distance>0.0 && Distance <=100.0){
	GPIOB ->BSRR |= GPIO_BSRR_BS7;
	}
	else{
	GPIOB ->BSRR &= ~(GPIO_BSRR_BS7);
	}
}
	

