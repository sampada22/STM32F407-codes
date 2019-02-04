#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void motor_init(void);
void forward_direction(void);
void backward_direction(void);

int main(){

}


void motor_init(void){
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();	

}
void forward_direction(void){




}