#include "stm32f4xx.h" 
#include "Servos.h"
#include "tim.h"


void servos_init()
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

//500 --0   1000 -- 45   1500 --90   2000 ---135  2500 --180 

void servos_control(uint16_t angle)
{
	float temp;
	temp = angle*(2500/127);
	if(temp<0)
	{
		temp=-temp;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, temp);
}
