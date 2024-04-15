#include "stm32f4xx.h" 
#include "Servos.h"
#include "tim.h"

servo_info_t servo_info[3]; 

void servos_init()
{

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

//500 --0   1000 -- 45   1500 --90   2000 ---135  2500 --180 

void servos_control(uint16_t angle,uint16_t id)
{
	float temp;
	temp = angle*(2500/127);
	
	if(temp<0)
	{
		temp=-temp;
	}
	
	switch(id)
	{
		case 3:
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, temp);
	   break;
		}
		case 4:
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, temp);
	   break;
		}
	}

}

