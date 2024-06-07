#include "stm32f4xx.h" 
#include "Servos.h"
#include "tim.h"
#include "control_data.h"

//=================��������=================
servo_info_t servo_info[3]; 

//=================��������=================
void servos_init();
void servos_control(uint16_t angle,uint16_t id);

	

//=================�����ʼ��=================
void servos_init()
{
  
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

//===============���λ������=================
void servos_reset()
{
	//��λ�ù�0
	for(uint16_t i=0;i<2;i++)
	{
		servo_info[i].position     = 0;
		servo_info[i].target_angle = 0;
	}
	
	servos_control(servo_info[0].target_angle,servo_info[1].channel_id);
	servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
}

//500 --0   1000 -- 45   1500 --90   2000 ---135  2500 --180 
//=================�������=================
/*
һ�������������
idΪ5�Ķ�����ƻ�е�۵ĵ�����ؽڣ��Ըùؽھ����ϸ�ľ���Ҫ��
idΪ6�Ķ�����ƻ�е�ۼ�צ��
*/

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
		case 3://���ƻ�е�۵�5���ؽ�
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, temp);
	   break;
		}
		case 4://���Ƽ�צ
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, temp);
	   break;
		}
	}

}

