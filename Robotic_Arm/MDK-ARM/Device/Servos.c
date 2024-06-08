#include "stm32f4xx.h" 
#include "Servos.h"
#include "tim.h"
#include "control_data.h"

//=================��������=================
servo_info_t servo_info[3]; 

//=================��������=================
void servos_init();
void servos_control(float angle,uint16_t id);

float flagggg  = 0;
float flaggggg = 0;
//=================�����ʼ��=================
void servos_init()
{
  servo_info[0].channel_id = 3;
	servo_info[1].channel_id = 4;
	
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

//�ؽ�5     1326 ��λ��0     700 ��90   2030 ��90  
//��צ      1500 ��ȡ        500�ɿ�

//=================�������=================
/*
һ�������������
idΪ5�Ķ�����ƻ�е�۵ĵ�����ؽڣ��Ըùؽھ����ϸ�ľ���Ҫ��
idΪ6�Ķ�����ƻ�е�ۼ�צ��
*/

void servos_control(float angle,uint16_t id)
{
	if(angle<0)
	{
		angle=-angle;
	}

	switch(id)
	{
		case 3://���ƻ�е�۵�5���ؽ�
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angle);
	   break;
		}
		case 4://���Ƽ�צ
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angle);
	   break;
		}
	}

}

