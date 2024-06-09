#include "stm32f4xx.h" 
#include "Servos.h"
#include "tim.h"
#include "control_data.h"

//=================变量定义=================
servo_info_t servo_info[3]; 

//=================函数声明=================
void servos_init();
void servos_control(float angle,uint16_t id);

//=================舵机初始化=================
void servos_init()
{
  servo_info[0].channel_id = 3;
	servo_info[1].channel_id = 4;
	
	servo_info[0].position = 0;
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

//===============舵机位置重置=================
void servos_reset()
{
	//舵位置归0
	for(uint16_t i=0;i<2;i++)
	{
		servo_info[i].position     = 0;
		servo_info[i].target_angle = 0;
	}
	
	servos_control(servo_info[0].target_angle,servo_info[1].channel_id);
	servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
}

//关节5     1315 中位置0     1940 顺时针90   690 逆时针90 
//1940 -1315 = 625 ； 1315 - 690 = 625 
//夹爪      1500 夹取        500松开

//=================舵机控制=================
/*
一共有两个舵机，
id为5的舵机控制机械臂的第五个关节，对该关节具有严格的精度要求
id为6的舵机控制机械臂夹爪。
*/

void servos_control(float angle,uint16_t id)
{

	switch(id)
	{
		case 3://控制机械臂第5个关节
		{
			//将弧度值换算为舵机的控制频率
			angle = angle * 398.1 + 1315;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angle);
	   break;
		}
		case 4://控制夹爪
		{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angle);
	   break;
		}
	}

}

