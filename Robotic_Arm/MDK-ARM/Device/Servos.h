#ifndef __SERVOS_H
#define __SERVOS_H
#include "Servos.h"

//函数声明
void servos_control(uint16_t angle,uint16_t id);
void servos_init();
void servos_reset();
	
//储存电机信息
typedef struct
{	  
	  uint16_t channel_id;		     //通道号
		
	  //信息接收
		float position;     //舵机位置
		float velocity;     //舵机角度
		float torque;       //舵机转矩
		
		//舵机数据设置
		float target_speed; //目标速度
		float target_angle; //目标角度
		float target_T_ff;  //目标转矩
	
}servo_info_t;

extern servo_info_t servo_info[3]; 
#endif
