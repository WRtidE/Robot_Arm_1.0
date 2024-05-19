#ifndef CAN_USER_H
#define CAN_USER_H

#include "path_planning.h"

extern uint8_t Data_Enable[8];    	  //电机使能命令
extern uint8_t Data_Failure[8];    		//电机失能命令
extern uint8_t Data_Save_zero[8];     //电机保存零点命令
extern uint8_t Data_Error_clear[8];	  //电机清除错误

//储存电机信息
typedef struct
{
    CAN_HandleTypeDef  hcan; //can线设置
	  uint16_t can_id;		     //ID号
		uint16_t mode;           //设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式
		
	  //信息接收
		float position;     //电机位置
		float velocity;     //电机角度
		float torque;       //电机转矩
		
		//电机数据设置
		float target_speed; //目标速度
		float target_angle; //目标角度
		float target_T_ff;  //目标转矩
		
	 //电机PID
		float Kp;//位置比例系数
		float Ki;
		float Kd;//位置微分系数
	 
}motor_info_t;

extern motor_info_t motor_info[4]; //表示有四个电机


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);

void can_filter_init(void);

void motor_commend(motor_info_t motor,uint8_t *pData);

#endif
