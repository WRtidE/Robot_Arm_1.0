#ifndef __SERVOS_H
#define __SERVOS_H
#include "Servos.h"

//��������
void servos_control(uint16_t angle,uint16_t id);
void servos_init();
void servos_reset();
	
//��������Ϣ
typedef struct
{	  
	  uint16_t channel_id;		     //ͨ����
		
	  //��Ϣ����
		float position;     //���λ��
		float velocity;     //����Ƕ�
		float torque;       //���ת��
		
		//�����������
		float target_speed; //Ŀ���ٶ�
		float target_angle; //Ŀ��Ƕ�
		float target_T_ff;  //Ŀ��ת��
	
}servo_info_t;

extern servo_info_t servo_info[3]; 
#endif
