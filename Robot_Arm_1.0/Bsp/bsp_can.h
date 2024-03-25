#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "can.h"


#define Motar_mode 0	//����ģʽΪ����ģʽ��Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ

#define P_MIN -12.5		//λ����Сֵ
#define P_MAX 12.5		//λ�����ֵ
#define V_MIN -45			//�ٶ���Сֵ
#define V_MAX 45			//�ٶ����ֵ
#define KP_MIN 0.0		//Kp��Сֵ
#define KP_MAX 500.0	//Kp���ֵ
#define KD_MIN 0.0		//Kd��Сֵ
#define KD_MAX 5.0		//Kd���ֵ
#define T_MIN -18			//ת�����ֵ
#define T_MAX 18			//ת����Сֵ

typedef struct
{
	int p_int[3],v_int[3],t_int[3];						//����ɸ��ݵ����Ŀ�����޸ģ���ȡ���������λ�á��ٶȡ�ת��
	float position[3],velocity[3],torque[3];	//���������λ�á��ٶȡ�ת�ؽ����洢
	uint8_t  Tx_Data[8];												//���ݷ��ʹ洢
	uint8_t  RxData[8];												//���ݽ��մ洢
	CAN_RxHeaderTypeDef Rx_pHeader;
}CANx_t;
extern CANx_t CAN_1,CAN_2;


void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq);
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
void Motor_enable(void);
void Motor_control(void);

#endif
