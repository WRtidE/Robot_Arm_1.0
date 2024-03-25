#ifndef CAN_USER_H
#define CAN_USER_H


extern uint8_t Data_Enable[8];    	  //���ʹ������
extern uint8_t Data_Failure[8];    		//���ʧ������
extern uint8_t Data_Save_zero[8];     //��������������
extern uint8_t Data_Error_clear[8];	  //����������

//��������Ϣ
typedef struct
{
    uint16_t can_id;		//ID��
		uint16_t mode;      //����ģʽΪ����ģʽ��Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ
		
	  //��Ϣ����
		float position;     //���λ��
		float velocity;     //����Ƕ�
		float torque;       //���ת��
		
		//�����������
		float target_speed; //Ŀ���ٶ�
		float target_angle; //Ŀ��Ƕ�
		float target_T_ff;  //Ŀ��ת��
		
	 //���PID
		float Kp;//λ�ñ���ϵ��
		float Ki;
		float Kd;//λ��΢��ϵ��
			
}motor_info_t;

extern motor_info_t motor_info[4]; //��ʾ���ĸ����


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);

void can_filter_init(void);

void motor_commend(motor_info_t motor,uint8_t *pData);

#endif
