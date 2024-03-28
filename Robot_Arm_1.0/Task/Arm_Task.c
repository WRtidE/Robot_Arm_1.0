#include "stm32f4xx.h"                  // Device header
#include "Arm_Task.h" 
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"
#include "bsp_can.h"
#include "Can_user.h"
#include "remote_control.h"
#include "Uart_user.h"

//�����ʼ��
void motor_init();

//mit����ģʽ
void mit_contorl();
void mit_rc_contorl();

//�ٶ�λ��ģʽ����
void vp_control();

//�ٶȿ���ģʽ
void speed_control();

void arm_task(void const * argument)
{
		motor_init();
	 
		for(;;)
		{		  
			   //send_packet();
				 //data_send();
			   //data_receive();

		}
		osDelay(1);
		
}


//�����ʼ��
void motor_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);     //���������Դ
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,  GPIO_PIN_RESET);     //�����
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     //������Դ1��� �ұ�
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_RESET);   //������Դ2��� ���
	
	HAL_Delay(1000);	//��ʱ1sΪ�˸�DM_MC01��ɿص�Դ��Դ1�ȶ�����
	
	//���ģʽ����:Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ
	motor_info[0].mode = 0;  
	motor_info[1].mode = 0; 
	motor_info[2].mode = 2; 
	
	//���õ��id
	motor_info[0].can_id = 0x01; 
	motor_info[1].can_id = 0x02; 
	motor_info[2].can_id = 0x03;
	motor_info[2].can_id = 0x04;

	//���pid ֱ���ڴ��ڵ�������
//	motor_info[0].Kp =  3;
//	motor_info[0].Ki =  0;
//	motor_info[0].Kd =  1;
//	motor_info[1].Kp =  0;
//	motor_info[1].Ki =  0;
//	motor_info[1].Kd =  1;
	
	motor_commend(motor_info[0],Data_Enable);
	motor_commend(motor_info[1],Data_Enable);
	motor_commend(motor_info[2],Data_Enable);
	motor_commend(motor_info[3],Data_Enable);
}


//mit����ģʽ
/*
	��λ�ý��п���ʱ��kd ���ܸ� 0���������ɵ���𵴣�����ʧ��
	���� MIT ģʽ�������������ֿ���ģʽ���� kp=0,kd ��Ϊ 0 ʱ������ v_des
	����ʵ������ת��;kp=0,kd=0������ t_ff ����ʵ�ָ���Ť�����
*/
void mit_contorl()
{

	MIT_CtrlMotor(&hcan1,0x02, motor_info[1].target_angle,
																						 motor_info[1].target_speed,
																						 motor_info[1].Kp,
																						 motor_info[1].Kd, 
																						 motor_info[1].target_T_ff);
}


//�ٶ�λ��ģʽ����
void vp_control()
{
	PosSpeed_CtrlMotor(&hcan1,motor_info[0].can_id, motor_info[0].target_angle, motor_info[0].target_speed);
}





