#include "stm32f4xx.h"                  // Device header
#include "Arm_Task.h" 
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"
#include "bsp_can.h"
#include "Can_user.h"
#include "Uart_user.h"
#include "Servos.h"
#include "control_data.h"
#include "path_planning.h"



float theta;
float vel;
float acc;

uint16_t num=0;
//�����ʼ��
void motor_init();
//�������ݴ���
void data_operate();
//mit����ģʽ
void mit_contorl();
void mit_rc_contorl();

//�ٶ�λ��ģʽ����
void vp_control();

//�ٶȿ���ģʽ
void speed_control(); 

//�켣�滮�������ῼ������task��
void path_planning();
	
void arm_task(void const * argument)
{
		//motor_init();
	  //servos_init();
      path_planning();
		for(;;)
		{		 
			//servos_control(data.z,3);
			//servos_control(data.x,4);
//			data_operate();
//		  Speed_CtrlMotor(&motor_info[0].hcan, motor_info[0].can_id, motor_info[0].target_speed);
//    	HAL_Delay(1);
//      Speed_CtrlMotor(&motor_info[1].hcan, motor_info[1].can_id, motor_info[1].target_speed);
//      HAL_Delay(1);
//      Speed_CtrlMotor(&motor_info[2].hcan, 0x03, motor_info[2].target_speed);
//      HAL_Delay(1);
//      Speed_CtrlMotor(&motor_info[3].hcan, 0x04, motor_info[3].target_speed);
//      HAL_Delay(1);
      osDelay(1);
		}
		osDelay(1);
		
}
//================================�������==========================================================================
//�����ʼ��
void motor_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);       //���������Դ
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,  GPIO_PIN_RESET);     //�����
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     //������Դ1��� �ұ�
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_SET);   //������Դ2��� ���
	
	HAL_Delay(1000);	//��ʱ1sΪ�˸�DM_MC01��ɿص�Դ��Դ1�ȶ�����
	
	motor_info[0].hcan =  hcan2;
	motor_info[1].hcan =  hcan1;
	motor_info[2].hcan =  hcan1;
	motor_info[3].hcan =  hcan1;
	
	//���ģʽ����:Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ
	motor_info[0].mode = 2;  
	motor_info[1].mode = 2; 
	motor_info[2].mode = 2; 
	motor_info[3].mode = 2; 
	
	//���õ��id
	motor_info[0].can_id = 0x01; 
	motor_info[1].can_id = 0x02; 
	motor_info[2].can_id = 0x03;
	motor_info[3].can_id = 0x04;


	motor_commend(motor_info[0],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[1],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[2],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[3],Data_Enable);
	HAL_Delay(2000);
	

}
//���ݴ���
void data_operate()
{
	for(uint8_t i = 0;i<4;i++)
	{
			float v_float;
		  v_float = data.v_int[i];
		  motor_info[i].target_speed = v_float/127;
		  motor_info[i].target_angle = data.p_int[i];
	
	}
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

//================================�켣�滮===========================================================================
void path_planning()
{
	
	//�滮·��1
  motor_info[1].joint.theta_s = 10;
	motor_info[1].joint.theta_f = 90;
	motor_info[1].joint.vel_s   =  0;
	motor_info[1].joint.vel_f   =  0;
	motor_info[1].joint.acc_s   =  0;
  motor_info[1].joint.acc_f   =  0;
	motor_info[1].joint.t_s     =  0;
	motor_info[1].joint.t_f     = 10;
	
	a_path_cala(&motor_info[1].joint,&motor_info[1].path);
	a_path_cala(&motor_info[1].joint,&motor_info[1].path);
	for(int i =0;i<100;i++)
	{
		theta = motor_info[1].path.theta[i];
		vel   = motor_info[1].path.vel[i];
		acc   = motor_info[1].path.acc[i];
		HAL_Delay(10);
	}
	
	//�滮·��2
  motor_info[1].joint.theta_s = 90;
	motor_info[1].joint.theta_f = 45;
	motor_info[1].joint.vel_s   =  0;
	motor_info[1].joint.vel_f   =  0;
	motor_info[1].joint.acc_s   =  0;
  motor_info[1].joint.acc_f   =  0;
	motor_info[1].joint.t_s     = 10;
	motor_info[1].joint.t_f     = 20;
	
	a_path_cala(&motor_info[1].joint,&motor_info[1].path);
	a_path_cala(&motor_info[1].joint,&motor_info[1].path);
	for(int i =0;i<100;i++)
	{
		theta = motor_info[1].path.theta[i];
		vel   = motor_info[1].path.vel[i];
		acc   = motor_info[1].path.acc[i];
		HAL_Delay(10);
	}	
}

//���켣�滮��������joint��
 void joint_init(pos_data *joint)
 {
	  //������Ӧ�û����һ�����
	  motor_info[1].joint.theta_s = 90;
		motor_info[1].joint.theta_f = 45;
		motor_info[1].joint.vel_s   =  0;
		motor_info[1].joint.vel_f   =  0;
		motor_info[1].joint.acc_s   =  0;
		motor_info[1].joint.acc_f   =  0;
		motor_info[1].joint.t_s     = 10;
		motor_info[1].joint.t_f     = 20;
 }

