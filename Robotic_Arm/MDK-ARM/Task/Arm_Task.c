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
float vel = 0;
float acc = 0;

uint16_t num=0;
//�����ʼ��
static void motor_init();
//ģʽѡ��
static void mode_chose();
//ģʽ0
static void motor_stop_and_keep();
//ģʽ1
static void motor_remote_control();
//ģʽ2
static void motor_control_kinemat();
//ģʽ3
static void mode_3();
//�������ݴ���
static void data_operate();
//���Ƶ��
static void motor_control_send();


	
float mat[4][4] = {0};
void arm_task(void const * argument)
{
		motor_init();
	  servos_init();
	 
		for(;;)
		{		 
			 //mode_chose();
			motor_remote_control();
			motor_control_send();
      osDelay(1);
		}
		osDelay(1);
		
}

//================================���ݳ�ʼ��==========================================
//�����ʼ��
static void motor_init()
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
	motor_info[0].mode = 1;  
	motor_info[1].mode = 1; 
	motor_info[2].mode = 1; 
	motor_info[3].mode = 1; 
	
	//���õ��id
	motor_info[0].can_id = 0x01; 
	motor_info[1].can_id = 0x02; 
	motor_info[2].can_id = 0x03;
	motor_info[3].can_id = 0x04;


  //��ʱ�ȴ����ʹ��
	motor_commend(motor_info[0],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[1],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[2],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[3],Data_Enable);
	HAL_Delay(2000);
	
}

//================================ģʽѡ��==========================================
void mode_chose()
{
	if(data.start == 1)//�����ܿ���
	{
			if(data.mode == 1)//ģʽѡ��
					{
						motor_remote_control(); //���������ģʽ
					}
			else if(data.mode == 2)
					{
						motor_control_kinemat();//ȫ�Զ�·���滮ģʽ
					}
			else if(data.mode == 3)
					{
						servos_reset();         //���ö��λ��
					}
			else
			{
				motor_stop_and_keep();//ֹͣ�����ֵ�ǰλ��
			}
	}
	else
	{
		motor_stop_and_keep();//ֹͣ�����ֵ�ǰλ��
	}

}
//================================ģʽ0 ���־�ֹ====================================
void motor_stop_and_keep()
{ 
	 for(uint16_t i =0;i<4;i++)
	{
	  motor_info[i].target_angle = motor_info[i].position;
		motor_info[i].target_speed = 0;
	}
	 for(uint16_t i =0;i<4;i++)
	{
		 PosSpeed_CtrlMotor(&motor_info[i].hcan,motor_info[i].can_id, motor_info[i].target_angle, motor_info[i].target_speed);
		 HAL_Delay(1);
	}
}
//================================ģʽ1 �ֶ�����====================================
void motor_remote_control()
{
   
	//���ݽ��д���
	for(uint8_t i = 0;i<4;i++)
	{
		//ͨ��ֵ����
		if(data.channel[i]>40)
		{
			float add = data.channel[i]-40;
		  motor_info[i].target_angle = motor_info[i].target_angle + add/12700;
		}
		else if(data.channel[i]<-40)
		{
			float add = data.channel[i]+40;
		  motor_info[i].target_angle = motor_info[i].target_angle + add/12700;
		}
		
		//�Ƕ�Խ�紦��
		if(motor_info[i].target_angle > 3.0 )
		{
			motor_info[i].target_angle = 3.0;
		}
		else if(motor_info[i].target_angle < -3.0)
		{
			motor_info[i].target_angle = -3.0;
		}
		else
		{
		}
	}
	
	servo_info[0].target_angle = data.channel[4];
	servo_info[1].target_angle = 500 + data.tool * 1000;
	
	for(uint16_t i=0;i<4;i++)
	{
		motor_info[i].target_speed = 5;
	}
	

}
//================================ģʽ3 �Զ�����====================================
void motor_control_kinemat()
{ 
	for(uint16_t i =0;i<4;i++)
	{
		 PosSpeed_CtrlMotor(&hcan1,motor_info[i].can_id, motor_info[i].target_angle, motor_info[i].target_speed);
		 HAL_Delay(1);
	}
	servos_control(servo_info[0].target_angle,servo_info[0].channel_id);
	servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
}

//==================================���������Ϣ����====================================
void motor_control_send()
{
		PosSpeed_CtrlMotor(&motor_info[0].hcan,motor_info[0].can_id, motor_info[0].target_angle, motor_info[0].target_speed);
		HAL_Delay(1);
		PosSpeed_CtrlMotor(&motor_info[1].hcan,motor_info[1].can_id, motor_info[1].target_angle, motor_info[1].target_speed);
		HAL_Delay(1);
		PosSpeed_CtrlMotor(&motor_info[2].hcan,motor_info[2].can_id, motor_info[2].target_angle, motor_info[2].target_speed);
		HAL_Delay(1);
		PosSpeed_CtrlMotor(&motor_info[3].hcan,motor_info[3].can_id, motor_info[3].target_angle, motor_info[3].target_speed);
		HAL_Delay(1);
	
		servos_control(servo_info[0].target_angle,servo_info[0].channel_id);
	  servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
}

//==================================����ģʽ====================================
//mit����ģʽ
/*
	��λ�ý��п���ʱ��kd ���ܸ� 0���������ɵ���𵴣�����ʧ��
	���� MIT ģʽ�������������ֿ���ģʽ���� kp=0,kd ��Ϊ 0 ʱ������ v_des
	����ʵ������ת��;kp=0,kd=0������ t_ff ����ʵ�ָ���mtŤ�����
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














