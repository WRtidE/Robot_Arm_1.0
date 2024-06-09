#include "stm32f4xx.h"                  // Device header
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"
#include "bsp_can.h"
#include "Can_user.h"
#include "Uart_user.h"
#include "Servos.h"
#include "control_data.h"
#include "path_planning.h"
#include "IK.h"

//================================��������================================
//���漸����Ҫpos
float inital_pos[5] = {0,0,0,1.5708,0}; //��ʼλ��
float   exit_pos[5] = {0,-1.8,1.22,0.73,0};  //��ȫ����λ��

float finish_flag = 1;
float martx[4][4];
float T_target[4][4];
float T_res[5];
float T_test[5];

//���������켣����������켣��Ϣ
path first_path;
path second_path;

float state_check = 0;
static float pi=3.1415;
//================================��������================================
static void arm_init();

//ģʽѡ��
static void mode_choice();
static void catch_the_object();
static void back_to_exit();
static void back_to_inital();

//�켣�滮����
static void path_plan(float x,float y,float z,float time_start,float time_final);

//================================������==================================
void kinemat_task(void const * argument)
{
   arm_init();

	for(;;)
		{		 
			if(data.start == 1)
			{
				if(data.mode == 1 && finish_flag == 0)
				{
					back_to_exit();
					finish_flag = 1;
				}
				else if(data.mode == 2 && finish_flag == 0)
				{
					path_plan(-200,200,10,0,3);
					finish_flag = 1;
				}
				else if(data.mode == 0 && finish_flag == 0)
				{
					back_to_inital();
					finish_flag = 1;
				}			
			}	
			else
			{
				finish_flag = 0;
			}
//	  state_check =0;
//		mode_choice();
		osDelay(1);
	 }

}
//================================���ݳ�ʼ��===============================
void arm_init()
{
	//����DH��
	arm.a[0] = 0;
	arm.a[1] = 0;
	arm.a[2] = 250;
	arm.a[3] = 250;
	arm.a[4] = 0;
	
	arm.d[0] = 63;
	arm.d[1] = 0;
	arm.d[2] = 0;
	arm.d[3] = 0;
	arm.d[4] = 116;

}
//================================ģʽѡ��==========================================
void mode_choice()
{
		if(data.start == 1 && data.mode == 2) //�����ܿ���
	{
    if(data.function == 1)
		{
			
			//catch_the_object();
		}
		else if(data.function == 2)
		{
			
		}
		else
		{
			
		}
	}
	else
	{
		
	}
}
//======================ģʽ1 �Զ���ȡ����=========================
void catch_the_object()
{
		//��ʾʶ��ɹ����������յ��˸�����xyzֵ����ʼ�켣�滮
	if(data.start == 1)
	{
		//��һ��·��
		state_check = 1;
		path_plan(data.x,data.y,data.z,0,10);
		//��ȡĿ��
		servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
    state_check = 2;
		//�ص�Ŀ���
		back_to_inital();
		state_check = 3;
		//�ڶ���·��
		path_plan(-300,200,10,0,10);
    state_check = 4;
		//�������
		servos_control(servo_info[1].target_angle,servo_info[1].channel_id);

		//��ɺ󷵻�ԭλ��
		back_to_inital();

		
		//�˴�Ӧ������һ����־λ
	}
}

////=====================�켣�滮=========================
//-------------------ִ����������ָ��λ��-------------------
void path_plan(float x,float y,float z,float time_start,float time_final)
{
	//��������ʱ��
	
	first_path.t_s = time_start;
	first_path.t_f = time_final;
	
	//���Ŀ��Ƕ�
	target_T_get(x,y,z,T_target);
  IK_calc(T_target,T_res);
	
	for(uint16_t i =0;i<4;i++)
	{
	  first_path.end.theta[i] = T_res[i];
	}
	servo_info[0].target_angle = T_res[4];
	//��ǰ�Ƕ�ֵ
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;

		
	//���ɹ켣���ҿ�ʼִ�п���
	path_planning(&first_path);
}


//-------------------�ص���ʼλ��-------------------
void back_to_inital()
{
	//��������ʱ��
	first_path.t_s = 0;
	first_path.t_f = 3;
	
	//���Ŀ��Ƕ�	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = inital_pos[i];
	}
	servo_info[0].target_angle =inital_pos[4];
	//��ǰ�Ƕ�
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;

	
	//���ɹ켣���ҿ�ʼִ�п���
	path_planning(&first_path);
}

//-------------------��ȫ����ģʽλ��-------------------
void back_to_exit()
{
	//��������ʱ��
	first_path.t_s = 0;
	first_path.t_f = 5;
	
	//���Ŀ��Ƕ�	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = exit_pos[i];
	}
	servo_info[0].target_angle = 0;
	//��ǰ�Ƕ�
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;
	
	//���ɹ켣���ҿ�ʼִ�п���
	path_planning(&first_path);
	
	
}


 