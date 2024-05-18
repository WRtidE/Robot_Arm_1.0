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
//电机初始化
void motor_init();
//控制数据处理
void data_operate();
//mit控制模式
void mit_contorl();
void mit_rc_contorl();

//速度位置模式控制
void vp_control();

//速度控制模式
void speed_control(); 

//轨迹规划（后续会考虑整个task）
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
//================================电机控制==========================================================================
//电机初始化
void motor_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);       //开启舵机电源
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,  GPIO_PIN_RESET);     //点个灯
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     //开启电源1输出 右边
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_SET);   //开启电源2输出 左边
	
	HAL_Delay(1000);	//延时1s为了给DM_MC01板可控电源电源1稳定启动
	
	motor_info[0].hcan =  hcan2;
	motor_info[1].hcan =  hcan1;
	motor_info[2].hcan =  hcan1;
	motor_info[3].hcan =  hcan1;
	
	//电机模式设置:为0为IMT模式，为1为位置速度模式，为2为速度模式
	motor_info[0].mode = 2;  
	motor_info[1].mode = 2; 
	motor_info[2].mode = 2; 
	motor_info[3].mode = 2; 
	
	//设置电机id
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
//数据处理
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


//mit控制模式
/*
	对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控
	根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des
	即可实现匀速转动;kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出
*/
void mit_contorl()
{

	MIT_CtrlMotor(&hcan1,0x02, motor_info[1].target_angle,
														 motor_info[1].target_speed,
														 motor_info[1].Kp,
														 motor_info[1].Kd, 
														 motor_info[1].target_T_ff);
}


//速度位置模式控制
void vp_control()
{
	PosSpeed_CtrlMotor(&hcan1,motor_info[0].can_id, motor_info[0].target_angle, motor_info[0].target_speed);
}

//================================轨迹规划===========================================================================
void path_planning()
{
	
	//规划路径1
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
	
	//规划路径2
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

//将轨迹规划参数传入joint中
 void joint_init(pos_data *joint)
 {
	  //在这里应该会进行一步逆解
	  motor_info[1].joint.theta_s = 90;
		motor_info[1].joint.theta_f = 45;
		motor_info[1].joint.vel_s   =  0;
		motor_info[1].joint.vel_f   =  0;
		motor_info[1].joint.acc_s   =  0;
		motor_info[1].joint.acc_f   =  0;
		motor_info[1].joint.t_s     = 10;
		motor_info[1].joint.t_f     = 20;
 }

