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
//电机初始化
static void motor_init();
//模式选择
static void mode_chose();
//模式0
static void motor_stop_and_keep();
//模式1
static void motor_remote_control();
//模式2
static void motor_control_kinemat();
//模式3
static void mode_3();
//控制数据处理
static void data_operate();
//控制电机
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

//================================数据初始化==========================================
//电机初始化
static void motor_init()
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
	motor_info[0].mode = 1;  
	motor_info[1].mode = 1; 
	motor_info[2].mode = 1; 
	motor_info[3].mode = 1; 
	
	//设置电机id
	motor_info[0].can_id = 0x01; 
	motor_info[1].can_id = 0x02; 
	motor_info[2].can_id = 0x03;
	motor_info[3].can_id = 0x04;


  //延时等待电机使能
	motor_commend(motor_info[0],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[1],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[2],Data_Enable);
	HAL_Delay(2000);
	motor_commend(motor_info[3],Data_Enable);
	HAL_Delay(2000);
	
}

//================================模式选择==========================================
void mode_chose()
{
	if(data.start == 1)//控制总开关
	{
			if(data.mode == 1)//模式选择
					{
						motor_remote_control(); //操作板控制模式
					}
			else if(data.mode == 2)
					{
						motor_control_kinemat();//全自动路径规划模式
					}
			else if(data.mode == 3)
					{
						servos_reset();         //重置舵机位置
					}
			else
			{
				motor_stop_and_keep();//停止并保持当前位置
			}
	}
	else
	{
		motor_stop_and_keep();//停止并保持当前位置
	}

}
//================================模式0 保持静止====================================
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
//================================模式1 手动控制====================================
void motor_remote_control()
{
   
	//数据进行处理
	for(uint8_t i = 0;i<4;i++)
	{
		//通道值处理
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
		
		//角度越界处理
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
//================================模式3 自动控制====================================
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

//==================================电机控制信息发送====================================
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

//==================================控制模式====================================
//mit控制模式
/*
	对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控
	根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des
	即可实现匀速转动;kp=0,kd=0，给定 t_ff 即可实现给定mt扭矩输出
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














