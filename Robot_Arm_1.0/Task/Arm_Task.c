#include "stm32f4xx.h"                  // Device header
#include "Arm_Task.h" 
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"
#include "bsp_can.h"
#include "Can_user.h"
#include "remote_control.h"
#include "Uart_user.h"

//电机初始化
void motor_init();

//mit控制模式
void mit_contorl();
void mit_rc_contorl();

//速度位置模式控制
void vp_control();

//速度控制模式
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


//电机初始化
void motor_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);     //开启舵机电源
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,  GPIO_PIN_RESET);     //点个灯
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);     //开启电源1输出 右边
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_RESET);   //开启电源2输出 左边
	
	HAL_Delay(1000);	//延时1s为了给DM_MC01板可控电源电源1稳定启动
	
	//电机模式设置:为0为IMT模式，为1为位置速度模式，为2为速度模式
	motor_info[0].mode = 0;  
	motor_info[1].mode = 0; 
	motor_info[2].mode = 2; 
	
	//设置电机id
	motor_info[0].can_id = 0x01; 
	motor_info[1].can_id = 0x02; 
	motor_info[2].can_id = 0x03;
	motor_info[2].can_id = 0x04;

	//电机pid 直接在串口调试助手
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





