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

//================================变量定义================================
//储存几个重要pos
float inital_pos[5] = {0,0,0,2,0}; //初始位置
float   exit_pos[5] = {0,-1.7,1,0.9,0};  //安全掉电位置


float martx[4][4];
float T_target[4][4];
float T_res[5];
float T_test[5];

//创建几条轨迹，用来储存轨迹信息
path first_path;
path second_path;

float state_check = 0;
static float pi=3.1415;
//================================函数声明================================
static void arm_init();

//模式选择
static void mode_choice();
static void catch_the_object();
static void back_to_exit();
static void back_to_inital();

//轨迹规划函数
static void path_plan(float x,float y,float z,float time_start,float time_final);

//================================主程序==================================
void kinemat_task(void const * argument)
{
   arm_init();

	for(;;)
		{		 
	  state_check =0;
		mode_choice();
		osDelay(1);
	 }

}
//================================数据初始化===============================
void arm_init()
{
	//建立DH表
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
//================================模式选择==========================================
void mode_choice()
{
		if(data.start == 1 && data.mode == 2) //控制总开关
	{
    if(data.function == 1)
		{
			catch_the_object();
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
//======================模式1 自动夹取物体=========================
void catch_the_object()
{
		//表示识别成功，或者是收到了给定的xyz值，则开始轨迹规划
	if(data.start == 1)
	{
		//第一段路径
		state_check = 1;
		path_plan(data.x,data.y,data.z,0,10);
		//夹取目标
		servos_control(servo_info[1].target_angle,servo_info[1].channel_id);
    state_check = 2;
		//回到目标点
		back_to_inital();
		state_check = 3;
		//第二段路径
		path_plan(-300,200,10,0,10);
    state_check = 4;
		//放下物块
		servos_control(servo_info[1].target_angle,servo_info[1].channel_id);

		//完成后返回原位置
		back_to_inital();

		
		//此处应该设置一个标志位
	}
}

////=====================轨迹规划=========================
//-------------------执行器运行至指定位置-------------------
void path_plan(float x,float y,float z,float time_start,float time_final)
{
	//设置启动时间
	
	first_path.t_s = time_start;
	first_path.t_f = time_final;
	
	//获得目标角度
	target_T_get(x,y,z,T_target);
  IK_calc(T_target,T_res);
	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = T_res[i];
	}
	
	//当前角度值
//	first_path.start.theta[0] = motor_info[0].position;
//	first_path.start.theta[1] = motor_info[1].position;
//	first_path.start.theta[2] = motor_info[2].position;
//	first_path.start.theta[3] = motor_info[3].position - pi/2;
//	first_path.start.theta[4] = servo_info[0].position;
	
	first_path.start.theta[0] = first_path.joint_path[0].theta[99];
	first_path.start.theta[1] = first_path.joint_path[1].theta[99];
	first_path.start.theta[2] = first_path.joint_path[2].theta[99];
	first_path.start.theta[3] = first_path.joint_path[3].theta[99];
	first_path.start.theta[4] = first_path.joint_path[4].theta[99];
	
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}


//-------------------回到初始位置-------------------
void back_to_inital()
{
	//设置启动时间
	first_path.t_s = 0;
	first_path.t_f = 10;
	
	//获得目标角度	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = inital_pos[i];
	}
	
	//当前角度
//	first_path.start.theta[0] = motor_info[0].position;
//	first_path.start.theta[1] = motor_info[1].position;
//	first_path.start.theta[2] = motor_info[2].position;
//	first_path.start.theta[3] = motor_info[3].position - pi/2;
//	first_path.start.theta[4] = servo_info[0].position;
	
	first_path.start.theta[0] = first_path.joint_path[0].theta[99];
	first_path.start.theta[1] = first_path.joint_path[1].theta[99];
	first_path.start.theta[2] = first_path.joint_path[2].theta[99];
	first_path.start.theta[3] = first_path.joint_path[3].theta[99];
	first_path.start.theta[4] = first_path.joint_path[4].theta[99];
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}

//-------------------安全掉电模式位置-------------------
void back_to_exit()
{
	//设置启动时间
	first_path.t_s = 0;
	first_path.t_f = 10;
	
	//获得目标角度	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = exit_pos[i];
	}
	
	//当前角度
//	first_path.start.theta[0] = motor_info[0].position;
//	first_path.start.theta[1] = motor_info[1].position;
//	first_path.start.theta[2] = motor_info[2].position;
//	first_path.start.theta[3] = motor_info[3].position - pi/2;
//	first_path.start.theta[4] = servo_info[0].position;
	first_path.start.theta[0] = first_path.joint_path[0].theta[99];
	first_path.start.theta[1] = first_path.joint_path[1].theta[99];
	first_path.start.theta[2] = first_path.joint_path[2].theta[99];
	first_path.start.theta[3] = first_path.joint_path[3].theta[99];
	first_path.start.theta[4] = first_path.joint_path[4].theta[99];
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}


 