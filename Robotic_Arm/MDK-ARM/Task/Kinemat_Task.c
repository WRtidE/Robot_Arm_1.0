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
#include "usart.h"

float stest[3];
//================================变量定义================================
//储存几个重要pos
float inital_pos[5] = {0,0,0,1.5708,0}; //初始位置
float   exit_pos[5] = {0,-1.8,1.22,0.73,0};  //安全掉电位置

float finish_flag = 1;
float martx[4][4];
float T_target[4][4];
float T_res[5];
float T_test[5];

//x,y,z
float x,y,z = 0;
//创建几条轨迹，用来储存轨迹信息
path first_path;
path second_path;
//用来判断是第几个字母
float letter_last = 0;
float letter_first= 0;

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
				else if(data.mode == 3 && finish_flag == 0)
				{
					catch_the_object();
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
//======================模式1 自动夹取物体=========================
void catch_the_object()
{

	//表示识别成功，或者是收到了给定的xyz值，则开始轨迹规划
  //获得xyz的值
	state_check = 0;
	if(data.start == 1 && data.kind !=0)
	{
		x = data.x;
		y = data.y;   
		
		state_check = 1;
		//立即关掉中断
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE); //关中断
	}
	if(data.start == 1 && state_check == 1)
	{
		data.catch_flag = 0;
    osDelay(3000);
	 //完成后返回原位置
		back_to_inital();
		//第一段路径
		path_plan(x,y,10,0,3);
		//夹取目标
		data.catch_flag = 1;
    osDelay(3000);
		//回到目标点
		back_to_inital();
		
		//第二段路径,将物块放置在
		path_plan(-150 - 50*data.kind,200,10,0,3);

		//放下物块
    data.catch_flag = 0;
    osDelay(3000);
		//完成后返回原位置
		back_to_inital();

		if(data.kind == 3)
		{
				state_check = 0;
		}
		//此处应该设置一个标志位
	}
	 __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE); //开中断
	data_receive();
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
	
	for(uint16_t i =0;i<4;i++)
	{
	  first_path.end.theta[i] = T_res[i];
	}

	servo_info[0].target_angle = T_res[4];

	//当前角度值
	first_path.start.theta[0] =  motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] =  motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;

		
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}


//-------------------回到初始位置-------------------
void back_to_inital()
{
	//设置启动时间
	first_path.t_s = 0;
	first_path.t_f = 3;
	
	//获得目标角度	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = inital_pos[i];
	}
	servo_info[0].target_angle =inital_pos[4];
	//当前角度
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;

	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}

//-------------------安全掉电模式位置-------------------
void back_to_exit()
{
	//设置启动时间
	first_path.t_s = 0;
	first_path.t_f = 5;
	
	//获得目标角度	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = exit_pos[i];
	}
	servo_info[0].target_angle = 0;
	//当前角度
	first_path.start.theta[0] =  motor_info[0].position;
	first_path.start.theta[1] = -motor_info[1].position;
	first_path.start.theta[2] =  motor_info[2].position;
	first_path.start.theta[3] = -motor_info[3].position;
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
	
	
}


 