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

static float pi=3.1415;
//================================函数声明================================
static void arm_init();

//模式选择
static void mode_choice();
static void back_to_exit();
static void back_to_inital();
//轨迹规划函数
static void path_plan(float x,float y,float z,float time_start,float time_final);

//================================主程序================================
void kinemat_task(void const * argument)
{
   arm_init();

	 while(1)
	{
		mode_choice();
		osDelay(1);
	}

}
//================================数据初始化==========================================
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
		if(data.start == 1)//控制总开关
	{
			if(data.mode == 1)//模式选择
					{
						
					}
					else if(data.mode == 2)
					{
            path_plan(-155,-296,10,0,10);
					}
					else if(data.mode == 3)
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
////================================轨迹规划//================================
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
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = motor_info[3].position - pi/2;
	first_path.start.theta[4] = motor_info[4].position;
	
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
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = motor_info[3].position - pi/2;
	first_path.start.theta[4] = motor_info[4].position;
	
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
	first_path.start.theta[0] = motor_info[0].position;
	first_path.start.theta[1] = motor_info[1].position;
	first_path.start.theta[2] = motor_info[2].position;
	first_path.start.theta[3] = motor_info[3].position - pi/2;
	first_path.start.theta[4] = motor_info[4].position;
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}


 