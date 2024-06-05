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


//設定上電後的標準位置以及結束時安全掉電的位置




float martx[4][4];
float T_target[4][4];
float T_res[5];
float T_test[5];
path first_path;
path second_path;

//函数定义
static void arm_init();

//模式选择
static void mode_choice();

//轨迹规划函数
void planning_regular();


void kinemat_task(void const * argument)
{
   arm_init();

	 while(1)
	{
		mode_choice();
    //planning_regular();
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
						planning_regular();
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
void planning_regular()
{
	//设置启动时间
	
	first_path.t_s = 0;
	first_path.t_f = 10;
	
	//获得目标角度
	target_T_get(-155,-269,10,T_target);
  IK_calc(T_target,T_res);
	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = T_res[i];
	}
	
	//当前角度值
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.start.theta[i] = 0;
	}
	
	//生成轨迹并且开始执行控制
	path_planning(&first_path);
}





 