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
manipulator arm;

point   reset_point;
point disable_point;
point  via_point;

path    first_path;
path     last_path;

Matrix_T T_start;
Matrix_T T_end;
Matrix_ans T_res_start;
Matrix_ans T_res_end;

//函数定义
void points_init();

void path_init();

void T_init();

void arm_init();

void kinemat_task(void const * argument)
{

   points_init();
	 path_init();
	 T_init();
	 arm_init();
	 IK_calc(&arm,&T_end,&T_res_end);
	 T_res_start = T_res_end;

	 while(1)
	{
		//IK_calc(&arm,&T_end,&T_res_end);
	}
	path_planning(&reset_point,&via_point,&first_path);
	 path_planning(&via_point,&disable_point,&last_path);
}
//初始化矩阵
/*
        -1         0         0      -250
         0         1         0         0
         0         0        -1       197
         0         0         0         1


   -0.2224    0.9727    0.0664   34.2267
   -0.8114   -0.2224    0.5406  278.7539
    0.5406    0.0664    0.8387  438.8897
         0         0         0    1.0000
*/
void T_init()
{
//	T_end.line1[0]=-1;T_end.line1[1]= 0;T_end.line1[2]= 0;T_end.line1[3]=-250;
//	T_end.line2[0]= 0;T_end.line2[1]= 1;T_end.line2[2]= 0;T_end.line2[3]= 0;
//	T_end.line3[0]= 0;T_end.line3[1]= 0;T_end.line3[2]=-1;T_end.line3[3]= 197;
//	T_end.line4[0]= 0;T_end.line4[1]= 0;T_end.line4[2]= 0;T_end.line4[3]= 1;
	
	T_end.line1[0]= 0.1219;T_end.line1[1]= -0.9925;T_end.line1[2]=       0;T_end.line1[3]=    0;
	T_end.line2[0]= 0.8324;T_end.line2[1]=  0.1022;T_end.line2[2]= -0.5446;T_end.line2[3]= -280;
	T_end.line3[0]= 0.5406;T_end.line3[1]=  0.0664;T_end.line3[2]=  0.8387;T_end.line3[3]=  438.8897;
	T_end.line4[0]=      0;T_end.line4[1]=       0;T_end.line4[2]=       0;T_end.line4[3]=    1;
}

void arm_init()
{
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
//设定基础点
void points_init()
{
   disable_point.theta[0] = -90;
	 disable_point.theta[1] = 20;
	 disable_point.theta[2] = 40;
	 disable_point.theta[3] = 50;
	 disable_point.theta[4] = 90;
	
	 via_point.theta[0] = 50;
	 via_point.theta[1] = 40;
	 via_point.theta[2] = 20;
	 via_point.theta[3] = 10;
	 via_point.theta[4] = 10;
	
	
}	

//设定路径
void path_init()
{
	first_path.t_f = 10;
	first_path.t_s = 0;
	
	last_path.t_s =10;
	last_path.t_f =20;
}


 