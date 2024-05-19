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

//設定上電後的標準位置以及結束時安全掉電的位置
point   reset_point;
point disable_point;
point  via_point;

path    first_path;
path     last_path;
//函数定义
void points_init();

void path_init();

void kinemat_task(void const * argument)
{
   points_init();
	 path_init();
	 path_planning(&reset_point,&via_point,&first_path);
	 path_planning(&via_point,&disable_point,&last_path);
	 while(1)
	{
		
	}
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


 