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


//�O�������Ę˜�λ���Լ��Y���r��ȫ��늵�λ��




float martx[4][4];
float T_target[4][4];
float T_res[5];
float T_test[5];
path first_path;
path second_path;

//��������
void arm_init();
void planning_regular();
	
void kinemat_task(void const * argument)
{
   arm_init();

	 while(1)
	{
    planning_regular();
		//target_T_get(-155,-269,10,T_target);
    //IK_calc(T_target,T_res);
		osDelay(1);
	}

}

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

void planning_regular()
{
	//��������ʱ��
	
	first_path.t_s = 0;
	first_path.t_f = 10;
	
	//���Ŀ��Ƕ�
	target_T_get(-155,-269,10,T_target);
  IK_calc(T_target,T_res);
	
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.end.theta[i] = T_res[i];
	}
	
	//��ǰ�Ƕ�ֵ
	for(uint16_t i =0;i<5;i++)
	{
	  first_path.start.theta[i] = T_res[i];
	}
	
  //	
	path_planning(&first_path);
}





 