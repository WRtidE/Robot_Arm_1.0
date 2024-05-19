#include "stm32f4xx.h" 
#include "path_planning.h"
#include "cmsis_os.h"
#include "Can_user.h"


//===============================计算N次方==============================================================
//計算n次方
float power(float num,uint16_t n)
{
	float res = 1;
	for(uint16_t i =0;i<n;i++)
	{
		res = res * num;
	}	
	
	return res;
}


//===============================直接传入两个点，生成这两个点之间所有关节相对路径==============================================================
void one_path_cala(point *points,point *pointf,path *path)
{
  float ts   = path->t_s;
	float tf   = path->t_f;
	
	for(uint16_t i = 0;i<5;i++)
	{
		float q0   = points->theta[i];
		float qf   = pointf->theta[i];
		float dq0  = points->vel[i];
		float dqf  = pointf->vel[i];
		float ddq0 = points->acc[i];
		float ddqf = pointf->acc[i];

		//计算轨迹五次多项式系数值
		float a0,a1,a2,a3,a4,a5 = 0;
		a0 = q0;
		a1 = dq0;
		a2 = ddq0/2;
		a3 = (20*(qf-q0)-( 8*dqf + 12*dq0)*(tf-ts) - (3*ddq0 -   ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts));
		a4 = (30*(q0-qf)+(14*dqf + 16*dq0)*(tf-ts) + (3*ddq0 - 2*ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts));
		a5 = (12*(qf-q0)-( 6*dqf +  6*dq0)*(tf-ts) - (  ddq0 -   ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts));
		
		uint16_t index = 0;
		//算出轨迹角度、速度和加速度
		for(float t = ts; t < tf ; t+=(tf-ts)/100)
		{
				 //角度計算
				 path->joint_path[i].theta[index] =  a0+a1*(t-ts)+a2*power((t-ts),2)+a3*power((t-ts),3)+a4*power((t-ts),4)+a5*power((t-ts),5);
				 //速度計算
				 path->joint_path[i].vel[index]   =  a1+2*a2*(t-ts)+3*a3*power((t-ts),2)+4*a4*power((t-ts),3)+5*a5*power((t-ts),4);
				 //加速度计算
				 path->joint_path[i].acc[index]   =  2*a2+6*a3*(t-ts)+12*a4*power((t-ts),2)+20*a5*power((t-ts),3);
				 index++;
		}		
	}
	
}


//===============================根据规划的路径，改变关节的target_angle==============================================================
void path_planning(point *points,point *pointf,path *path)
 {
	 //创建一条路径
  one_path_cala(points,pointf,path);
	 
	for(int index =0;index<100;index++)
	{
		for(uint16_t id = 0;id<5;id++)
		{
			motor_info[id].target_angle = path->joint_path[id].theta[index];
			motor_info[id].target_speed = path->joint_path[id].vel[index];
			motor_info[id].target_T_ff  = path->joint_path[id].acc[index];
		}	
		float delay_time = (path->t_f - path->t_s)*10;
		HAL_Delay(delay_time);
	}
 }

 
