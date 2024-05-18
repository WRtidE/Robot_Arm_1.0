#include "stm32f4xx.h" 
#include "path_planning.h"
 #include "cmsis_os.h"


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

void a_path_cala(pos_data *joint,path_message *path)
{
	float q0   = joint->theta_s;
	float qf   = joint->theta_f;
  float dq0  = joint->vel_s;
  float dqf  = joint->vel_f;
	float ddq0 = joint->acc_s;
  float ddqf = joint->acc_f;
	float ts   = joint->t_s;
	float tf   = joint->t_f;
	
	
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
			 path->theta[index] =  a0+a1*(t-ts)+a2*power((t-ts),2)+a3*power((t-ts),3)+a4*power((t-ts),4)+a5*power((t-ts),5);
		   //速度計算
		   path->vel[index]   =  a1+2*a2*(t-ts)+3*a3*power((t-ts),2)+4*a4*power((t-ts),3)+5*a5*power((t-ts),4);
			 //加速度计算
		   path->acc[index]   =  2*a2+6*a3*(t-ts)+12*a4*power((t-ts),2)+20*a5*power((t-ts),3);
		   index++;
	}		

}