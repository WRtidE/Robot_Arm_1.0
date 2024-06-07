#include "stm32f4xx.h" 
#include "path_planning.h"
#include "cmsis_os.h"
#include "Can_user.h"
static float pi=3.1415;
//===============================����N�η�==============================================================
//Ӌ��n�η�
float power(float num,uint16_t n)
{
	float res = 1;
	for(uint16_t i =0;i<n;i++)
	{
		res = res * num;
	}	
	
	return res;
}


//===============================ֱ�Ӵ��������㣬������������֮�����йؽ����·��==============================================================
//��Ҫ����������ĽǶ���Ϣ�������켣��ʱ����Ϣ������������
void one_path_cala(path *path)
{
  float ts   = path->t_s;
	float tf   = path->t_f;
	
	for(uint16_t i = 0;i<5;i++)
	{
		float q0   = path->start.theta[i];
		float qf   = path->end.theta[i];
		float dq0  = path->start.vel[i];
		float dqf  = path->end.vel[i];
		float ddq0 = path->start.acc[i];
		float ddqf = path->end.acc[i];

		//����켣��ζ���ʽϵ��ֵ
		float a0,a1,a2,a3,a4,a5 = 0;
		a0 = q0;
		a1 = dq0;
		a2 = ddq0/2;
		a3 = (20*(qf-q0)-( 8*dqf + 12*dq0)*(tf-ts) - (3*ddq0 -   ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts));
		a4 = (30*(q0-qf)+(14*dqf + 16*dq0)*(tf-ts) + (3*ddq0 - 2*ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts));
		a5 = (12*(qf-q0)-( 6*dqf +  6*dq0)*(tf-ts) - (  ddq0 -   ddqf)*(tf-ts)*(tf-ts))/(2*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts)*(tf-ts));
		
		uint16_t index = 0;
		//����켣�Ƕȡ��ٶȺͼ��ٶ�
		for(float t = ts; t < tf ; t+=(tf-ts)/100)
		{
				 //�Ƕ�Ӌ��
				 path->joint_path[i].theta[index] =  a0+a1*(t-ts)+a2*power((t-ts),2)+a3*power((t-ts),3)+a4*power((t-ts),4)+a5*power((t-ts),5);
				 //�ٶ�Ӌ��
				 path->joint_path[i].vel[index]   =  a1+2*a2*(t-ts)+3*a3*power((t-ts),2)+4*a4*power((t-ts),3)+5*a5*power((t-ts),4);
				 //���ٶȼ���
				 path->joint_path[i].acc[index]   =  2*a2+6*a3*(t-ts)+12*a4*power((t-ts),2)+20*a5*power((t-ts),3);
				 index++;
		}		
	}
}


//===============================���ݹ滮��·�����ı�ؽڵ�target_angle==============================================================
void path_planning(path *path)
 {
	 //����һ��·�����ȼ���
  one_path_cala(path);
	 
	 //�������ִ��·���滮
	for(int index =0;index<100;index++)
	{
		//�ԽǶ�ֵ���д���������Ƕ���MATLAB�����еĽǶ�һһ��Ӧ
		//��Ŀ��Ƕ�ֵ�������		
		motor_info[0].target_angle =  path->joint_path[0].theta[index];
		motor_info[1].target_angle = -path->joint_path[1].theta[index];
		motor_info[2].target_angle =  path->joint_path[2].theta[index];
		motor_info[3].target_angle = -path->joint_path[3].theta[index]+pi/2;
		motor_info[4].target_angle =  path->joint_path[4].theta[index];
		
		float delay_time = (path->t_f - path->t_s)*10;
		osDelay(delay_time);
	}
 }

 
