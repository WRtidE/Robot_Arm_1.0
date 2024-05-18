#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

typedef __packed struct
{
  //��ʼλ�ú�����λ��
	float theta_s;
	float theta_f;
	//��ʼ�ٶ������ٶ�
	float vel_s;
	float vel_f;
	//��ʼ���ٶȺ����ռ��ٶ�
	float acc_s;
	float acc_f;
	//��ʼʱ�������ʱ��
	float t_s;
	float t_f;
		
}pos_data;

typedef __packed struct
{
  //�������ĽǶȡ��ٶȡ����ٶ�·���滮 
  float theta[100];
	float vel[100];
	float acc[100];
	
}path_message;


void a_path_cala(pos_data *joint,path_message *path);
#endif
