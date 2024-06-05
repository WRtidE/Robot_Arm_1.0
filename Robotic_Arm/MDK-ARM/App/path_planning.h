#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

//�������������
typedef __packed struct
{
  //�������ĽǶȡ��ٶȡ����ٶ�·���滮 
  float theta[100];
	float   vel[100];
	float   acc[100];
	
}path_message;
//һ���������������5���ؽڵĽǶȡ��ٶȺͼ��ٶ�����
//Ĭ��Ϊ��ʼ��λ��
typedef __packed struct
{
	float theta[5];
	float vel[5];
	float acc[5];
}point;

//һ��·��������
typedef __packed struct
{
	//��ʼʱ�������ʱ��
	float t_s;
	float t_f;
	//ÿ�������·���滮����
  path_message joint_path[5];
	point start;
	point end;
}path;



void path_planning(path *path);
#endif
