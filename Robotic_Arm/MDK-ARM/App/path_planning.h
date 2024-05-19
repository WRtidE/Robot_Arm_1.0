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

//һ��·��������
typedef __packed struct
{
	//��ʼʱ�������ʱ��
	float t_s;
	float t_f;
	//ÿ�������·���滮����
  path_message joint_path[5];
}path;


//һ���������������5���ؽڵĽǶȡ��ٶȺͼ��ٶ�����
//Ĭ��Ϊ��ʼ��λ��
typedef __packed struct
{
	float theta[5];
	float vel[5];
	float acc[5];
}point;


void one_path_cala(point *points,point *pointf,path *path);
void path_planning(point *points,point *pointf,path *path);
#endif
