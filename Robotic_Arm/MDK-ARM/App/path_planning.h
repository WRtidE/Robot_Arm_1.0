#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

typedef __packed struct
{
  //初始位置和最终位置
	float theta_s;
	float theta_f;
	//初始速度最终速度
	float vel_s;
	float vel_f;
	//初始加速度和最终加速度
	float acc_s;
	float acc_f;
	//初始时间和最终时间
	float t_s;
	float t_f;
		
}pos_data;

typedef __packed struct
{
  //储存电机的角度、速度、加速度路径规划 
  float theta[100];
	float vel[100];
	float acc[100];
	
}path_message;


void a_path_cala(pos_data *joint,path_message *path);
#endif
