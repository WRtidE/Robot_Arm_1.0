#ifndef __PATH_PLANNING_H
#define __PATH_PLANNING_H

//单个电机的数据
typedef __packed struct
{
  //储存电机的角度、速度、加速度路径规划 
  float theta[100];
	float   vel[100];
	float   acc[100];
	
}path_message;
//一个点而已啦，包含5个关节的角度、速度和加速度数据
//默认为起始点位置
typedef __packed struct
{
	float theta[5];
	float vel[5];
	float acc[5];
}point;

//一条路径的数据
typedef __packed struct
{
	//初始时间和最终时间
	float t_s;
	float t_f;
	//每个电机的路径规划数据
  path_message joint_path[5];
	point start;
	point end;
}path;



void path_planning(path *path);
#endif
