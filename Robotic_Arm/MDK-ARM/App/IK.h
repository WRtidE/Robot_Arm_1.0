#ifndef __IK_H
#define __IK_H

//机械臂初始化
typedef __packed struct
{
  //DH表
	int a[5];
	int d[5];
	
	//关节角度限制
	float joint[5]; //五个关节的角度限制，主要限制前三个关节即可
	
	//theta
}manipulator;

extern manipulator arm;
 
void target_T_get(float x,float y,float z,float (*T)[4]);
void IK_calc(float (*T)[4],float *res);
	
#endif
