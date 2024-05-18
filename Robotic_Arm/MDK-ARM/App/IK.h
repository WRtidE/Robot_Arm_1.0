#ifndef __IK_H
#define __IK_H

//机械臂初始化DH表
typedef __packed struct
{
  //DH表
	int a[5];
	int d[5];
	
	//theta
}manipulator;


//T矩阵结构体
typedef __packed struct
{
   float line1[4];
	 float line2[4];
	 float line3[4];
}Matrix_4;


//T矩阵结构体
typedef __packed struct
{
   float res1[5];
	 float res2[5];
	 float res3[5];
	 float res4[5]; 
}Matrix_ans;

#endif
