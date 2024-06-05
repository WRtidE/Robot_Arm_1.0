#include "stm32f4xx.h" 
#include "IK.h"
#include <math.h>

float t[100];
float pi = 3.1416;
manipulator arm;

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))


//================================arctan数据处理函数================================
float arctan(float sa,float ca)
{
	float eps = 0.0001;
	float angle = 0;
	
	//abs(cos) = 0 和abs(sin) = 0 的时候，让theta = 0；
	if((fabsf(sa)<eps) && (fabsf(ca)<eps))
	{
		return angle;
	}
	//abs(cos) = 0 的时候，让theta = 90 *（sin的符号）
	if(fabsf(ca) < eps)
	{
		if(sa>0)
		{
			angle =   pi/2;
		}
		if(sa<0)
		{
			angle = - pi/2;
		}
		return angle;
	}
	else if(fabsf(sa) < eps)
	{
		
	}
	else
	{
//	  float ca = fabsf(ca), sa = fabsf(sa);
//    float temp1 = min(ca, sa)/max(ca, sa);
//    float temp2 = temp1*temp1;
//    float result = ((-0.0464964749 * temp2 + 0.15931422) * temp2 - 0.327622764) * temp2 * temp1 + temp1;
		
		angle = atan2f(sa,ca);
	}
	
	return angle;
	
	//abs(cos = 0 的时候，让theta = 0   
}

float Tans[4][5];
//================================机械臂逆解函数================================
void IK_calc(float (*T)[4],float *res)
{
	//Tans 储存获得的4个解
	
	
	//机械臂参数
  uint16_t a2 = arm.a[1]; uint16_t a3 = arm.a[2];uint16_t a4 = arm.a[3];
	uint16_t d1 = arm.d[0]; uint16_t d5 = arm.d[4];
	
  //T06矩阵
   float r11 = T[0][0];float r12 = T[0][1];float r13 = T[0][2];float px = T[0][3];
   float r21 = T[1][0];float r22 = T[1][1];float r23 = T[1][2];float py = T[1][3];
   float r31 = T[2][0];float r32 = T[2][1];float r33 = T[2][2];float pz = T[2][3];
	
	
	 //储存theta解
	 float theta1[4]={0};
	 float theta2[4]={0};
   float theta3[4]={0};
	 float theta4[4]={0};
	 float theta5[4]={0};
	 

	 
	 
	 //================================theta1================================
	 theta1[0] = arctan(py,px);
	 theta1[1] = arctan(-py,-px);
	 
	 theta1[2] = theta1[0];
	 theta1[3] = theta1[1];

   //================================theta5================================
	 for(uint16_t i =0;i<2;i++)
	 {
		 theta5[i] = 1;
		 //theta5[i] =  arctan(r21*cos(theta1[i])-r11*sin(theta1[i]),r22*cos(theta1[i])-r12*sin(theta1[i]));
	 }
	 
	 theta5[2] = theta5[0];
	 theta5[3] = theta5[1];
	 //================================theta3 ================================
	 for(uint16_t i =0;i<2;i++)
	 {
		 float x = px*cos(theta1[i]) - d5*(r13*cos(theta1[i]) + r23*sin(theta1[i])) + py*sin(theta1[i]);
		 float y = pz - d1 - d5*r33;
		 
		 if( ((x*x+y*y-a4*a4-a3*a3)/(2*a3*a4))>1 || ((x*x+y*y-a4*a4-a3*a3)/(2*a3*a4))<-1)
		 {
			   theta3[i]   = 255; //表示无解
			   theta3[i+2] = 255; //表示无解
		 }
		 else
		 {
			   theta3[i]   = asin(-(x*x+y*y-a4*a4-a3*a3)/(2*a3*a4));
			   theta3[i+2] = pi - asin(-(x*x+y*y-a4*a4-a3*a3)/(2*a3*a4));
		 }
	 }
	  

	 //判断角度是否越界
	 for(uint16_t i=0;i<4;i++)
	 {
		 if(theta3[i]>3.14)
		 {
				theta3[i] = theta3[i] - 2 * pi;
		 }
		 else if(theta3[i]<-3.14)
		 {
				theta3[i] = theta3[i] + 2 * pi;
		 }
	 }
	 
	
	//================================theta2================================
	 for(uint16_t i=0;i<4;i++)
	 {
		 if(theta3[i]<25)
		 {			
			float s3 = sin(theta3[i]);
			float c3 = cos(theta3[i]);
			float s1 = sin(theta1[i]);
			float c1 = cos(theta1[i]);
			 
			float c2 =  - ((a3 - a4*s3)*(d1 - pz + d5*r33))/(a4*a4*c3*c3 + a4*a4*s3*s3 + a3*a3 - 2*a3*a4*s3) - (a4*c3*(px*c1 - d5*(r13*c1 + r23*s1) + py*s1))/(a4*a4*c3*c3 + a4*a4*s3*s3 + a3*a3 - 2*a3*a4*s3);
      float s2 =    (a4*c3*(d1 - pz + d5*r33))/(a4*a4*c3*c3 + a4*a4*s3*s3 + a3*a3 - 2*a3*a4*s3) - ((a3 - a4*s3)*(px*c1 - d5*(r13*c1 + r23*s1) + py*s1))/(a4*a4*c3*c3 + a4*a4*s3*s3 + a3*a3 - 2*a3*a4*s3);
			
			if(s2>1||s2<-1||c2>1||c2<-1)
			{
				theta2[i] = 255;
			}	
			else
			{
				theta2[i] = arctan(s2,c2); 
			}
		 }
		 else
		 {
			 theta2[i]= 255;
		 }
	 }
	 
	 //判断角度是否越界
	 for(uint16_t i=0;i<4;i++)
	 {
		 if(theta2[i]>3.14)
		 {
				theta2[i] = theta2[i] - 2 * pi;
		 }
		 else if(theta2[i]< - 3.14)
		 {
				theta2[i] = theta2[i] + 2 * pi;
		 }
	 }
	  
	 //================================theta4================================ 
   for(uint16_t i =0;i<4;i++)
	 {
		float s5 = sin(theta5[i]);
		float c5 = cos(theta5[i]);
		float s1 = sin(theta1[i]);
		float c1 = cos(theta1[i]);
		 
		float c4  =  r31*c5 - r32*s5;
		float s4  =  r11*c1*c5 - r12*c1*s5 + r21*c5*s1 - r22*s1*s5;
		theta4[i] =  arctan(-s4,c4) - theta2[i] - theta3[i]; 
	 }
	 	 //判断角度是否越界
	 for(uint16_t i=0;i<4;i++)
	 {
		 if(theta4[i]>3.14)
		 {
				theta4[i] = theta4[i] - 2 * pi;
		 }
		 else if(theta4[i]<-3.14)
		 {
				theta4[i] = theta4[i] + 2 * pi;
		 }
	 }

	 //獲得四個解
	 for(int i = 0;i<4;i++)
	 {
		 Tans[i][0] = theta1[i];
		 Tans[i][1] = theta2[i];
		 Tans[i][2] = theta3[i];
		 Tans[i][3] = theta4[i]; 
		 Tans[i][4] = theta5[i]; 
	 }
	 
	 //=============================解的筛选(处理多解问题)========================================
   //主要分析机械臂的1/2/3关节角度即可
	 for(int i =0;i<4;i++)
	 {
		if((-2<Tans[i][0]&&Tans[i][0]<2)&&(-1<Tans[i][1]&&Tans[i][1]<1)&&(-1.67<Tans[i][2]&&Tans[i][2]<0.9))
		{
		 res[0] = theta1[i];
		 res[1] = theta2[i];
		 res[2] = theta3[i];
		 res[3] = theta4[i]; 
		 res[4] = theta5[i]; 
		}
	 }
	  
}


//=============输入目标物体位置，获得物体相对于机械臂的变换矩阵================
void target_T_get(float x,float y,float z,float (*T)[4])
{
	T[0][0] = 0;T[0][1] = 1;T[0][2] =  0;T[0][3] = x;
	T[1][0] = 1;T[1][1] = 0;T[1][2] =  0;T[1][3] = y;
	T[2][0] = 0;T[2][1] = 0;T[2][2] = -1;T[2][3] = z+63;
	T[3][0] = 0;T[3][1] = 0;T[3][2] =  0;T[3][3] = 1;

}
