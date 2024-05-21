#include "stm32f4xx.h" 
#include "IK.h"
#include <math.h>

float t[100];
float pi = 3.1416;
	 float theta11[4]={0};
	 float theta22[4]={0};
   float theta33[4]={0};
	 float theta44[4]={0};
	 float theta55[4]={0};

float arctan(float sa,float ca)
{
	float eps = 0.00001;
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
			angle = atan2(sa,ca);
	}
	
	return angle;
	
	//abs(cos = 0 的时候，让theta = 0   
}

//================================机械臂逆解函数================================
void IK_calc(manipulator *arm,Matrix_T *T,Matrix_ans *Tans)
{
	//机械臂参数
  uint16_t a2 = arm->a[1]; uint16_t a3 = arm->a[2];uint16_t a4 = arm->a[3];
	uint16_t d1 = arm->d[0]; uint16_t d5 = arm->d[4];
	
  //T06矩阵
   float r11 = T->line1[0];float r12 = T->line1[1];float r13 = T->line1[2];float px = T->line1[3];
   float r21 = T->line2[0];float r22 = T->line2[1];float r23 = T->line2[2];float py = T->line2[3];
   float r31 = T->line3[0];float r32 = T->line3[1];float r33 = T->line3[2];float pz = T->line3[3];
	
	
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
		 theta5[i] =  arctan(r21*cos(theta1[i])-r11*sin(theta1[i]),r22*cos(theta1[i])-r12*sin(theta1[i]));
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
			   theta3[i]   = 25500000000; //表示无解
			   theta3[i+2] = 25500000000; //表示无解
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
				theta2[i] = 25500000000;
			}	
			else
			{
				theta2[i] = arctan(s2,c2); 
			}
		 }
		 else
		 {
			 theta2[i]= 25500000000;
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
		Tans->res1[0]=theta1[0];Tans->res1[1]=theta2[0];Tans->res1[2]=theta3[0];Tans->res1[3]=theta4[0];Tans->res1[4]=theta5[0];
	  Tans->res2[0]=theta1[1];Tans->res2[1]=theta2[1];Tans->res2[2]=theta3[1];Tans->res2[3]=theta4[1];Tans->res2[4]=theta5[1];
	  Tans->res3[0]=theta1[2];Tans->res3[1]=theta2[2];Tans->res3[2]=theta3[2];Tans->res3[3]=theta4[2];Tans->res3[4]=theta5[2];
	  Tans->res4[0]=theta1[3];Tans->res4[1]=theta2[3];Tans->res4[2]=theta3[3];Tans->res4[3]=theta4[3];Tans->res4[4]=theta5[3];
}