#include "stm32f4xx.h" 
#include "IK.h"
#include <math.h>


//机械臂逆解函数
void IK_calc(manipulator *arm,Matrix_4 *T,Matrix_ans *Tans)
{
	//机械臂参数
  uint16_t a2 = arm->a[1]; uint16_t a3 = arm->a[2];
	
  //T06矩阵
   float r11 = T->line1[0];float r12 = T->line1[0];float r13 = T->line1[0];float px = T->line1[0];
   float r21 = T->line2[0];float r22 = T->line2[0];float r23 = T->line2[0];float py = T->line2[0];
   float r31 = T->line3[0];float r32 = T->line3[0];float r33 = T->line3[0];float pz = T->line3[0];
	
	//theta1
	 float theta1 = atan2(py,px);
	 float c1 = cos(theta1);
	 float s1 = sin(theta1);

  //theta5
   float theta5 = atan2(-r21*c1 + r11*s1,-r22*c1 + r12*s1);
	
	//theta3 
	 float c3 = (pz*pz + px*px*c1*c1+py*py*s1*s1+2*px*py*c1*s1-a2*a2-a3*a3)/(2*a2*a3);
	 float s3 = sqrt(1-c3*c3);
	 float theta3_1 =  atan2(s3,+c3);
   float theta3_2 =  atan2(s3,-c3);
	
	//theta2
	 float a = 2*(px*c1+py*s1)*a2;
	 float b = 2*pz*a2;
	 float c = (px*c1+py*s1)*(px*c1+py*s1) +a2*a2+pz*pz-a3*a3;
	 
	 float theta2_1 = atan2(b,a)+atan2(sqrt(a*a+b*b-c*c),c);
   float theta2_2 = atan2(b,a)-atan2(sqrt(a*a+b*b-c*c),c);
	 
	 //theta4 四个解
   float c234 = -r33;
   float s234 = sqrt(1-r33*r33);
   float theta4_1 = atan2(s234,c234) -theta2_1 - theta3_1;
   float theta4_2 = atan2(s234,c234) -theta2_1 - theta3_2;
   float theta4_3 = atan2(s234,c234) -theta2_2 - theta3_1;
   float theta4_4 = atan2(s234,c234) -theta2_2 - theta3_2;
	 
	 //獲得四個解
	 Tans->res1[0] = theta1; Tans->res1[1] = theta2_1; Tans->res1[2] = theta3_1; Tans->res1[3] = theta4_1; Tans->res1[4] = theta5;
	 Tans->res2[0] = theta1; Tans->res2[1] = theta2_1; Tans->res2[2] = theta3_2; Tans->res2[3] = theta4_2; Tans->res1[4] = theta5;
	 Tans->res3[0] = theta1; Tans->res3[1] = theta2_2; Tans->res3[2] = theta3_1; Tans->res3[3] = theta4_3; Tans->res1[4] = theta5;
	 Tans->res4[0] = theta1; Tans->res4[1] = theta2_2; Tans->res4[2] = theta3_2; Tans->res4[3] = theta4_4; Tans->res1[4] = theta5;

	 
}