#ifndef __IK_H
#define __IK_H

//��е�۳�ʼ��DH��
typedef __packed struct
{
  //DH��
	int a[5];
	int d[5];
	
	//theta
}manipulator;


//T����ṹ��
typedef __packed struct
{
   float line1[4];
	 float line2[4];
	 float line3[4];
	 float line4[4];

}Matrix_T;



//T����ṹ��
typedef __packed struct
{
   float res1[5];
	 float res2[5];
	 float res3[5];
	 float res4[5]; 
}Matrix_ans;

void IK_calc(manipulator *arm,Matrix_T *T,Matrix_ans *Tans);
	
#endif
