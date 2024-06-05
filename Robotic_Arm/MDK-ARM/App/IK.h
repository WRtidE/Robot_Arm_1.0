#ifndef __IK_H
#define __IK_H

//��е�۳�ʼ��
typedef __packed struct
{
  //DH��
	int a[5];
	int d[5];
	
	//�ؽڽǶ�����
	float joint[5]; //����ؽڵĽǶ����ƣ���Ҫ����ǰ�����ؽڼ���
	
	//theta
}manipulator;

extern manipulator arm;
 
void target_T_get(float x,float y,float z,float (*T)[4]);
void IK_calc(float (*T)[4],float *res);
	
#endif
