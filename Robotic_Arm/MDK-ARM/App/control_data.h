#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H

typedef __packed struct
{
	//��е��λ����Ϣ
  int16_t x;
	int16_t y;
	int16_t z;
	
	//������Ϣ
	int16_t kind;

	
	//ͨ��ֵ
	int16_t channel[5];
	 
	//�˶����ƿ���
	int16_t start; 

  //ִ����
  int16_t tool;

 //ģʽѡ��
  int16_t mode;	
 
	//����ѡ��
	int16_t function;
	
	//��ȡ
	int16_t catch_flag;
	
		
} Control_Data;

extern Control_Data data;



#endif
