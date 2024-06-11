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
	
	//������ĸ����
	int16_t letter_num;
	
} Control_Data;

extern Control_Data data;

//ʶ�𵽵�����
/*A B C D E F G H I J L  M  N  O  P  Q  R  S  T  U  V  W  X  Y  Z
  0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 */



#endif
