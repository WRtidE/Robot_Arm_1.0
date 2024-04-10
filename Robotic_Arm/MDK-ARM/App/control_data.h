#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H

typedef __packed struct
{
	//��е��λ����Ϣ
  int16_t x;
	int16_t y;
	int16_t z;
	//�Ƕ���Ϣ
	int16_t p_int[4];
	
	//�ٶȿ���
	int16_t v_int[4];     
	
	//������Ϣ
	int16_t key[6];       
		
} Control_Data;

extern Control_Data data;



#endif
