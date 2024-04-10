#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H

typedef __packed struct
{
	//机械臂位置信息
  int16_t x;
	int16_t y;
	int16_t z;
	//角度信息
	int16_t p_int[4];
	
	//速度控制
	int16_t v_int[4];     
	
	//按键信息
	int16_t key[6];       
		
} Control_Data;

extern Control_Data data;



#endif
