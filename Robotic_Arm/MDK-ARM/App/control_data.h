#ifndef __CONTROL_DATA_H
#define __CONTROL_DATA_H

typedef __packed struct
{
	//机械臂位置信息
  int16_t x;
	int16_t y;
	int16_t z;
	
	//种类信息
	int16_t kind;

	
	//通道值
	int16_t channel[5];
	 
	//运动控制开关
	int16_t start; 

  //执行器
  int16_t tool;

 //模式选择
  int16_t mode;	
 
	//功能选择
	int16_t function;
	
	//夹取
	int16_t catch_flag;
	
		
} Control_Data;

extern Control_Data data;



#endif
