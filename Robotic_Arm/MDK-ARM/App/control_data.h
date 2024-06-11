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
	
	//单词字母个数
	int16_t letter_num;
	
} Control_Data;

extern Control_Data data;

//识别到的种类
/*A B C D E F G H I J L  M  N  O  P  Q  R  S  T  U  V  W  X  Y  Z
  0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 */



#endif
