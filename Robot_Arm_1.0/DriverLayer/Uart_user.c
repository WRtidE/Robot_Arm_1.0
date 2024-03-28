#include "stm32f4xx.h"                  // Device header
#include "Uart_user.h"
#include "usart.h"

void data_send();
void data_receive();

//串口5通信
uint8_t rx_buffer[10];
uint8_t tx_buffer[10];
uint8_t rx_flag;
uint8_t buffer_length = 6;
uint8_t rx_data[4];

//串口6通信
uint8_t data_length = 9;
uint8_t rx_buffer_6[15];
uint8_t tx_buffer_6[15];
uint8_t ch05_data[12];

App_Data app_data;
//测试
uint8_t flag=0;

void data_receive_IT()
{
	HAL_UART_Receive_IT(&huart6, rx_buffer_6,data_length);
  HAL_Delay(1000);
}


//串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{  
	if(huart->Instance==USART6)
	{	 
   //接收蓝牙模块数据包
      static uint8_t RxState=0;
		
			if(RxState==0)
      { 
           if(rx_buffer_6[0]==0xA5)
           {
               RxState=1;
           } 
			}
      if(RxState==1)
      {  
        for(uint16_t i=0;i<data_length-3;i++)
				{
					ch05_data[i] =  rx_buffer_6[i+1];
				}
				//APP数据处理
			   app_data.x = (ch05_data[1]<<8)|ch05_data[0];
				 app_data.y = (ch05_data[3]<<8)|ch05_data[2];
				 app_data.z = (ch05_data[5]<<8)|ch05_data[4];
         RxState=2;
			}
      if(RxState==2)
      {
          if(rx_buffer_6[data_length-1]==0x5A)
         { 
				   RxState=0;
				 }
			}
		  HAL_UART_Receive_IT(&huart6,rx_buffer_6,data_length);
	}	
  
}

//中断发送方式
void data_send_IT()
{
	HAL_UART_Transmit_IT(&huart5,(uint8_t*)" Hello ",7);
	HAL_Delay(1000);
}

//发送数据包
void send_packet()
{
	tx_buffer[0] = 00;
	tx_buffer[1] = 01;
	tx_buffer[2] = 02;
	tx_buffer[3] = 03;
	tx_buffer[4] = 04;
	tx_buffer[5] = 05;
	tx_buffer[6] = 06;

	
	HAL_UART_Transmit(&huart5,(uint8_t*)0x12,1,0xffff);
	HAL_UART_Transmit(&huart5,(uint8_t*)tx_buffer,buffer_length,0xffff);
	HAL_UART_Transmit(&huart5,(uint8_t*)0xFE,1,0xffff);
	
	HAL_Delay(1000);
}