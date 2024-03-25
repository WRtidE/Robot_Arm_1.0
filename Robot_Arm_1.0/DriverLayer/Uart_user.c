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
uint8_t rx_buffer_6[15];
uint8_t tx_buffer_6[15];
uint8_t ch05_data[12];

//中断发送方式
void data_send_IT()
{
	HAL_UART_Transmit_IT(&huart5,(uint8_t*)" Hello ",7);
	HAL_Delay(1000);
}
void data_receive_IT()
{
	HAL_UART_Receive_IT(&huart5, rx_buffer,buffer_length);
	HAL_UART_Receive_IT(&huart6, rx_buffer_6,15);
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



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{  
	if(huart->Instance==UART5)
	{	 
		
			if(rx_buffer[0] == 0xFF && rx_buffer[5] == 0xFE)
		  {
        rx_data[3] =  rx_buffer[1];
			  rx_data[2] =  rx_buffer[2];
			  rx_data[1] =  rx_buffer[3];
			  rx_data[0] =  rx_buffer[4];
				HAL_UART_Transmit_IT(&huart5,(uint8_t*)rx_data,4);
		  }

			
		static uint8_t RxState=0;
   //接收数据包
 
			if(RxState==0)
      { 
           if(rx_buffer[0]==0x0A)
           {
               RxState=1;
           } 
			}
      else if(RxState==1)
      {  
         rx_data[0]=rx_buffer[1];
         rx_data[1]=rx_buffer[2]; 
         rx_data[2]=rx_buffer[3];
         rx_data[3]=rx_buffer[4]; 
         RxState=2;
			}
      else if(RxState==2)
      {
          if(rx_buffer[5]==0xFE)
         { RxState=0;}
				 HAL_UART_Transmit_IT(&huart5,(uint8_t*)rx_data,4);
			}
		  HAL_UART_Receive_IT(&huart5,rx_buffer,buffer_length);
	}	
	
	if(huart->Instance==USART6)
	{	 
		
			if(rx_buffer_6[0] == 0xA5 && rx_buffer_6[14] == 0x5A)
		  {
        for(uint16_t i=0;i<12;i++)
				{
					ch05_data[i] =  rx_buffer[i+1];
				}
				HAL_UART_Transmit_IT(&huart6,(uint8_t*)ch05_data,12);
		  }

			
		static uint8_t RxState=0;
   //接收数据包
 
			if(RxState==0)
      { 
           if(rx_buffer_6[0]==0xA5)
           {
               RxState=1;
           } 
			}
      else if(RxState==1)
      {  
        for(uint16_t i=0;i<12;i++)
				{
					ch05_data[i] =  rx_buffer_6[i+1];
				}
         RxState=2;
			}
      else if(RxState==2)
      {
          if(rx_buffer_6[5]==0x5A)
         { RxState=0;}
				 HAL_UART_Transmit_IT(&huart6,(uint8_t*)ch05_data,12);
			}
		  HAL_UART_Receive_IT(&huart6,rx_buffer_6,15);
	}	

}