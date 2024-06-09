#include "stm32f4xx.h"                  // Device header
#include "Uart_user.h"
#include "usart.h"
#include "control_data.h"

void data_send();
void data_receive();


//串口6通信
uint8_t data_length  = 25;
uint8_t rx_buffer_ch05[25];
uint8_t tx_buffer_ch05[25];

//串口5通信
uint8_t data_length_pc = 10;
uint8_t rx_buffer_pc[10];
uint8_t tx_buffrt_pc[10];


Control_Data data;
//测试
uint8_t flag=0;


void data_receive_IT()
{
	
	HAL_UART_Receive_IT(&huart5,rx_buffer_pc,data_length_pc);
	//接受空闲中断
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	HAL_UART_Receive_DMA(&huart6, rx_buffer_ch05,data_length);
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
           if(rx_buffer_ch05[0]==0xA5)
           {
               RxState=1;
           } 
			}
      if(RxState==1)
      {  
				//APP数据处理
//				 data.x = (rx_buffer_ch05[2]<<8)|rx_buffer_ch05[1];
//				 data.y = (rx_buffer_ch05[4]<<8)|rx_buffer_ch05[3];
//				 data.z = (rx_buffer_ch05[6]<<8)|rx_buffer_ch05[5];
				
				 data.channel[0] = (rx_buffer_ch05[8]<<8)|rx_buffer_ch05[7];
				 data.channel[1] = (rx_buffer_ch05[10]<<8)|rx_buffer_ch05[9];
				 data.channel[2] = (rx_buffer_ch05[12]<<8)|rx_buffer_ch05[11];
				 data.channel[3] = (rx_buffer_ch05[14]<<8)|rx_buffer_ch05[13];
				 data.channel[4] = (rx_buffer_ch05[16]<<8)|rx_buffer_ch05[15];
				 data.tool       = (rx_buffer_ch05[18]<<8)|rx_buffer_ch05[17];
         data.mode       = (rx_buffer_ch05[20]<<8)|rx_buffer_ch05[19];
				 data.start      = (rx_buffer_ch05[22]<<8)|rx_buffer_ch05[21];
				 data.function   = (rx_buffer_ch05[24]<<8)|rx_buffer_ch05[23];
						 
         RxState=2;
			}
      if(RxState==2)
      {
          if(rx_buffer_ch05[data_length-1]==0x5A)
         { 
				   RxState=0;
				 }
				 //HAL_UART_Receive_DMA(&huart6,rx_buffer_ch05,data_length);
			}
		  
	}	
		if(huart->Instance==UART5)
	{	 
		//接收上位机模块数据包
    static uint8_t RxState=0;
		  if(RxState==0)
      { 
           if(rx_buffer_pc[0]==0xFE)
           {
               RxState=1;
           } 
			}
      if(RxState==1)
      {  
				//测试数据 FE 38 FF C8 00 32 00 01 00 EF
				uint8_t data_rx[8];
				//APP数据处理
				 data.x    = (rx_buffer_pc[2]<<8)|rx_buffer_pc[1];
			   data.y    = (rx_buffer_pc[4]<<8)|rx_buffer_pc[3];
				 data.z    = (rx_buffer_pc[6]<<8)|rx_buffer_pc[5];
				 data.kind = (rx_buffer_pc[8]<<8)|rx_buffer_pc[7];
				
				//data_rx[0] = data.x>>8;
				//data_rx[1] = data.x;	
         RxState=2;
			}
      if(RxState==2)
      {
          if(rx_buffer_pc[data_length_pc-1]==0xEF)
         { 
				   RxState=0;
				 }
			}
	}
	
	HAL_UART_Receive_IT(&huart5,rx_buffer_pc,data_length_pc);
	HAL_UART_Receive_DMA(&huart6,rx_buffer_ch05,data_length);
}

//中断发送方式
void data_send_IT()
{
	HAL_UART_Transmit(&huart5,(uint8_t*)" Hello ",7,0xFFFF);
	//HAL_UART_Transmit_IT(&huart5,(uint8_t*)" Hello ",7);
	HAL_Delay(1000);
}

////发送数据包
//void send_packet()
//{
//	tx_buffer[0] = 00;
//	tx_buffer[1] = 01;
//	tx_buffer[2] = 02;
//	tx_buffer[3] = 03;
//	tx_buffer[4] = 04;
//	tx_buffer[5] = 05;
//	tx_buffer[6] = 06;

//	
//	HAL_UART_Transmit(&huart5,(uint8_t*)0x12,1,0xffff);
//	HAL_UART_Transmit(&huart5,(uint8_t*)tx_buffer,buffer_length,0xffff);
//	HAL_UART_Transmit(&huart5,(uint8_t*)0xFE,1,0xffff);
//	
//	HAL_Delay(1000);
//}