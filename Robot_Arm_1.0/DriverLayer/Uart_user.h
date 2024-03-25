#ifndef UART_USER_H
#define UART_USER_H
#include "Uart_user.h"

void data_send();
void data_receive();

//中断发送方式
void data_send_IT();
void data_receive_IT();
void send_packet();



#endif
