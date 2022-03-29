#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200
#define EN_USART1_RX 			1
	  	
extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;
extern UART_HandleTypeDef UART1_Handler;

#define RXBUFFERSIZE   1
extern u8 aRxBuffer[RXBUFFERSIZE];


void uart_init(u32 bound);
#endif


