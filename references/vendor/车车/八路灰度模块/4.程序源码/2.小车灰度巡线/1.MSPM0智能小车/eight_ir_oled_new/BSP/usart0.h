#ifndef __USART0_H_
#define __USART0_H_

#include "AllHeader.h"

//#define EN_USART1_RX 			1		//??(1)/??(0)??1??
//	  	
//#define  RINGBUFF_LEN          (200)     //?????????
//#define  RINGBUFF_OK           1     
//#define  RINGBUFF_ERR          0   

//typedef struct
//{
//    uint16_t Head;           
//    uint16_t Tail;
//    uint16_t Lenght;
//    uint8_t  Ring_data[RINGBUFF_LEN];
//}RingBuff_t;

void USART_Init(void);
void uart0_send_char(char ch);
void uart0_send_string(char* str);


#endif

