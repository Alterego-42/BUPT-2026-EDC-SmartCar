#ifndef __APP_IRTRACKING_H__
#define __APP_IRTRACKING_H__


//#define X1  IR_Data_number[0]
//#define X2  IR_Data_number[1]
//#define X3  IR_Data_number[2]
//#define X4  IR_Data_number[3]
//#define X5  IR_Data_number[4]
//#define X6  IR_Data_number[5]
//#define X7  IR_Data_number[6]
//#define X8  IR_Data_number[7]


#include "AllHeader.h"

#define BLACK       1        //黑线black
#define WHITE       0        //白线white


#define IRR_SPEED_LIMIT 			  30 
#define u8 uint8_t
#define u16 uint16_t
extern u8 X1,X2,X3,X4,X5,X6,X7,X8;
int LineCheck(void);
void Line_Tracke(void);

void LineWalking_PWM(void);
void deal_IRdata(u8 *x1,u8 *x2,u8 *x3,u8 *x4,u8 *x5,u8 *x6,u8 *x7,u8 *x8);
void LineWalking(void);
#endif

