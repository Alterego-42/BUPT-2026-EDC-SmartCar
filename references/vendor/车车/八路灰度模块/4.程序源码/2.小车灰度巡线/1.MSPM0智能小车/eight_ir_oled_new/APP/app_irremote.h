#ifndef __APP_IRREMOTE
#define __APP_IRREMOTE


#include "AllHeader.h"

#define IR_POWER			0x00   
#define IR_UP				0x80
#define IR_LIGHT			0x40
#define IR_LEFT				0x20
#define IR_BEEP				0xA0
#define IR_RIGHT			0x60
#define IR_BACK       0x90
#define IR_LEFT_SPIN	 	0x10
//#define IR_DOWN		   		0x90
#define IR_RIHGT_SPIN		0x50
#define IR_ADD				0x30
#define IR_0				0xB0
#define IR_SUB				0x70
#define IR_1				0x08
#define IR_2				0x88
#define IR_3				0x48
#define IR_4				0x28
#define IR_5				0xA8
#define IR_6				0x68
#define IR_7				0x18
#define IR_8				0x98
#define IR_9				0x58

#define CLEAR_DATA          0xFF





void Printf_Irremote(void);

void IR_Control_Car(void);









#endif


