#ifndef __APP_SPEECH_H_
#define __APP_SPEECH_H_


#include "AllHeader.h"


//²¥±¨´Ê Active broadcast content
#define This_red 0x5F    
#define This_blue 0x60
#define This_green 0x61
#define This_yellow 0x62
#define Recognize_yellow 0x63
#define Recognize_green 0x64
#define Recognize_blue 0x65
#define Recognize_red 0x66
#define init 0x67     


typedef enum _Speech_Cmd_t
{
    Car_STOP = 0x01,
    Car_STOP2 ,
    Car_STOP3 ,
    Car_Forword,
    Car_Back,
    Car_Left,
    Car_Right,
    Car_SpinLeft,
    Car_SpinRight,
}Speech_Cmd_t;


void Processing_Data(uint8_t RXdata);
void Write_Data(uint8_t dat);
void USER_Speech_Contorl_Car(void);


#endif

