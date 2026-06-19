#ifndef __APP_K210_H_
#define __APP_K210_H_

#include "AllHeader.h"
typedef struct K210_Data
{
	uint16_t k210_X ; //识别框x轴的中心点 Identify the center point of the x-axis of the box
	uint16_t k210_Y ; //识别框y轴的中心点 Identify the center point of the y-axis of the box
	uint16_t k210_W ;
	uint16_t k210_H ;
	uint16_t k210_AREA;
}K210_Data_t;

extern K210_Data_t K210_data;
void APP_K210X_Init(void);
float APP_K210X_PID_Calc(float actual_value);
void APP_K210X_Line_PID(void);
void BSP_Loop(void);
void Color_Trace(int x,int y,int w,int h);


#endif


