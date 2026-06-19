#ifndef  __APP_MOTOR_H
#define  __APP_MOTOR_H

#include "AllHeader.h"

// 450RPM电机，轮子转一整圈，编码器获得的脉冲数=减速比*码盘线数*编码器脉冲（20*13*4）
//For a 450RPM motor, the number of pulses obtained by the encoder for one full revolution of the wheel = reduction ratio * number of code disc lines * encoder pulses (20 * 13 * 4)
#define ENCODER_CIRCLE_450           (1040.0f)

//小车底盘电机间距之和的一半 / Half of the distance between the chassis motors
#define STM32Car_APB          				(132.8f)

// 轮子转一整圈的位移，单位为米 / Wheel displacement for one full rotation, in millimeters
#define MECANUM_CIRCLE_MM            (204.203f)


typedef struct _car_data
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} car_data_t;


void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);

void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);

void Motion_Handle(void);

void Motion_Get_Speed(car_data_t* car);
float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);

#endif

