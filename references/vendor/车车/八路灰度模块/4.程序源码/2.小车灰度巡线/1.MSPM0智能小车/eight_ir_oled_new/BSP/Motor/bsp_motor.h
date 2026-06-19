#ifndef __BSP_MOTOR_H_
#define __BSP_MOTOR_H_


#include "AllHeader.h"


#define MOTOR_DEAD_ZONE  (75) 
#define MAX_SPEED (1000)

typedef enum
{
	MOTOR_ID_M1 = 0,
	MOTOR_ID_M2,
	MAX_MOTOR
} Motor_ID;


int myabs(int a);
int16_t speed_limit(int16_t speed,int16_t min,int16_t max);
void Init_Motor_PWM(void);
void PWM_Control_Car(int16_t L_motor_speed , int16_t R_motor_speed );
void L1_control(uint16_t motor_speed,uint8_t dir);
void R1_control(uint16_t motor_speed,uint8_t dir);
void Motor_Stop(uint8_t brake);

void Motor_Run(float x,float y);
void Motor_Right(float x,float y);
void Motor_Back(float x,float y);
void Motor_Left(float x,float y);
#endif

