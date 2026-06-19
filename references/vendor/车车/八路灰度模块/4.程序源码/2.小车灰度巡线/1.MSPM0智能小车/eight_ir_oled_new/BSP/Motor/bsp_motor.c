#include "bsp_motor.h"



void Init_Motor_PWM(void)
{
    DL_TimerA_startCounter(motor_PWM_INST);
   	
}

//电机死区
static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 0) return pulse + MOTOR_DEAD_ZONE;
    if (pulse < 0) return pulse - MOTOR_DEAD_ZONE;
    return 0;
}

//速度限制
int16_t speed_limit(int16_t speed,int16_t min,int16_t max)
{
    if(speed == 0)
        return 0;
    else if (speed<min)
        return min;
    
    else if(speed >max)
        return max;
    
    return speed;
}

int myabs(int a)
{
    if (a<0)
        return -a;
    return a;
}


void PWM_Control_Car(int16_t L_motor_speed , int16_t R_motor_speed )
{
    int16_t sl,sr;
    
    //限制最大最小pwm输出
    sl = speed_limit(L_motor_speed,-MAX_SPEED,MAX_SPEED);
    sr = speed_limit(R_motor_speed,-MAX_SPEED,MAX_SPEED);
    
    //限制电机死区
    sl = Motor_Ignore_Dead_Zone(sl);
    sr = Motor_Ignore_Dead_Zone(sr);
    
    if(sl<0)
    {
        sl = myabs(sl);
        L1_control(sl,1);
    }
    else
    {
       L1_control(sl,0); 
    }
    
    
    if(sr<0)
    {
        sr = myabs(sr);
        R1_control(sr,1);
    }
    else
    {
       R1_control(sr,0);
    }

    
}


//小车停止 参数刹车和不刹车
void Motor_Stop(uint8_t brake)
{
   if(brake == 1)//刹车
   {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, MAX_SPEED, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, MAX_SPEED, DL_TIMER_CC_2_INDEX);
       
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, MAX_SPEED, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, MAX_SPEED, DL_TIMER_CC_1_INDEX);
       
       
   }
   else
   {
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_2_INDEX);
       
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_1_INDEX);
   }
}






//motor_speed 0-999
//dir:0正转 1反转

void L1_control(uint16_t motor_speed,uint8_t dir)
{
	if(dir)
	{
      DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_3_INDEX);
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, motor_speed, DL_TIMER_CC_2_INDEX);
			
			
	}
	else
	{
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, motor_speed, DL_TIMER_CC_3_INDEX);
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_2_INDEX);
	}
		
}


void R1_control(uint16_t motor_speed,uint8_t dir)
{
	if(dir)
	{
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_0_INDEX);
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, motor_speed, DL_TIMER_CC_1_INDEX);
	}
	else
	{
      DL_TimerA_setCaptureCompareValue(motor_PWM_INST, motor_speed, DL_TIMER_CC_0_INDEX);
			DL_TimerA_setCaptureCompareValue(motor_PWM_INST, 0, DL_TIMER_CC_1_INDEX);
			
	}

}

void Motor_Run(float x,float y)
{
		L1_control(x,0);
		R1_control(y,0);
}


void Motor_Back(float x,float y)
{
		L1_control(x,1);
		R1_control(y,1);
}

void Motor_Left(float x,float y)
{

		L1_control(x,1);
		R1_control(y,0);
		delay_ms(500);
		Motor_Stop(1);
}

void Motor_Right(float x,float y)
{

		L1_control(x,0);
		R1_control(y,1);
		delay_ms(500);
		Motor_Stop(1);

}




