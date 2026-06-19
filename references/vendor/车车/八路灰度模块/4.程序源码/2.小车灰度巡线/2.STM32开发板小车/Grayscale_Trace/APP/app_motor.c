#include "app_motor.h"

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;

static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

static int g_offset_yaw = 0;
static uint16_t g_speed_setup = 0;


// 编码器10毫秒前后数据 encoder data 10 milliseconds before and after
int g_Encoder_All_Now[Motor_MAX] = {0};
int g_Encoder_All_Last[Motor_MAX] = {0};
int g_Encoder_All_Offset[Motor_MAX] = {0};


uint8_t g_start_ctrl = 0;

car_data_t car_data;
motor_data_t motor_data;

static float Motion_Get_Circle_Pulse(void)
{  
    return ENCODER_CIRCLE_450;
}

// 返回当前小车轮子轴间距和的一半 back to half of the current car wheelbase sum
float Motion_Get_APB(void)
{
    return STM32Car_APB;
}

// 返回当前小车轮子转一圈的多少毫米 back to millimeters per wheel revolution
float Motion_Get_Circle_MM(void)
{
		return MECANUM_CIRCLE_MM; 
}

// 获取编码器数据，并计算偏差脉冲数 Get encoder data and calculate deviation pulse number
void Motion_Get_Encoder(void)
{
    Encoder_Get_ALL(g_Encoder_All_Now);

    for(uint8_t i = 0; i < Motor_MAX; i++)
    {
        // 记录两次测试时间差的脉冲数 record the pulse number of the time difference between two tests
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    // 记录上次编码器数据 record last encoder data
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    
    }
}

// 控制小车运动，Motor_X=[-3600, 3600]，超过范围则无效。 Control car movement, Motor_X=[-3600, 3600], out of range is invalid.
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    if (Motor_1 >= -MOTOR_MAX_PULSE && Motor_1 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_M1, Motor_1);
    }
    if (Motor_2 >= -MOTOR_MAX_PULSE && Motor_2 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_M2, Motor_2);
    }
    if (Motor_3 >= -MOTOR_MAX_PULSE && Motor_3 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_M3, Motor_3);
    }
    if (Motor_4 >= -MOTOR_MAX_PULSE && Motor_4 <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_M4, Motor_4);
    }
}

// speed_mX=[-1000, 1000], 单位为：mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;
    for (uint8_t i = 0; i < Motor_MAX; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i]*1.0);
    }
}

// 从编码器读取当前各轮子速度，单位mm/s get the current wheel speed from the encoder, unit mm/s
void Motion_Get_Speed(car_data_t* car)
{
    int i = 0;
    float speed_mm[Motor_MAX] = {0};
    float circle_mm = Motion_Get_Circle_MM();
    float circle_pulse = Motion_Get_Circle_Pulse();
    float robot_APB = Motion_Get_APB();

    Motion_Get_Encoder();

    // 计算轮子速度，单位mm/s。 Calculate wheel speed, unit mm/s.
    for (i = 0; i < 4; i++)
    {
        speed_mm[i] = (g_Encoder_All_Offset[i]) * 100 * circle_mm / circle_pulse;
    }
		
		 car->Vx = (speed_mm[0] + speed_mm[1] + speed_mm[2] + speed_mm[3]) / 4;
     car->Vy = 0;
     car->Vz = -(speed_mm[0] + speed_mm[1] - speed_mm[2] - speed_mm[3]) / 4.0f / robot_APB * 1000;
		
		if (g_start_ctrl)
    {
        for (i = 0; i < Motor_MAX; i++)
        {
            motor_data.speed_mm_s[i] = speed_mm[i];
        }
        
        PID_Calc_Motor(&motor_data);
		}
}

// 运动控制句柄，每10ms调用一次，主要处理速度相关的数据 Motion control handle, called every 10ms, mainly handles speed-related data
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1], motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}
		

