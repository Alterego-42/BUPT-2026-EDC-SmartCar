#ifndef  __PID_MOTOR_H
#define  __PID_MOTOR_H

#include "AllHeader.h"

#define PI      (3.1415926f)

#define PID_MOTOR_KP  (1.5f)
#define PID_MOTOR_KI  (0.08f)
#define PID_MOTOR_KD  (0.5f)


typedef struct _pid
{
    float target_val;               //目标值                 //Target value
    float output_val;               //输出值                //Output value
    float pwm_output;        		//PWM输出值          //PWM output value
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数 //Define proportional, integral, and derivative coefficients
    float err;             			//定义偏差值              //Define error value
    float err_last;          		//定义上一个偏差值  //Define last error value

    float err_next;                 //定义下一个偏差值, 增量式 //Define next error value, incremental type
    float integral;          		//定义积分值，位置式 //Define integral value, positional type
} pid_t;


typedef struct _motor_data_t
{
    float speed_mm_s[4];        // 输入值，编码器计算速度 // Input value, encoder calculated speed
    float speed_pwm[4];         // 输出值，PID计算出PWM值 // Output value, PID calculated PWM value
    int16_t speed_set[4];       // 速度设置值 // Speed setting value
} motor_data_t;

void PID_Param_Init(void);
void PID_Calc_Motor(motor_data_t* motor);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
float PID_Incre_Calc(pid_t *pid, float actual_val);


#endif

