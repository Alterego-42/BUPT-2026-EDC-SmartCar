#include "PID_Motor.h"

pid_t pid_motor[4];


// 初始化PID参数 Initialize PID parameters
void PID_Param_Init(void)
{
    /* 速度相关初始化参数 Speed-related initialization parameters*/
    for (int i = 0; i < Motor_MAX; i++)
    {
        pid_motor[i].target_val = 0.0;
        pid_motor[i].pwm_output = 0.0;
        pid_motor[i].err = 0.0;
        pid_motor[i].err_last = 0.0;
        pid_motor[i].err_next = 0.0;
        pid_motor[i].integral = 0.0;

        pid_motor[i].Kp = PID_MOTOR_KP;
        pid_motor[i].Ki = PID_MOTOR_KI;
        pid_motor[i].Kd = PID_MOTOR_KD;
    }
    
}

// 增量式PID计算公式 Incremental PID calculation formula
float PID_Incre_Calc(pid_t *pid, float actual_val)
{
    /*计算目标值与实际值的误差 Calculate the error between the target value and the actual value*/
    pid->err = pid->target_val - actual_val;
    /*PID算法实现 PID algorithm implementation*/
    pid->pwm_output += pid->Kp *(pid->err - pid->err_next) 
                    + pid->Ki  * pid->err 
                    + pid->Kd  *(pid->err - 2 * pid->err_next + pid->err_last);
    /*传递误差 Error transmission*/
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    
    /*返回PWM输出值 Return PWM output value*/

		if (pid->pwm_output > (MOTOR_MAX_PULSE-MOTOR_IGNORE_PULSE))
				pid->pwm_output = (MOTOR_MAX_PULSE-MOTOR_IGNORE_PULSE);
		if (pid->pwm_output < (MOTOR_IGNORE_PULSE-MOTOR_MAX_PULSE))
				pid->pwm_output = (MOTOR_IGNORE_PULSE-MOTOR_MAX_PULSE);

    return pid->pwm_output;
}


// PID计算输出值 Calculate PID output value
void PID_Calc_Motor(motor_data_t* motor)
{
    int i;
    for (i = 0; i < Motor_MAX; i++)
    {
        motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i]);
    }
}


// 设置PID目标速度，单位为：mm/s Set PID target speed, unit: mm/s
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > Motor_MAX) return;

    if (motor_id == Motor_MAX)
    {
        for (int i = 0; i < Motor_MAX; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}
