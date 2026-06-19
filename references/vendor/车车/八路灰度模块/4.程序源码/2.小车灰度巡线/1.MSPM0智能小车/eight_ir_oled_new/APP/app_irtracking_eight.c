#include "app_irtracking_eight.h"

#define IRTrack_Trun_KP (700)  //400
#define IRTrack_Trun_KI (0.02f)  // 0.01
#define IRTrack_Trun_KD (0)  //40
#define ACTIVE_LEVEL 1


u8 X1,X2,X3,X4,X5,X6,X7,X8;
int pid_output_IRR = 0;
u8 trun_flag = 0;

#define IRR_SPEED 			  350  //巡线速度	Line patrol speed 200




float APP_HD_PID_Calc(int8_t actual_value)
{

	float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRTrack_Integral;//积分	integral
	
	error=actual_value;
	
	IRTrack_Integral +=error;
	
	//位置式pid	Position pid
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	
	if (IRTrackTurn > MAX_SPEED+200)
			IRTrackTurn = MAX_SPEED+200;
	if (IRTrackTurn < -MAX_SPEED-200)
			IRTrackTurn = -MAX_SPEED-200;
	return IRTrackTurn;
}

static int16_t last_error = 0;

void LineWalking_PWM(void)
{

    int16_t left_speed = IRR_SPEED;  // 基础PWM值
    int16_t right_speed = IRR_SPEED; // 基础PWM值
    

    // 简单逻辑控制（0表示检测到黑线）
    if (X4 == 0 && X5 == 0) {
        // 中间两个传感器同时检测到黑线：直行
        left_speed = IRR_SPEED;
        right_speed = IRR_SPEED;
    }
    else if (X3 == 0) {
        // 中左侧传感器检测到黑线：小幅左转
        left_speed -= 10;
        right_speed += 10;
    }
    else if (X6 == 0) {
        // 中右侧传感器检测到黑线：小幅右转
        left_speed += 10;
        right_speed -= 10;
    }
    else if (X1 == 0 || X2 == 0) {
        // 最左侧传感器检测到黑线：大幅左转
        left_speed -= 100;
        right_speed += 100;
    }
    else if (X7 == 0 || X8 == 0) {
        // 最右侧传感器检测到黑线：大幅右转
        left_speed += 100;
        right_speed -= 100;
    }
    else {

    }
    
    // 控制电机
    PWM_Control_Car(left_speed, right_speed);
}





//检测现在位于黑线还是在白线上	Detection is now on the black line or on the white line
int LineCheck(void)
{

	//当所有传感器都为1（未检测到黑线）时，if_have才为0
	if(X1 && X2 && X3 && X4 && X5 && X6 && X7 && X8)
	{
		return WHITE;
	}
	else
	{
		return BLACK;
	}
		
}
void Copy_HD_Data()
{
			X1 =IR_Data_number[0];
			X2 =IR_Data_number[1]; 
			X3 =IR_Data_number[2];
			X4 =IR_Data_number[3];
			X5 =IR_Data_number[4];
			X6 =IR_Data_number[5];
			X7 =IR_Data_number[6];
			X8 =IR_Data_number[7];

}


void Line_Tracke(void) {
    static int8_t err = 0;

    int weights[8] = {-30, -20, -15, -5, 5, 15, 20, 30};
    int weighted_sum = 0;
    int sensor_active_count = 0;

    // 假设检测到黑线时传感器值为1
    if (X1 == 1) { weighted_sum += weights[0]; sensor_active_count++; }
    if (X2 == 1) { weighted_sum += weights[1]; sensor_active_count++; }
    if (X3 == 1) { weighted_sum += weights[2]; sensor_active_count++; }
    if (X4 == 1) { weighted_sum += weights[3]; sensor_active_count++; }
    if (X5 == 1) { weighted_sum += weights[4]; sensor_active_count++; }
    if (X6 == 1) { weighted_sum += weights[5]; sensor_active_count++; }
    if (X7 == 1) { weighted_sum += weights[6]; sensor_active_count++; }
    if (X8 == 1) { weighted_sum += weights[7]; sensor_active_count++; }

    if (sensor_active_count > 0) {
        // 计算平均误差，减少传感器数量变化的干扰
        err = weighted_sum / sensor_active_count;
    } else {
        // 所有传感器都未检测到黑线，进入寻线模式
        if (err > 0) {
            err = 30; // 上次偏右，则继续右转寻找
        } else if (err < 0) {
            err = -30; // 上次偏左，则继续左转寻找
        } else {
            err = 0; 
        }
    }

    // 2. 应用PID计算
    pid_output_IRR = (int)(APP_HD_PID_Calc(err));

    // 3. 控制小车运动
    Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);
}

void LineWalking(void)
{
    static int8_t err = 0;
		ReadEightIR(IR_Data_number);
		Copy_HD_Data();


		 if(X4 == ACTIVE_LEVEL && X5 == ACTIVE_LEVEL&& X1 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = 0;  
    }
		// 直角左转判断
		else if((X1 == ACTIVE_LEVEL || X2 == ACTIVE_LEVEL)&&(X4 == ACTIVE_LEVEL || X5 == ACTIVE_LEVEL))
    {
        err = -20;   // APP_HD_PID_Calc
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(-20, 0, pid_output_IRR);
				delay_ms(300);
			return;
		} 
		 else if((X7 == ACTIVE_LEVEL || X8 == ACTIVE_LEVEL)&&(X4 == ACTIVE_LEVEL || X5 == ACTIVE_LEVEL))
    {
        err = 20;   // APP_HD_PID_Calc
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(-20, 0, pid_output_IRR);
				delay_ms(300);
			return;
		} 
//    // 左侧传感器状态处理 - 小幅度调整
    else if(X4 == ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL)
    {
        // X4检测到线，X5未检测到，轻微向左偏
        err = -1;   // 小幅度左转
    }

    
    // 右侧传感器状态处理 - 小幅度调整
    else if(X4 == !ACTIVE_LEVEL && X5 == ACTIVE_LEVEL)
    {
        // X5检测到线，X4未检测到，轻微向右偏
        err = 1;    // 小幅度右转
    }

    
    // 左侧较大偏移处理
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = -1;
    }
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == ACTIVE_LEVEL && 
            X4 == ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = -2;
    }
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = -4;
    }
    else if(X1 == !ACTIVE_LEVEL && X2 == ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = -6;
    }
    else if(X1 == ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = -12;
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(IRR_SPEED-70, 0, pid_output_IRR);
    }
		
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == ACTIVE_LEVEL && X6 == ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = 3;
    }
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = 5;
    }
    else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == ACTIVE_LEVEL && 
            X7 == ACTIVE_LEVEL && X8 == !ACTIVE_LEVEL)
    {
        err = 8;
    }
  
     else if(X1 == !ACTIVE_LEVEL && X2 == !ACTIVE_LEVEL && X3 == !ACTIVE_LEVEL && 
            X4 == !ACTIVE_LEVEL && X5 == !ACTIVE_LEVEL && X6 == !ACTIVE_LEVEL && 
            X7 == !ACTIVE_LEVEL && X8 == ACTIVE_LEVEL)
    {
        err = 12;
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(IRR_SPEED-70, 0, pid_output_IRR);
    }   

		else  if(X1 == ACTIVE_LEVEL && X2 == ACTIVE_LEVEL&&(X3 == ACTIVE_LEVEL || X4 == ACTIVE_LEVEL || X5 == ACTIVE_LEVEL) && X8 == !ACTIVE_LEVEL)
    {
			
        err = -15;  // 负向误差，需要左转
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(-20, 0, pid_output_IRR);
				delay_ms(250);
			return;
		} 
		

    // 直角右转判断
    else if(X7 == ACTIVE_LEVEL && X8 == ACTIVE_LEVEL && (X6 == ACTIVE_LEVEL || X4 == ACTIVE_LEVEL || X5 == ACTIVE_LEVEL) && X1 == !ACTIVE_LEVEL)
    {
        err = 15;   // APP_HD_PID_Calc
				pid_output_IRR = (int)(APP_HD_PID_Calc(err));
				Motion_Car_Control(-20, 0, pid_output_IRR);
				delay_ms(300);
			return;
    }

    // 剩下的情况保持上一个状态（防止频繁切换）
    // 通过PID计算输出并控制小车运动
    pid_output_IRR = (int)(APP_HD_PID_Calc(err));
    Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);
}
