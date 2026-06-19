#include "AllHeader.h"


//#######################################################
// 重要配置参数 Important Configuration Parameters
//#######################################################
// 将灰度巡线模块放在赛道上，观察对着线的灯，如果灯亮，数值为1，LINE_RAW_VALUE=1；如果灯灭，数值为0，LINE_RAW_VALUE=0
// Place the grayscale line-following module on the track and observe the light facing the line. If the light is on, the value is 1, LINE_RAW_VALUE=1; if the light is off, the value is 0, LINE_RAW_VALUE=0
#define LINE_RAW_VALUE 1
//#######################################################

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
line_following_t g_line_controller;

// 巡线参数初始化 Initialize line following parameters
void line_following_init(line_following_t* controller) {
    controller->kp = 270.0f;    // 比例系数 | proportional gain
    controller->ki = 0.5f;      // 积分系数 | integral gain
    controller->kd = 50.0f;     // 微分系数 | derivative gain

    controller->last_error = 0.0f;  // 上次偏差 | last error
    controller->integral = 0.0f;    // 积分累积 | integral accumulation

    controller->base_speed = 450;   // 基础速度 | base speed
    controller->max_speed = 800;    // 最大速度 | maximum speed

    controller->sensor_weights[0] = -5.0f;
    controller->sensor_weights[1] = -4.0f;
    controller->sensor_weights[2] = -2.0f;
    controller->sensor_weights[3] = -1.0f;
    controller->sensor_weights[4] = 1.0f;
    controller->sensor_weights[5] = 2.0f;
    controller->sensor_weights[6] = 4.0f;
    controller->sensor_weights[7] = 5.0f;

    controller->motor_locked = true;  // 上电默认锁定电机 | lock motors on power-up
}

int main()
{
	bsp_init();  // 硬件初始化 board hardware initialization

	// 初始化灰度传感器GPIO Initialize grayscale sensor GPIO
	Grayscale_Sensor_Init();

	// 初始化循迹控制器参数 Initialize line following controller parameters
	line_following_init(&g_line_controller);

	TIM6_Init();  // 10ms定时器 (用于电机PID) 10ms timer (for motor PID)

	while(1)
	{
		// 读取8路传感器数据 Read 8-channel sensor data
		Grayscale_Sensor_Read_All(g_sensor_data);

		// 执行循迹控制 Execute line following control
		follow_line(&g_line_controller, g_sensor_data, LINE_RAW_VALUE);

		delay_ms(1);
	}
}
