//Grayscale Sensor / ESP32S3        Grayscale Sensor / Four-way Motor Driver
//      AD0		        RX1(35)             5v               5V
//      AD1		        TX1(36)             GND		        GND
//      AD2		        SCL(37)
//      OUT		        SDA(38)
//ESP32S3 / Four-way Motor Driver
//   5V                5V
//   GND		       GND
//   TX0(43)		   RX2
//   RX0(44)		   TX2
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "grayscale_sensor.h"
#include "uart_module.h"
#include "trace_task.h"

//#######################################################
// 重要配置参数 Important Configuration Parameters
//#######################################################
// 根据自己使用的电机类型，选择对应的电机参数配置
// Select the corresponding motor parameter configuration according to the type of motor you use
#define MOTOR_TYPE 2  //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor
//#######################################################
// 将灰度巡线模块放置于赛道，观察正对着线的灯，如果灯亮，则数值为1，LINE_RAW_VALUE=1；如果灯灭，则数值为0，LINE_RAW_VALUE=0
// Place the grayscale line-following module on the track and observe the light facing the line. If the light is on, the value is 1, LINE_RAW_VALUE=1; if the light is off, the value is 0, LINE_RAW_VALUE=0
#define LINE_RAW_VALUE 1
//#######################################################

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))// 毫秒延时宏 | Millisecond delay macro

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
line_following_t g_line_controller;

//巡线参数初始化 Initialize line following parameters
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


void Grayscale_ReadData_Task(void *arg) {
	while (1) {
		Grayscale_Sensor_Read_All(g_sensor_data);
		delay_ms(10);  // 5ms读取一次 Read every 5ms
	}
}

void Trace_Task(void *pvParameters) {
    while (1) {
        // 调用巡线主函数 Call main line following function
        follow_line(&g_line_controller, g_sensor_data, MOTOR_TYPE, LINE_RAW_VALUE);

        // 延时，控制循环频率 Delay to control loop frequency
        delay_ms(10);
    }
}


void app_main(void)
{
    //初始化电机驱动板串口通信 Initialize motor driver board UART communication
	uart0_init();
    //初始化灰度传感器 Initialize grayscale sensor
    Grayscale_Sensor_Init();
    //设置电机类型 Set motor type
    Set_Motor(MOTOR_TYPE);
    //设置电机PID参数 Set motor PID parameters
    send_motor_PID(1.9,0.2,0.8);
    delay_ms(100);
    line_following_init(&g_line_controller);

    // 创建灰度传感器读取任务 Create grayscale sensor reading task
    xTaskCreate(
        Grayscale_ReadData_Task,    // 任务函数	Task Function
        "GrayscaleReadData",         // 任务名称	Task Name
        4096,                        // 堆栈大小（字节）	Stack size (bytes)
        NULL,
        3,                           // 优先级（数值越大优先级越高）	Priority (the larger the value, the higher the priority)
        NULL
    );

    // 创建巡线任务 Create line following task
    xTaskCreate(Trace_Task, "Trace", 4096, NULL, 4, NULL);
}
