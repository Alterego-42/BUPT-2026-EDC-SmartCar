//Grayscale Sensor / Arduino UNO           Arduino UNO / Four-way Motor Driver
//        5V		          5V                 IOREF(5V)            5v
//        GND		          GND                   GND               GND        
//        AD0		          2                     SCL               SCL
//        AD1		          3                     SDA               SDA
//        AD2		          4
//        OUT		          5
#include <Arduino.h>
#include <stdio.h>
#include "grayscale_sensor.hpp"
#include "trace_task.hpp"

//#######################################################
// 重要配置参数 Important Configuration Parameters
//#######################################################
// 根据自己使用的电机类型，选择对应的电机参数配置
// Select the corresponding motor parameter configuration according to the type of motor you use
#define MOTOR_TYPE 5  //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor
//#######################################################
// 将灰度巡线模块放置于赛道，观察正对着线的灯，如果灯亮，则数值为1，LINE_RAW_VALUE=1；如果灯灭，则数值为0，LINE_RAW_VALUE=0
// Place the grayscale line-following module on the track and observe the light facing the line. If the light is on, the value is 1, LINE_RAW_VALUE=1; if the light is off, the value is 0, LINE_RAW_VALUE=0
#define LINE_RAW_VALUE 1
//#######################################################

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
line_following_t g_line_controller;

//巡线参数初始化 Initialize line following parameters
void line_following_init(line_following_t* controller) {
    controller->kp = 100.0f;    // 比例系数 | proportional gain
    controller->ki = 0.5f;      // 积分系数 | integral gain
    controller->kd = 50.0f;     // 微分系数 | derivative gain

    controller->last_error = 0.0f;  // 上次偏差 | last error
    controller->integral = 0.0f;    // 积分累积 | integral accumulation

    controller->base_speed = 350;   // 基础速度 | base speed
    controller->max_speed = 700;    // 最大速度 | maximum speed

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

void setup() {
  Grayscale_Sensor_Init();
  line_following_init(&g_line_controller);
  IIC_Motor_Init();
  Set_Motor(MOTOR_TYPE);
}

void loop() {
  Grayscale_Sensor_Read_All(g_sensor_data);
  delay(10);
  follow_line(&g_line_controller, g_sensor_data, MOTOR_TYPE, LINE_RAW_VALUE);
}

