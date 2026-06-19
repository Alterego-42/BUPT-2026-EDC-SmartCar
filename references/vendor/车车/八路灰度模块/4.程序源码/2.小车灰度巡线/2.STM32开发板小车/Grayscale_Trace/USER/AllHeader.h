#ifndef __AllHeader_H
#define __AllHeader_H

#define bool _Bool
#define true 1
#define false 0

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Time.h>

//基本头文件 Basic header files
#include "stm32f10x.h"
#include "switch_function.h"
#include "delay.h"
#include "bsp.h"
#include "bsp_timer.h"

//串口部分 UART part
#include "bsp_usart.h"

//按键部分 Key part
#include "bsp_key.h"

#include "bsp_beep.h"

//电机部分 Motor part
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "PID_Motor.h"
#include "app_motor.h"

#include "grayscale_sensor.h"
#include "trace_task.h"

//板载led部分 On-board LED part
#include "bsp_LED.h"



//其它变量 Other variables
extern u8 g_board_no_error;
extern u8 g_new_package_flag; //接收到新的一包数据标志 Receive a new packet flag
extern u8 ruijiao_flag;
#endif

