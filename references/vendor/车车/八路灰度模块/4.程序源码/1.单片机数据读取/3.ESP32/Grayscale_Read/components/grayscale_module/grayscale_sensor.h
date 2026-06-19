#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h> 
//=====================================================================================
//  引脚配置接口 (Pin Configuration)
//=====================================================================================
// --- 通道选择引脚定义 (AD0, AD1, AD2)  Channel selection pin definition (AD0, AD1, AD2)---
#define SENSOR_AD0_PIN          GPIO_NUM_35
#define SENSOR_AD1_PIN          GPIO_NUM_36
#define SENSOR_AD2_PIN          GPIO_NUM_37
#define GrayS_OUT_PIN           GPIO_NUM_38
//=====================================================================================
//  GPIO操作抽象接口 (GPIO Operation Macros)
//=====================================================================================
#define SENSOR_AD0_WRITE(state)  gpio_set_level(SENSOR_AD0_PIN, (state) ? 1 : 0)
#define SENSOR_AD1_WRITE(state)  gpio_set_level(SENSOR_AD1_PIN, (state) ? 1 : 0)
#define SENSOR_AD2_WRITE(state)  gpio_set_level(SENSOR_AD2_PIN, (state) ? 1 : 0)
#define SENSOR_OUT_READ()        gpio_get_level(GrayS_OUT_PIN)

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数 Number of sensor channels

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
