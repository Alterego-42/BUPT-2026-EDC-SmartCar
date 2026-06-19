#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include <Arduino.h>
//=====================================================================================
//  引脚配置接口 (Pin Configuration)
//=====================================================================================
// --- 通道选择引脚定义 (AD0, AD1, AD2)  Channel selection pin definition (AD0, AD1, AD2)---
#define SENSOR_AD0_PIN          2
#define SENSOR_AD1_PIN          3
#define SENSOR_AD2_PIN          4
#define SENSOR_OUT_PIN          5

//=====================================================================================
//  GPIO操作抽象接口 (GPIO Operation Macros)
//=====================================================================================
#define SENSOR_AD0_WRITE(state)  digitalWrite(SENSOR_AD0_PIN, (state) ? HIGH : LOW)
#define SENSOR_AD1_WRITE(state)  digitalWrite(SENSOR_AD1_PIN, (state) ? HIGH : LOW)
#define SENSOR_AD2_WRITE(state)  digitalWrite(SENSOR_AD2_PIN, (state) ? HIGH : LOW)

#define SENSOR_OUT_READ()        digitalRead(SENSOR_OUT_PIN)

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数 Number of sensor channels

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
