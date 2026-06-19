#ifndef __BSP_IR_EIGHT_H__
#define __BSP_IR_EIGHT_H__
#include "ti_msp_dl_config.h"


// 八路灰度传感器的引脚定义
#define EIGHT_IR_PORT        Eight_IR_PORT   // 端口
#define EIGHT_IR_AD0_PIN     Eight_IR_AD0_X1_PIN    // 通道选择：A（最低位）
#define EIGHT_IR_AD1_PIN     Eight_IR_AD1_X2_PIN    // 通道选择：B
#define EIGHT_IR_AD2_PIN     Eight_IR_AD2_X3_PIN    // 通道选择：C（最高位）
#define EIGHT_IR_OUT_PIN     Eight_IR_OUT_X4_PIN    // 红外检测输出引脚


// 设置通道选择引脚 C、B、A 的电平（0 或 1）
#define SET_CHANNEL(c_val, b_val, a_val) \
    do { \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD2_PIN, c_val); \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD1_PIN, b_val); \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD0_PIN, a_val); \
    } while(0)


// 读取灰度输出引脚状态（返回 1 或 0）
#define READ_IR_OUT()  DL_GPIO_readPins(EIGHT_IR_PORT, EIGHT_IR_OUT_PIN)
void GPIO_setPins(GPIO_Regs* gpio, uint32_t pins,uint8_t val);
void ReadEightIR(uint8_t ir_results[8]);	
extern volatile uint8_t IR_Data_number[8];
void OLED_SHOW_IR(void);
#endif






