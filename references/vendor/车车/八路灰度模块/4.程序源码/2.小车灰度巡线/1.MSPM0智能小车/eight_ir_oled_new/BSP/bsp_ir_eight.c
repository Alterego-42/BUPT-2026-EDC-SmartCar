#include "bsp_ir_eight.h"
#include "oled.h"
volatile uint8_t IR_Data_number[8];//数字值
uint8_t oledbuf[13] = {0};  
void GPIO_setPins(GPIO_Regs* gpio, uint32_t pins,uint8_t val)
{
	if(val !=0)
	{
			 DL_GPIO_setPins(gpio, pins);
	}
	else
			DL_GPIO_clearPins(gpio, pins);
}

/**
 * @brief 读取八路红外传感器的状态
 * @param ir_results 长度为 8 的数组，用于存储每路的状态（0/1）
 */
void ReadEightIR(uint8_t ir_results[8]) {

    for (int channel = 0; channel < 8; channel++) {
        // 计算当前通道对应的 C、B、A 值（C 是最高位，A 是最低位）
        uint8_t c = (channel >> 2) & 0x01;
        uint8_t b = (channel >> 1) & 0x01;
        uint8_t a = channel & 0x01;       
        SET_CHANNEL(c, b, a); // 设置通道选择引脚  
        delay_ms(1);
        // 读取当前通道的红外状态，存入结果数组
        ir_results[channel] = READ_IR_OUT() ? 1 : 0;
				
    }
}


void OLED_SHOW_IR(void)
{
		//ReadEightIR(IR_Data_number);
				for (int i = 0; i < 8; i++) 
						OLED_ShowNum(20 + i * 10, 25, IR_Data_number[i], 1, 8, 1);        
				OLED_Refresh();	
	
}