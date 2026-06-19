#include "delay.h"
#include "bsp_ir_eight.h"

void GPIO_setPins(GPIO_Regs* gpio, uint32_t pins,uint8_t val)
{
	if(val != 0)
	{
			 DL_GPIO_setPins(gpio, pins);
	}
	else
				DL_GPIO_clearPins(gpio, pins);
}


void ReadEightIR(uint8_t ir_results[8]) {

    for (int channel = 0; channel < 8; channel++) {
        // 计算当前通道对应的 C、B、A 值（C 是最高位，A 是最低位）
        uint8_t c = (channel >> 2) & 0x01;
        uint8_t b = (channel >> 1) & 0x01;
        uint8_t a = channel & 0x01;       
        SET_CHANNEL(c, b, a); // 设置通道选择引脚  
        delay_ms(10); 
        
        // 读取当前通道的红外状态，存入结果数组
        ir_results[channel] = READ_IR_OUT() ? 1 : 0;
    }
}