//Grayscale Sensor / STM32
//		5V			5V
//		GND			GND
//		AD0			PA0
//		AD1			PA1
//		AD2			PA2
//		OUT			PA3
#include "AllHeader.h"

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
int i;
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
        "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
    };

int main(void)
{	
	bsp_init();
	
	TIM3_Init();
	
	Grayscale_Sensor_Init();
	while(1)
	{
		Grayscale_Sensor_Read_All(g_sensor_data);
        
        for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
        {
            printf("[ %s:%d ]", sensor_labels[i], g_sensor_data[i]);
        }
        printf("\n");
		//当X1的灯亮起时，值为1 / When the X1 light is on, the value is 1
		delay_ms(60);
	}
	
}

