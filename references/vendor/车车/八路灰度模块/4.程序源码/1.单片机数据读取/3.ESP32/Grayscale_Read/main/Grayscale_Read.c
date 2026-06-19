//Grayscale Sensor / ESP32S3
//      5V		        5V
//      GND		        GND
//      AD0		        RX1(35)
//      AD1		        TX1(36)
//      AD2		        SCL(37)
//      OUT		        SDA(38)
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "grayscale_sensor.h"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))// 毫秒延时宏 | Millisecond delay macro

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
int i;
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
        "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
    };


void Grayscale_ReadData_Task(void *arg) {
	while (1) {
		Grayscale_Sensor_Read_All(g_sensor_data);
        
        for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
        {
            printf("[ %s:%d ]", sensor_labels[i], g_sensor_data[i]);
        }
        printf("\n");
		//当X1的灯亮起时，值为1 / When the X1 light is on, the value is 1
		delay_ms(100);// 防止任务卡死 | Preventing tasks from getting stuck

	}
}

void app_main(void)
{
    Grayscale_Sensor_Init();	

    xTaskCreate(
        Grayscale_ReadData_Task, 		// 任务函数	Task Function
        "GrayscaleReadData",         // 任务名称	Task Name
        4096,                   // 堆栈大小（字节）	Stack size (bytes)
        NULL, 
        2,                      // 优先级（数值越大优先级越高）	Priority (the larger the value, the higher the priority)
        NULL
    );
}
