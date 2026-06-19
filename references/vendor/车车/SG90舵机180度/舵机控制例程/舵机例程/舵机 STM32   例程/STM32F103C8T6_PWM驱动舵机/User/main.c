//PB9接舵机信号线（橙色线）
//5V供电，红正棕负

#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "Key.h"
#include "Servo.h"

uint8_t i;
uint8_t KeyNum;
float Angle=0;

int main(void)
{
	OLED_Init();
	Servo_Init();
	Key_Init();
	Servo_SetAngle(50);
	while (1)
	{
		
		KeyNum = Key_GetNum();
		if(KeyNum==1)
		{
			Angle += 15;
			if(Angle > 180)
			{
				Angle = 0;
			}
		}
		if(KeyNum==2)
		{
			Angle -= 15;
			if(Angle <= 0)
			{
				Angle = 180;
			}
		}
		Servo_SetAngle(Angle);
		

//		for (i = 0; i <= 100; i++)
//		{
//			PWM_SetCompare4(i);
//			Delay_ms(10);
//		}
//		for (i = 0; i <= 100; i++)
//		{
//			PWM_SetCompare4(100 - i);
//			Delay_ms(10);
//		}
	}
}
