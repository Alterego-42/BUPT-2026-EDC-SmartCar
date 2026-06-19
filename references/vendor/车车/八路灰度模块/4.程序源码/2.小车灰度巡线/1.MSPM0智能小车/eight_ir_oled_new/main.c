#include "AllHeader.h"

int main(void)
{
	// 系统配置初始化  System configuration initialization
	SYSCFG_DL_init();       
	 // OLED显示屏初始化  OLED display initialization
	OLED_Init();

	Init_Motor_PWM();
	//Timer_Display_Init();
  encoder_init();
//  电机控速pid初始化
	PID_Param_Init();
	OLED_ShowString(0,25,"IR:",8,1);
	while (1)                
  
		{			
		  LineWalking();
			OLED_SHOW_IR();

		}
}	

	