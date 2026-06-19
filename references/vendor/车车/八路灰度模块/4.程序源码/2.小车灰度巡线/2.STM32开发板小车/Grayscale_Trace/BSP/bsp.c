#include "bsp.h"

//硬件初始化 Board hardware initialization
void bsp_init(void)
{
	SystemInit();//系统时钟初始化 System clock initialization
	
	delay_init();//延迟初始化 Delay initialization
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置为优先级组2 Set to priority group 2
	
	__disable_irq();//关中断 Disable interrupts
	
	//串口初始化 UART initialization
	USART1_init(115200);
    	
	
#if ENABLE_MOTOR
	//电机初始化 Motor initialization
	motor_gpio_init();//引脚初始化 Pin initialization
	motor_pwm_init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);
	motor_encode_init();//霍尔部分初始化 Encoder part initialization
	PID_Param_Init();//PID初始化 PID initialization
#endif	


	__enable_irq();//开中断 Enable interrupts
	
	usart_irq_rx_enable(); 
	
}



//JTAG模式设置,用于设置JTAG的模式 JTAG mode setting, used to set the mode of JTAG
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00
void Bsp_JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1 << 0;	  //开启辅助时钟 Enable AFIO clock
	AFIO->MAPR &= 0XF8FFFFFF; //清除MAPR的[26:24]位 Clear bits [26:24] of MAPR
	AFIO->MAPR |= temp;		  //设置jtag模式 Set jtag mode
}
