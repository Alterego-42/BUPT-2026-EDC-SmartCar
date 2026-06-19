 #include "AllHeader.h"
volatile uint32_t systick_counter = 0;

//20ms定时器
void Timer_20ms_Init(void)
{
    //打开20ms定时器
    NVIC_ClearPendingIRQ(TIMER_20ms_INST_INT_IRQN);
	  NVIC_EnableIRQ(TIMER_20ms_INST_INT_IRQN);
		DL_TimerA_startCounter(TIMER_20ms_INST);
}
void Timer_Display_Init(void)
{
    //打开显示定时器
    NVIC_ClearPendingIRQ(TIMER_DISPLAY_INST_INT_IRQN);
	  NVIC_EnableIRQ(TIMER_DISPLAY_INST_INT_IRQN);
		DL_TimerA_startCounter(TIMER_DISPLAY_INST);
}
u8 time_cnt = 0;
u8 gled_cnt = 0;

void TIMER_20ms_INST_IRQHandler(void)
{
    //20ms归零中断触发
	if( DL_TimerA_getPendingInterrupt(TIMER_20ms_INST) == DL_TIMER_IIDX_ZERO )
	{
				encoder_update();
				Motion_Handle(); //小车测速
				//ReadEightIR(IR_Data_number);
				gled_cnt++;
        if(gled_cnt>=10)
        {
            gled_cnt=0;
            DL_GPIO_togglePins(LED_PORT,LED_MCU_PIN);
        }
	}

}
void TIMER_DISPLAY_INST_IRQHandler(void)
{
    //50ms归零中断触发
	if( DL_TimerA_getPendingInterrupt(TIMER_DISPLAY_INST) == DL_TIMER_IIDX_ZERO )
	{
		ReadEightIR(IR_Data_number);
			OLED_SHOW_IR();
	}

}