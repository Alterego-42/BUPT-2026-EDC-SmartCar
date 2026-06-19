#include "app_speech.h"

//串口解析部分 115200 波特率

uint8_t recv_buf[5]={0}; //接收buf Receive buf
char cmd[5];
uint8_t g_index = 0;

uint8_t g_speech=0;//语音标志
uint8_t g_speech_cmd=0xFF;//命令标识 

void Processing_Data(uint8_t RXdata)
{
	recv_buf[g_index++] =RXdata;
	
	if (g_index >=5)
		{
			g_index = 0;
            g_speech = 1;
            g_speech_cmd = recv_buf[3];
		}

}


void Write_Data(uint8_t dat)
{

	cmd[0] = 0xAA;
	cmd[1] = 0x55;
	cmd[2] = 0xFF;
	cmd[3] = dat;  // 直接使用传入的数据
	cmd[4] = 0xFB;

	for(int j = 0; j<5 ;j++)
	{
		uart3_send_char(cmd[j]);
	}
   

}


void USER_Speech_Contorl_Car(void)
{
    if(g_speech == 0)return;
    
    switch(g_speech_cmd)
    {
        case Car_STOP: 
        case Car_STOP2: 
        case Car_STOP3:    Motion_Stop(0);break;
        
        case Car_Forword:  wheel_State(MOTION_RUN,200);break;
        case Car_Back:     wheel_State(MOTION_BACK,200);break;
        case Car_Left:     wheel_State(MOTION_LEFT,200);break;
        case Car_Right:    wheel_State(MOTION_RIGHT,200);break;
        case Car_SpinLeft: wheel_State(MOTION_SPIN_LEFT,200);break;
        case Car_SpinRight:wheel_State(MOTION_SPIN_RIGHT,200);break;
        default:break;
    }
    
//    Write_Data(g_speech_cmd); //会自动播报
    g_speech_cmd = 0xFF;
    g_speech = 0;
}

