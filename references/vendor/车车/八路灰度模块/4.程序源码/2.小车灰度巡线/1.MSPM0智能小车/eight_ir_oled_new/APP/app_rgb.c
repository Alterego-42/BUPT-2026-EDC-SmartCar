#include "app_rgb.h"

//RGB简单灯效


//#define 	Red_RGB     '2'//按键前 Before pressing the button
//#define 	Green_RGB    '3'//按键后 After pressing the button
//#define 	Blue_RGB    '4'//按键左 Left button
////#define 	Yellow_RGB   '4'//按键右 Right button
////#define 	Cyan_RGB    '5'//按键停 Button stop
//#define 	OFF    '8'//按键停 Button stop

extern uint8_t ProtocolString[80];//引入备份数据区 Introducing backup data area
static void set_ALL_RGB_COLOR(unsigned long color)
{
    rgb_SetColor(Left_RGB,color);
    rgb_SetColor(Right_RGB,color);
}


void app_color()
{
	
		if (ProtocolString[8] == '2')      //小车左旋 Left rotation of the car
	{
		 Control_RGB_ALL(Red_RGB);
	}
 else if (ProtocolString[8] == '3') //小车右旋 Car turning right
	{
		Control_RGB_ALL(Green_RGB);
	}
	else if (ProtocolString[8] == '4')
	{
		Control_RGB_ALL(Blue_RGB);
	}
else {

	Control_RGB_ALL(OFF);


}



}

void Control_RGB_ALL(RGB_Color_t color)
{
    switch(color)
    {
        case    Red_RGB:     set_ALL_RGB_COLOR(RED);break;
        case    Green_RGB:   set_ALL_RGB_COLOR(GREEN);break;
        case    Blue_RGB:    set_ALL_RGB_COLOR(BLUE);break;
        case    Yellow_RGB:  set_ALL_RGB_COLOR(YELLOW);break;
        case    Purple_RGB:  set_ALL_RGB_COLOR(PURPLE); break;  
        case    Cyan_RGB:    set_ALL_RGB_COLOR(CYAN);break;
        case    OFF  :       set_ALL_RGB_COLOR(BLACK);break;
        
        default : return;
        
    }
    
    rgb_SendArray();//必须发送,才显示
    delay_ms(200);
    
    
}

