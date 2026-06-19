#include "app_k210.h"
float g_K210x_tarea;
float g_K210x_median;
float pid_output_k210x = 0;
PID_t pid_k210_x;
PID_t pid_k210_y;
K210_Data_t K210_data;
volatile u8 k210_buf[64];
uint8_t  number_count ;
//中心偏移调节小车位置 The center offset adjusts the position of the trolley
uint8_t off_centering =10;



//增量PID Incremental PID
#define K210X_PID_KP       (20)
#define K210X_PID_KI       (0)
#define K210X_PID_KD       (2)

#define K210Y_PID_KP       (8)
#define K210Y_PID_KI       (0)
#define K210Y_PID_KD       (2)

#define K210X_SPEED 		400


#define AREA_DEAD_ZONE 2       // 面积误差死区  Dead zone of area error
#define POSITION_DEAD_ZONE 10    // 位置误差死区 Position error dead zone
#define MAX_PID_OUTPUT_X 150    // X方向PID输出限幅 PID output limiting in the X direction
#define MAX_PID_OUTPUT_Y 200    // Y方向PID输出限幅 Y-direction PID output limiting
#define FILTER_COEFFICIENT 1  //滤波系数 filter coefficient
#define TARGET_RANGE 25     
static int filtered_x = 0;
static int filtered_area = 0;


// 初始化K210X轴 PID参数 Initialize the K210X axis PID parameter
void APP_K210X_Init(void)
{
	pid_k210_x.target_val = 0.0;
	pid_k210_x.pwm_output = 0.0;
	pid_k210_x.err = 0.0;
	pid_k210_x.err_last = 0.0;
	pid_k210_x.err_next = 0.0;
	pid_k210_x.integral = 0.0;

	pid_k210_x.Kp = K210X_PID_KP;
	pid_k210_x.Ki = K210X_PID_KI;
	pid_k210_x.Kd = K210X_PID_KD;
	
	pid_k210_y.Kp = K210Y_PID_KP;
	pid_k210_y.Ki = K210Y_PID_KI;
	pid_k210_y.Kd = K210Y_PID_KD;
	
	//初始化 Initialization
	K210_data.k210_X = 160 ;
	K210_data.k210_Y = 120;
	K210_data.k210_AREA = 12;  //k210_W *k210_H /100
}

// 调用增量式pid计算,返回计算的结果 Call incremental pid calculation and return the result of calculation.
// float actual_value: 当前的误差 Current error
float APP_K210X_PID_Calc(float actual_value)
{
    return PID_Incre_Calc(&pid_k210_x, actual_value);
}

// 根据传感器的值反馈，使用pid控制小车电机做巡线运动
//  According to the feedback of the sensor, pid is used to control the car motor to patrol the line.
void APP_K210X_Line_PID(void)
{
	//这里位置可能需要根据实际情况来进行修改摄像头的中间并不一定是小车的中间
	//The position here may need to be modified according to the actual situation. The middle of the camera is not necessarily the middle of the car

	g_K210x_median=160+off_centering-k210_msg.x-(k210_msg.w/2); //k210屏幕的x轴是320 所以中间值为160 The x-axis of the k210 screen is 320, so the median value is 160.

	pid_output_k210x = (int)(APP_K210X_PID_Calc(g_K210x_median));

	Motion_Ctrl(K210X_SPEED, 0, pid_output_k210x);

}

void Color_Trace(int x,int y,int w,int h)
{
    
    filtered_x = (filtered_x * (1-FILTER_COEFFICIENT)) + (x *FILTER_COEFFICIENT); // 一阶低通滤波 First-order low-pass filtering
    filtered_area = (filtered_area *(1-FILTER_COEFFICIENT)) + ((w * h / 100) * FILTER_COEFFICIENT); // 面积也滤波 Area also filters
		int pid_output_y ;
    int now_area = filtered_area; // 使用滤波后的面积 Use the filtered area
    int now_x = filtered_x;       // 使用滤波后的X坐标 Use the filtered X-coordinate

		int16_t err_area = 0;
		int16_t err_y = 0;
	
    // X方向控制：计算位置偏差  X-direction control: Calculate position deviation
    int16_t err_x = K210_data.k210_X  - now_x;
    // 位置误差死区处理  Position error dead zone handling
    if(err_x < POSITION_DEAD_ZONE && err_x > -POSITION_DEAD_ZONE) err_x = 0;
    int pid_output_x = PID_Incre_Color_Calc(&pid_k210_x,err_x); 
    // 输出限幅
    if(pid_output_x > MAX_PID_OUTPUT_X) pid_output_x = MAX_PID_OUTPUT_X;
    if(pid_output_x < -MAX_PID_OUTPUT_X) pid_output_x = -MAX_PID_OUTPUT_X;
		
	
		// Y方向控制：计算面积偏差 Y-direction control: Calculate the area deviation
		 err_area = K210_data.k210_AREA - now_area;
		// 面积误差死区处理 Area error dead zone handling
		if(err_area < AREA_DEAD_ZONE && err_area > -AREA_DEAD_ZONE) err_area = 0;
			pid_output_y = PID_Incre_Color_Calc(&pid_k210_y,err_area); 
		// 限幅
		if(pid_output_y > MAX_PID_OUTPUT_Y) pid_output_y = MAX_PID_OUTPUT_Y;
		if(pid_output_y < -MAX_PID_OUTPUT_Y) pid_output_y = -MAX_PID_OUTPUT_Y;
			
			//最终控制合成 (Y: 前后, X: 左右)  Final control synthesis (Y: front and back, X: left and right)
			Motion_Car_Control(pid_output_y, 0, -pid_output_x); 
}




void BSP_Loop(void)
{
	if( keyHandle[0].key_event_short == 1)
	{
		g_key_flag = !g_key_flag;
		keyHandle[0].key_event_short = 0;
	}

	if(g_key_flag == 0)
	{

		Motor_Stop(1);

		}

	else
	{
		Motor_Run(200,200);
		if(k210_msg.class_n == 11)
		{
			//解析到的字符串对比 Comparison of the parsed strings
			if(k210_msg.id == '8')
				{
						number_count++;
						if(number_count >=3)
						{
							sprintf((char *)k210_buf,"recongnize_num:%c",k210_msg.id);
							OLED_ShowString(0, 10, (uint8_t *)k210_buf, 8, 1);
							OLED_Refresh();
							Motor_Back(300,300);
							delay_ms(1000);
							delay_ms(1000);
							number_count =0;
						}
				}
			
		}
		//显示识别到的内容 Display the recognized content
			sprintf((char *)k210_buf,"recongnize_num:%c",k210_msg.id);
			OLED_ShowString(0, 10, (uint8_t *)k210_buf, 8, 1);
			k210_msg.class_n =0;
			k210_msg.id = '-';
	}

}
