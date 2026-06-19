#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "AllHeader.h"

typedef enum {
    FORWARD,  // 正向
    REVERSAL  // 反向
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //保存实时计数值
    volatile int count;         				//根据定时器时间更新的计数值
    volatile  ENCODER_DIR dir;            	 //旋转方向
    volatile long long ALLcount;  //开机到现在总的编码器计数
} ENCODER_RES;


void encoder_init(void);
ENCODER_DIR get_encoderL_dir(void);
ENCODER_DIR get_encoderR_dir(void);
extern volatile u8 encoder_buf[64];
extern volatile  ENCODER_RES motorL_encoder;
extern volatile  ENCODER_RES motorR_encoder;
extern volatile int motor_distance;
extern volatile uint64_t timer_count ;

extern volatile u8 encoder_odometry_flag;
void Encoder_Get_ALL(int *Encoder_all);
void Encoder_Get_Temp(int *Encoder_temp);
void encoder_update(void);
float Encoder_Get_Distance(void);
void Odm_Stop(void);
#endif
