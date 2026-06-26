#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_standby(uint8_t enable);
void motor_stop(void);
void motor_brake(void);
void motor_set_percent(int16_t left_percent, int16_t right_percent);
void motor_point_left(int16_t percent);
void motor_point_right(int16_t percent);

#endif
