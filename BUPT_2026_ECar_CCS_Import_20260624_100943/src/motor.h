#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void motor_init(void);
void motor_enable(void);
void motor_disable(void);
void motor_stop(void);
void motor_set_percent(int32_t left_percent, int32_t right_percent);

#endif
