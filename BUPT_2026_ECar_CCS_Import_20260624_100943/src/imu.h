#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>

void imu_init(void);
void imu_poll(uint32_t now_ms);
bool imu_get_yaw_x10(uint32_t now_ms, int16_t *yaw_x10);
int16_t imu_wrap_error_x10(int16_t target_x10, int16_t current_x10);

#endif
