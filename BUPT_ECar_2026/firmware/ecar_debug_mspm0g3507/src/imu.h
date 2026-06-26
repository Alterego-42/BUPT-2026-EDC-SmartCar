#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

void imu_init(void);
void imu_feed_byte(uint8_t byte);
bool imu_yaw_valid(void);
int16_t imu_get_yaw_x10(void);
int16_t imu_yaw_error_x10(int16_t target_x10);

#endif
