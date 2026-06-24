#ifndef PCA9685_H_
#define PCA9685_H_

#include <stdbool.h>
#include <stdint.h>

void pca9685_init(void);
bool pca9685_set_servo_angle(uint8_t channel, uint8_t angle_deg);
void laser_init(void);
void laser_set(bool on);

#endif
