#ifndef PCA9685_H
#define PCA9685_H

#include <stdbool.h>
#include <stdint.h>

bool pca9685_init(void);
bool pca9685_is_ready(void);
bool pca9685_all_off(void);
bool pca9685_set_motor4(uint8_t base_ch, const uint8_t pattern[4]);
uint32_t pca9685_error_count(void);

#endif
