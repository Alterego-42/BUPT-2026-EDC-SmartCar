#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void encoder_init(void);
void encoder_reset(void);
void encoder_get_counts(int32_t *lr_count, int32_t *rr_count);
int32_t encoder_get_average_distance_mm(void);
void encoder_handle_gpio_interrupts(void);

#endif
