#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t avg_ticks;
    int32_t distance_mm;
} encoder_snapshot_t;

void encoder_init(void);
void encoder_reset(void);
encoder_snapshot_t encoder_get(void);

#endif
