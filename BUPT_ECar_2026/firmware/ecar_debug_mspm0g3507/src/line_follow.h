#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#include "grayscale.h"

typedef struct {
    bool active;
    bool lost;
    bool fault;
    grayscale_sample_t gray;
    int16_t left_percent;
    int16_t right_percent;
    int16_t correction_percent;
    int32_t distance_mm;
} line_follow_status_t;

void line_follow_reset(void);
void line_follow_reset_at(int32_t distance_mm);
line_follow_status_t line_follow_update(uint32_t now_ms, int32_t distance_mm);
line_follow_status_t line_follow_update_ex(uint32_t now_ms, int32_t distance_mm, bool bridge_gap);
void line_follow_stop(void);

#endif
