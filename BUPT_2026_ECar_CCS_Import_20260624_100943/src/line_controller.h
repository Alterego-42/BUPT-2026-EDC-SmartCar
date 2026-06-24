#ifndef LINE_CONTROLLER_H_
#define LINE_CONTROLLER_H_

#include "grayscale.h"
#include <stdint.h>

typedef enum {
    LINE_MODE_GRAY = 0,
    LINE_MODE_IMU_HOLD,
    LINE_MODE_SEARCH,
    LINE_MODE_STOP
} line_mode_t;

typedef struct {
    int32_t left_percent;
    int32_t right_percent;
    line_mode_t mode;
} line_output_t;

void line_controller_reset(void);
line_output_t line_controller_gray(const grayscale_sample_t *sample);
line_output_t line_controller_heading_hold(int16_t yaw_error_x10);
line_output_t line_controller_search(int16_t last_error);
line_output_t line_controller_stop(void);
int16_t line_controller_last_error(void);

#endif
