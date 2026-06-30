#ifndef GIMBAL_ROUTE_H
#define GIMBAL_ROUTE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    int32_t distance_mm;
    uint8_t segment;
    int16_t yaw_center_cdeg;
    int16_t pitch_center_cdeg;
    int16_t yaw_error_cdeg;
    int16_t pitch_error_cdeg;
} gimbal_route_sample_t;

void gimbal_route_reset(void);
void gimbal_route_enable(bool enabled);
void gimbal_route_set_distance(int32_t distance_mm);
gimbal_route_sample_t gimbal_route_get_sample(int16_t yaw_position_cdeg,
    int16_t pitch_position_cdeg);

#endif
