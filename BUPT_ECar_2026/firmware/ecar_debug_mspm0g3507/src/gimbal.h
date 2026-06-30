#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
} gimbal_pose_t;

typedef struct {
    int32_t yaw_steps;
    int32_t pitch_steps;
} gimbal_steps_t;

void gimbal_init(void);
void gimbal_start(void);
void gimbal_stop(void);
void gimbal_task(void);
void gimbal_sweep_test_task(void);
bool gimbal_is_active(void);
void gimbal_zero_pose(void);
gimbal_pose_t gimbal_get_pose(void);
gimbal_steps_t gimbal_get_steps(void);
bool gimbal_drive_to_cdeg(int16_t yaw_target_cdeg, int16_t pitch_target_cdeg,
    uint16_t yaw_tolerance_cdeg, uint16_t pitch_tolerance_cdeg,
    uint16_t step_interval_ms, uint8_t writes_per_task);
bool gimbal_drive_error_cdeg(int16_t yaw_error_cdeg, int16_t pitch_error_cdeg,
    uint16_t deadband_cdeg, uint16_t step_interval_ms, uint8_t writes_per_task);

#endif
