#ifndef TASKS_H_
#define TASKS_H_

#include <stdbool.h>
#include <stdint.h>

#include "grayscale.h"

typedef enum {
    APP_TASK_NONE = 0,
    APP_TASK_AUTO_TRACE = 1,
    APP_TASK_STATIC_AIM = 2,
    APP_TASK_TRACE_AIM = 3,
    APP_TASK_DYNAMIC_AIM = 4,
    APP_TASK_SYNC_DRAW = 5,
    APP_TASK_FOUR_LAPS = 6,
    APP_TASK_SELFTEST = 7,
    APP_TASK_SENSOR_DASH = 8,
    APP_TASK_STRAIGHT_TEST = 9
} app_task_t;

typedef enum {
    TASK_STATUS_IDLE = 0,
    TASK_STATUS_RUNNING,
    TASK_STATUS_FINISHED,
    TASK_STATUS_ERROR
} task_status_t;

typedef struct {
    app_task_t task;
    task_status_t status;
    const char *state;
    uint8_t phase;
    uint8_t keypoint;
    grayscale_sample_t gray;
    int16_t yaw_x10;
    int32_t distance_mm;
    bool imu_fresh;
    bool k230_fresh;
} task_snapshot_t;

void tasks_init(void);
void tasks_start(app_task_t task, uint32_t now_ms);
void tasks_stop(void);
void tasks_update(uint32_t now_ms);
bool tasks_running(void);
bool tasks_done(void);
app_task_t tasks_current(void);
const char *tasks_name(app_task_t task);
app_task_t tasks_from_slot(uint8_t slot);
void tasks_snapshot(uint32_t now_ms, task_snapshot_t *snapshot);

#endif
