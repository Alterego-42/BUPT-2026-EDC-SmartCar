#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/* ===== Hardware polarity ===== */
#define BOARD_MOTOR_LEFT_INVERT             (0)
#define BOARD_MOTOR_RIGHT_INVERT            (0)
#define BOARD_ENCODER_LEFT_INVERT           (0)
#define BOARD_ENCODER_RIGHT_INVERT          (0)
#define BOARD_STEERING_INVERT               (0)
#define BOARD_BEEP_ACTIVE_HIGH              (0)

/* Set to 0 if black line reads low on the grayscale module OUT pin. */
#define BOARD_GRAY_BLACK_IS_HIGH            (1)
#define BOARD_GRAY_REVERSE_ORDER            (0)

/* ===== Motor safety limits ===== */
#define BOARD_PWM_PERIOD_COUNTS             (1600U)
#define BOARD_MOTOR_TEST_PERCENT            (12)
#define BOARD_MOTOR_START_PERCENT           (24)
#define BOARD_MOTOR_MAX_PERCENT             (55)
#define BOARD_MOTOR_SEARCH_PERCENT          (11)
#define BOARD_MOTOR_BRAKE_ON_STOP           (0)
/* Positive trim steers right. Negative trim steers left. */
#define BOARD_STRAIGHT_TRIM_PERCENT         (3)

/* ===== Encoder geometry =====
 * Defaults come from the vendor MSPM0 car example. Recalibrate on the real car.
 */
#define BOARD_WHEEL_CIRCUMFERENCE_MM        (210)
#define BOARD_ENCODER_TICKS_PER_REV         (1170)
#define BOARD_ENCODER_SELFTEST_MIN_TICKS    (4)

/* ===== Line-following control =====
 * error is -3500..+3500. correction percent ~= error * KP / 1000
 * plus derivative * KD / 1000.
 */
#define BOARD_LINE_KP_PER_MILLE             (7)
#define BOARD_LINE_KD_PER_MILLE             (18)
#define BOARD_LINE_CONTROL_PERIOD_MS        (10U)
#define BOARD_LINE_START_DELAY_MS           (1000U)
#define BOARD_LINE_LOST_HOLD_MS             (250U)
#define BOARD_LINE_LOST_STOP_MS             (1200U)
#define BOARD_LINE_BRIDGE_MAX_MS            (6000U)
#define BOARD_LINE_BRIDGE_MAX_MM            (1400)
#define BOARD_LINE_BRIDGE_KEEP_LAST_MS      (500U)
#define BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT (18)

/* ===== Optional IMU hold when line is missing briefly ===== */
#define BOARD_IMU_ENABLE_HEADING_HOLD       (1)
#define BOARD_IMU_VALID_MS                  (300U)
#define BOARD_HEADING_KP_PER_DEG            (5)
#define BOARD_HEADING_INVERT                (0)
#define BOARD_BRIDGE_HEADING_OFFSET_X10     (222)

/* ===== Keypoint detection for B, C, D, A from the start point A.
 * Keypoints are stable white/black transitions. Distances are only gates
 * to reject grayscale jitter before the expected transition area.
 */
#define BOARD_KEYPOINT_B_MM                 (1000)
#define BOARD_KEYPOINT_C_MM                 (2257)
#define BOARD_KEYPOINT_D_MM                 (3257)
#define BOARD_KEYPOINT_A_MM                 (4513)
#define BOARD_KEYPOINT_GATE_BEFORE_MM       (350)
#define BOARD_KEYPOINT_TRANSITION_STABLE_MS (60U)
#define BOARD_KEYPOINT_B_STOP_MS            (2500U)
#define BOARD_KEYPOINT_BEEP_MS              (180U)

/* ===== Button and feedback ===== */
#define BOARD_BUTTON_DEBOUNCE_MS            (35U)
#define BOARD_BUTTON_LONG_MS                (800U)
#define BOARD_MODE_COUNT                    (6U)

#endif
