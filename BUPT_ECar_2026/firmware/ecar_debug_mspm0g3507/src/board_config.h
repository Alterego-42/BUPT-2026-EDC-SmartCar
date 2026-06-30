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
#define BOARD_MOTOR_START_PERCENT           (30)
#define BOARD_MOTOR_MAX_PERCENT             (85)
#define BOARD_MOTOR_SEARCH_PERCENT          (11)
#define BOARD_MOTOR_BRAKE_ON_STOP           (0)
/* Positive trim steers right. Negative trim steers left. */
#define BOARD_STRAIGHT_TRIM_PERCENT         (10)

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
#define BOARD_LINE_KP_PER_MILLE             (9)
#define BOARD_LINE_KD_PER_MILLE             (22)
#define BOARD_LINE_CONTROL_PERIOD_MS        (10U)
#define BOARD_LINE_START_DELAY_MS           (1000U)
#define BOARD_LINE_LOST_HOLD_MS             (250U)
#define BOARD_LINE_LOST_STOP_MS             (1200U)
#define BOARD_LINE_BRIDGE_MAX_MS            (6000U)
#define BOARD_LINE_BRIDGE_MAX_MM            (1400)
#define BOARD_LINE_BRIDGE_KEEP_LAST_MS      (600U)
#define BOARD_LINE_BRIDGE_KEEP_LAST_PERCENT (22)

/* ===== Optional IMU hold when line is missing briefly ===== */
#define BOARD_IMU_ENABLE_HEADING_HOLD       (1)
#define BOARD_IMU_VALID_MS                  (300U)
#define BOARD_HEADING_KP_PER_DEG            (6)
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
#define BOARD_KEYPOINT_GATE_BEFORE_MM       (300)
#define BOARD_KEYPOINT_GATE_AFTER_MM        (900)
#define BOARD_KEYPOINT_TRANSITION_STABLE_MS (60U)
#define BOARD_KEYPOINT_SIGNAL_COOLDOWN_MS   (500U)
#define BOARD_KEYPOINT_B_STOP_MS            (2500U)
#define BOARD_KEYPOINT_BEEP_MS              (180U)

/* ===== K230 laser aiming task =====
 * K230 sends "$K230,valid,yaw,pitch\n" at 115200 baud.
 * Laser and stepper coils stay off until gimbal_start() is called.
 */
#define BOARD_LASER_ACTIVE_HIGH             (1)
#define BOARD_K230_VISION_TIMEOUT_MS        (300U)
#define BOARD_GIMBAL_YAW_DEADBAND_CDEG      (2)
#define BOARD_GIMBAL_PITCH_DEADBAND_CDEG    (2)
#define BOARD_GIMBAL_YAW_MAX_ERROR_CDEG     (12)
#define BOARD_GIMBAL_PITCH_MAX_ERROR_CDEG   (12)
#define BOARD_GIMBAL_YAW_DIR                (1)
#define BOARD_GIMBAL_PITCH_DIR              (1)
#define BOARD_GIMBAL_YAW_STEP_CDEG          (9)
#define BOARD_GIMBAL_PITCH_STEP_CDEG        (9)
#define BOARD_GIMBAL_POSITION_LIMIT_CDEG    (20000)
#define BOARD_GIMBAL_YAW_STEP_MIN_MS        (3U)
#define BOARD_GIMBAL_YAW_STEP_MAX_MS        (14U)
#define BOARD_GIMBAL_PITCH_STEP_MIN_MS      (3U)
#define BOARD_GIMBAL_PITCH_STEP_MAX_MS      (14U)
#define BOARD_GIMBAL_YAW_STEPS_PER_FRAME    (4U)
#define BOARD_GIMBAL_PITCH_STEPS_PER_FRAME  (4U)
#define BOARD_GIMBAL_I2C_WRITES_PER_TASK    (2U)
#define BOARD_GIMBAL_LOCAL_FORCE_TEST       (0)
#define BOARD_GIMBAL_LOCAL_FORCE_YAW_CDEG   (300)
#define BOARD_GIMBAL_LOCAL_FORCE_PITCH_CDEG (0)
#define BOARD_GIMBAL_LOCAL_FORCE_FRAME_MS   (20U)
#define BOARD_GIMBAL_LOCAL_FORCE_STEP_MS    (8U)
#define BOARD_GIMBAL_RELEASE_COILS_IDLE     (0)
#define BOARD_GIMBAL_PCA_RETRY_MS           (500U)
#define BOARD_PCA9685_I2C_TIMEOUT_MS        (3U)

/* ===== Route feed-forward window for line + gimbal task =====
 * The route is A-B straight, B-C half arc, C-D straight, D-A half arc.
 * These center angles are relative to the gimbal pose at mode start.
 * Keep the defaults at zero until one lap is calibrated on the real car:
 * set the A/B/C/D/A2 yaw/pitch centers so the target stays inside view,
 * then K230 will only trim inside the sliding window below.
 */
#define BOARD_GIMBAL_ROUTE_ENABLE           (1)
#define BOARD_GIMBAL_ROUTE_LAP_MM           (BOARD_KEYPOINT_A_MM)
#define BOARD_GIMBAL_ROUTE_A_MM             (0)
#define BOARD_GIMBAL_ROUTE_B_MM             (BOARD_KEYPOINT_B_MM)
#define BOARD_GIMBAL_ROUTE_C_MM             (BOARD_KEYPOINT_C_MM)
#define BOARD_GIMBAL_ROUTE_D_MM             (BOARD_KEYPOINT_D_MM)
#define BOARD_GIMBAL_ROUTE_A2_MM            (BOARD_KEYPOINT_A_MM)
#define BOARD_GIMBAL_ROUTE_YAW_A_CDEG       (0)
#define BOARD_GIMBAL_ROUTE_YAW_B_CDEG       (0)
#define BOARD_GIMBAL_ROUTE_YAW_C_CDEG       (0)
#define BOARD_GIMBAL_ROUTE_YAW_D_CDEG       (0)
#define BOARD_GIMBAL_ROUTE_YAW_A2_CDEG      (0)
#define BOARD_GIMBAL_ROUTE_PITCH_A_CDEG     (0)
#define BOARD_GIMBAL_ROUTE_PITCH_B_CDEG     (0)
#define BOARD_GIMBAL_ROUTE_PITCH_C_CDEG     (0)
#define BOARD_GIMBAL_ROUTE_PITCH_D_CDEG     (0)
#define BOARD_GIMBAL_ROUTE_PITCH_A2_CDEG    (0)
#define BOARD_GIMBAL_ROUTE_YAW_WINDOW_CDEG  (700)
#define BOARD_GIMBAL_ROUTE_PITCH_WINDOW_CDEG (450)
#define BOARD_GIMBAL_ROUTE_VISION_TRIM_CDEG (18)
#define BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG (6000)
#define BOARD_GIMBAL_ROUTE_POSITION_LIMIT_CDEG (8000)

/* ===== Mode 2: fixed-point target shooting =====
 * Start with the yaw axis physically placed at the 0-degree reference.
 * The search path is CCW 180 degrees, then CW 360 degrees. Flip
 * BOARD_TARGET_MODE_CCW_SIGN if the physical direction is reversed.
 */
#define BOARD_TARGET_MODE_CYCLES            (3U)
#define BOARD_TARGET_MODE_HALF_TURN_TEST    (0)
#define BOARD_TARGET_MODE_CCW_SIGN          (-1)
/* Confirmed on the real gimbal: 1024 updates is about 90 degrees, so mode 2
 * applies hard step-count limits below.  The soft cdeg target is deliberately
 * past 180 degrees so the step limit, not the cdeg estimate, ends each sweep.
 */
#define BOARD_TARGET_MODE_SEARCH_HALF_CDEG  (18432)
#define BOARD_TARGET_MODE_SEARCH_SOFT_CDEG  (BOARD_GIMBAL_POSITION_LIMIT_CDEG)
#define BOARD_TARGET_MODE_SEARCH_HALF_STEPS (2048L)
#define BOARD_TARGET_MODE_SEARCH_FULL_STEPS (4096L)
#define BOARD_TARGET_MODE_SEARCH_TOL_CDEG   (18)
#define BOARD_TARGET_MODE_ZERO_TOL_CDEG     (20)
#define BOARD_TARGET_MODE_SCAN_STEP_MS      (18U)
#define BOARD_TARGET_MODE_RETURN_STEP_MS    (6U)
#define BOARD_TARGET_MODE_AIM_STEP_MS       (12U)
#define BOARD_TARGET_MODE_AIM_MAX_CDEG      (8)
#define BOARD_TARGET_MODE_AIM_DEADBAND_CDEG (1)
#define BOARD_TARGET_MODE_AIM_STABLE_CDEG   (2)
#define BOARD_TARGET_MODE_AIM_MIN_MS        (900U)
#define BOARD_TARGET_MODE_AIM_STABLE_MS     (700U)
#define BOARD_TARGET_MODE_VALID_STABLE_MS   (160U)
#define BOARD_TARGET_MODE_VALID_MIN_FRAMES  (4U)
#define BOARD_TARGET_MODE_CANDIDATE_HOLD_MS (260U)
#define BOARD_TARGET_MODE_LOST_MS           (500U)
#define BOARD_TARGET_MODE_REPLACE_WAIT_MS   (7000U)
#define BOARD_TARGET_MODE_SEARCH_RETRY_MS   (600U)

/* ===== Button and feedback ===== */
#define BOARD_BUTTON_DEBOUNCE_MS            (35U)
#define BOARD_BUTTON_LONG_MS                (800U)
#define BOARD_MODE_COUNT                    (7U)

#endif
