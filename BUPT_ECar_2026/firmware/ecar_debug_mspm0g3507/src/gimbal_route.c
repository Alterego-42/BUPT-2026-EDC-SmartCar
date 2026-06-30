#include "gimbal_route.h"

#include "board_config.h"
#include "util.h"

typedef struct {
    int32_t distance_mm;
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
} route_node_t;

static bool g_enabled;
static int32_t g_distance_mm;

static const route_node_t k_nodes[] = {
    {BOARD_GIMBAL_ROUTE_A_MM, BOARD_GIMBAL_ROUTE_YAW_A_CDEG,
        BOARD_GIMBAL_ROUTE_PITCH_A_CDEG},
    {BOARD_GIMBAL_ROUTE_B_MM, BOARD_GIMBAL_ROUTE_YAW_B_CDEG,
        BOARD_GIMBAL_ROUTE_PITCH_B_CDEG},
    {BOARD_GIMBAL_ROUTE_C_MM, BOARD_GIMBAL_ROUTE_YAW_C_CDEG,
        BOARD_GIMBAL_ROUTE_PITCH_C_CDEG},
    {BOARD_GIMBAL_ROUTE_D_MM, BOARD_GIMBAL_ROUTE_YAW_D_CDEG,
        BOARD_GIMBAL_ROUTE_PITCH_D_CDEG},
    {BOARD_GIMBAL_ROUTE_A2_MM, BOARD_GIMBAL_ROUTE_YAW_A2_CDEG,
        BOARD_GIMBAL_ROUTE_PITCH_A2_CDEG},
};

static int32_t wrap_distance(int32_t distance_mm)
{
    int32_t lap_mm = BOARD_GIMBAL_ROUTE_LAP_MM;

    if (lap_mm <= 0) {
        return distance_mm;
    }

    distance_mm %= lap_mm;
    if (distance_mm < 0) {
        distance_mm += lap_mm;
    }
    return distance_mm;
}

static int16_t clamp_center(int32_t value)
{
    return clamp_i16(value, -BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG,
        BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG);
}

static int16_t interpolate_cdeg(int16_t start, int16_t end,
    int32_t num, int32_t den)
{
    int32_t value;

    if (den <= 0) {
        return clamp_center(end);
    }

    value = (int32_t)start + (((int32_t)(end - start) * num) + (den / 2)) / den;
    return clamp_center(value);
}

static int32_t smooth_arc_num(int32_t num, int32_t den)
{
    int64_t q15;
    int64_t smooth;

    if (den <= 0) {
        return num;
    }

    q15 = ((int64_t)num * 32767LL) / den;
    smooth = (q15 * q15 * (3LL * 32767LL - 2LL * q15)) /
        (32767LL * 32767LL);
    return (int32_t)((smooth * den) / 32767LL);
}

static int16_t window_error(int16_t center_cdeg, int16_t position_cdeg,
    int16_t half_window_cdeg)
{
    int32_t diff = (int32_t)center_cdeg - position_cdeg;

    if (half_window_cdeg < 0) {
        half_window_cdeg = (int16_t)-half_window_cdeg;
    }

    if (diff > half_window_cdeg) {
        return clamp_i16(diff - half_window_cdeg,
            -BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG,
            BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG);
    }
    if (diff < -(int32_t)half_window_cdeg) {
        return clamp_i16(diff + half_window_cdeg,
            -BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG,
            BOARD_GIMBAL_ROUTE_CENTER_LIMIT_CDEG);
    }
    return 0;
}

static gimbal_route_sample_t sample_at_distance(int32_t distance_mm)
{
    gimbal_route_sample_t sample = {0};
    uint8_t i;

    sample.distance_mm = wrap_distance(distance_mm);

    for (i = 0U; i < (uint8_t)((sizeof(k_nodes) / sizeof(k_nodes[0])) - 1U); i++) {
        const route_node_t *a = &k_nodes[i];
        const route_node_t *b = &k_nodes[i + 1U];

        if (sample.distance_mm >= a->distance_mm &&
            sample.distance_mm < b->distance_mm) {
            int32_t num = sample.distance_mm - a->distance_mm;
            int32_t den = b->distance_mm - a->distance_mm;

            if (i == 1U || i == 3U) {
                num = smooth_arc_num(num, den);
            }

            sample.segment = i;
            sample.yaw_center_cdeg = interpolate_cdeg(a->yaw_cdeg, b->yaw_cdeg,
                num, den);
            sample.pitch_center_cdeg = interpolate_cdeg(a->pitch_cdeg, b->pitch_cdeg,
                num, den);
            return sample;
        }
    }

    sample.segment = 3U;
    sample.yaw_center_cdeg = clamp_center(BOARD_GIMBAL_ROUTE_YAW_A2_CDEG);
    sample.pitch_center_cdeg = clamp_center(BOARD_GIMBAL_ROUTE_PITCH_A2_CDEG);
    return sample;
}

void gimbal_route_reset(void)
{
    g_enabled = false;
    g_distance_mm = 0;
}

void gimbal_route_enable(bool enabled)
{
    g_enabled = enabled && (BOARD_GIMBAL_ROUTE_ENABLE != 0);
}

void gimbal_route_set_distance(int32_t distance_mm)
{
    g_distance_mm = distance_mm;
}

gimbal_route_sample_t gimbal_route_get_sample(int16_t yaw_position_cdeg,
    int16_t pitch_position_cdeg)
{
    gimbal_route_sample_t sample = sample_at_distance(g_distance_mm);

    if (!g_enabled) {
        return sample;
    }

    sample.active = true;
    sample.yaw_error_cdeg = window_error(sample.yaw_center_cdeg,
        yaw_position_cdeg, BOARD_GIMBAL_ROUTE_YAW_WINDOW_CDEG);
    sample.pitch_error_cdeg = window_error(sample.pitch_center_cdeg,
        pitch_position_cdeg, BOARD_GIMBAL_ROUTE_PITCH_WINDOW_CDEG);
    return sample;
}
