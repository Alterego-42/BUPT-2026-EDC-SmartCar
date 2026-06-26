#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

static inline int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static inline int16_t clamp_i16(int32_t value, int16_t lo, int16_t hi)
{
    return (int16_t) clamp_i32(value, lo, hi);
}

static inline int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

#endif
