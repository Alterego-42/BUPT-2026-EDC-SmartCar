#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t raw_mask;
    uint8_t black_mask;
    uint8_t black_count;
    int16_t error;
    bool line_seen;
} grayscale_sample_t;

void grayscale_init(void);
grayscale_sample_t grayscale_read(void);

#endif
