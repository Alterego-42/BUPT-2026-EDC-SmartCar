#ifndef GRAYSCALE_H_
#define GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t mask;
    uint8_t active_count;
    bool line_seen;
    int16_t position;
} grayscale_sample_t;

void grayscale_init(void);
grayscale_sample_t grayscale_read(void);

#endif
