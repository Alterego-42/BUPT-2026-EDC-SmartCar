#ifndef K230_H_
#define K230_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    uint8_t quality;
    bool fresh;
} k230_target_t;

void k230_init(void);
void k230_poll(uint32_t now_ms);
bool k230_get_target(uint32_t now_ms, k230_target_t *target);

#endif
