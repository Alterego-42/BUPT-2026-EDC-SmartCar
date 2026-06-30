#ifndef K230_UART_H
#define K230_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool target_ok;
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
    uint32_t received_ms;
    uint32_t frame_count;
    uint32_t rx_byte_count;
    uint32_t parse_errors;
    uint32_t overflow_count;
} k230_frame_t;

void k230_uart_init(void);
void k230_uart_task(void);
bool k230_uart_get_latest(k230_frame_t *frame);
bool k230_uart_get_fresh(k230_frame_t *frame, uint32_t max_age_ms);

#endif
