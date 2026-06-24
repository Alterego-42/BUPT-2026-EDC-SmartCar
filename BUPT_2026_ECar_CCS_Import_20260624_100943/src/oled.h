#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_print(uint8_t row, uint8_t col, const char *text);
void oled_print_i32(uint8_t row, uint8_t col, const char *label, int32_t value);
void oled_show_status(uint8_t task, const char *state, int16_t line_pos,
    uint8_t gray_mask, int16_t yaw_x10, int32_t distance_mm);

#endif
