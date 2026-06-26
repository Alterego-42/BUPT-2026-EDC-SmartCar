#ifndef UI_H
#define UI_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    UI_BUTTON_NONE = 0,
    UI_BUTTON_SHORT,
    UI_BUTTON_LONG,
} ui_button_event_t;

void ui_init(void);
ui_button_event_t ui_poll_button(void);

void ui_leds(bool red, bool green);
void ui_red(bool on);
void ui_green(bool on);
void ui_beep(bool on);
void ui_beep_for(uint32_t ms);
void ui_task(void);
void ui_signal_mode(uint8_t mode);
void ui_signal_error(void);
void ui_signal_keypoint(uint8_t index);

#endif
