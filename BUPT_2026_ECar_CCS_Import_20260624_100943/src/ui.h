#ifndef UI_H_
#define UI_H_

#include <stdbool.h>
#include <stdint.h>

void ui_init(void);
void ui_update_10ms(void);
bool ui_start_button_pressed_edge(void);
bool ui_start_button_long_press_event(void);
bool ui_button_is_pressed(void);
uint16_t ui_button_hold_ms(void);
bool ui_button_release_event(uint16_t *held_ms);
void ui_set_status_leds(bool red_on, bool green_on);
void ui_set_running(bool running);
void ui_set_error(bool error);
void ui_beep_ms(uint16_t duration_ms);
void ui_keypoint_signal(uint8_t index);
void ui_finish_signal(void);

#endif
