#ifndef SOUND_LIGHT_H
#define SOUND_LIGHT_H

#include <stdbool.h>
#include <stdint.h>

void sound_light_init(void);
void sound_light_task(void);
void sound_light_cancel(void);
bool sound_light_active(void);

void sound_light_keypoint(uint8_t point_index);
void sound_light_stop(void);
void sound_light_complete(void);
void sound_light_error(void);
void sound_light_aiming_ok(void);

void beep_on(void);
void beep_off(void);
void beep_toggle(void);

void led_on(void);
void led_off(void);
void led_toggle(void);

void signal_short(void);
void signal_keypoint(void);
void signal_stop(void);
void signal_complete(void);
void signal_error(void);
void signal_aiming_ok(void);

uint8_t is_beep_on(void);
uint8_t is_led_on(void);

#endif
