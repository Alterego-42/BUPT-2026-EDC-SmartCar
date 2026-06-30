#ifndef LASER_H
#define LASER_H

#include <stdbool.h>

void laser_init(void);
void laser_set(bool on);
void laser_off(void);
bool laser_is_on(void);

#endif
