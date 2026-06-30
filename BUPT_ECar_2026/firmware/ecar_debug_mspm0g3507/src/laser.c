#include "laser.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#define LASER_PORT       (GPIOB)
#define LASER_PIN        (DL_GPIO_PIN_15)
#define LASER_IOMUX      (IOMUX_PINCM32)

static bool g_laser_on;

void laser_init(void)
{
    DL_GPIO_initDigitalOutput(LASER_IOMUX);
    laser_off();
    DL_GPIO_enableOutput(LASER_PORT, LASER_PIN);
}

void laser_set(bool on)
{
    g_laser_on = on;
#if BOARD_LASER_ACTIVE_HIGH
    if (on) {
        DL_GPIO_setPins(LASER_PORT, LASER_PIN);
    } else {
        DL_GPIO_clearPins(LASER_PORT, LASER_PIN);
    }
#else
    if (on) {
        DL_GPIO_clearPins(LASER_PORT, LASER_PIN);
    } else {
        DL_GPIO_setPins(LASER_PORT, LASER_PIN);
    }
#endif
}

void laser_off(void)
{
    laser_set(false);
}

bool laser_is_on(void)
{
    return g_laser_on;
}
