#include "encoder.h"
#include "board_pins.h"
#include <stdbool.h>

static volatile int32_t g_lr_count;
static volatile int32_t g_rr_count;

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void update_lr(void)
{
    bool a = (DL_GPIO_readPins(BOARD_LR_ENC_A_PORT, BOARD_LR_ENC_A_PIN) != 0U);
    bool b = (DL_GPIO_readPins(BOARD_LR_ENC_B_PORT, BOARD_LR_ENC_B_PIN) != 0U);
    int32_t step = (a == b) ? 1 : -1;
    if (BOARD_LR_ENCODER_INVERT) {
        step = -step;
    }
    g_lr_count += step;
}

static void update_rr(void)
{
    bool a = (DL_GPIO_readPins(BOARD_RR_ENC_A_PORT, BOARD_RR_ENC_A_PIN) != 0U);
    bool b = (DL_GPIO_readPins(BOARD_RR_ENC_B_PORT, BOARD_RR_ENC_B_PIN) != 0U);
    int32_t step = (a == b) ? 1 : -1;
    if (BOARD_RR_ENCODER_INVERT) {
        step = -step;
    }
    g_rr_count += step;
}

void encoder_init(void)
{
    DL_GPIO_initDigitalInputFeatures(BOARD_LR_ENC_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_LR_ENC_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_RR_ENC_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_RR_ENC_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(
        BOARD_LR_ENC_A_PORT, DL_GPIO_PIN_19_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(
        BOARD_RR_ENC_A_PORT, DL_GPIO_PIN_26_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(BOARD_LR_ENC_A_PORT, BOARD_LR_ENC_A_PIN);
    DL_GPIO_clearInterruptStatus(BOARD_RR_ENC_A_PORT, BOARD_RR_ENC_A_PIN);
    DL_GPIO_enableInterrupt(BOARD_LR_ENC_A_PORT, BOARD_LR_ENC_A_PIN);
    DL_GPIO_enableInterrupt(BOARD_RR_ENC_A_PORT, BOARD_RR_ENC_A_PIN);

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_reset(void)
{
    __disable_irq();
    g_lr_count = 0;
    g_rr_count = 0;
    __enable_irq();
}

void encoder_get_counts(int32_t *lr_count, int32_t *rr_count)
{
    __disable_irq();
    int32_t lr = g_lr_count;
    int32_t rr = g_rr_count;
    __enable_irq();

    if (lr_count != 0) {
        *lr_count = lr;
    }
    if (rr_count != 0) {
        *rr_count = rr;
    }
}

int32_t encoder_get_average_distance_mm(void)
{
    int32_t lr;
    int32_t rr;
    encoder_get_counts(&lr, &rr);

    int32_t avg_counts = (abs32(lr) + abs32(rr)) / 2;
    int32_t circumference_mm =
        (BOARD_WHEEL_DIAMETER_MM * 31416 + 5000) / 10000;

    return (avg_counts * circumference_mm) / BOARD_ENCODER_COUNTS_PER_REV;
}

void encoder_handle_gpio_interrupts(void)
{
    uint32_t gpio_b =
        DL_GPIO_getEnabledInterruptStatus(BOARD_LR_ENC_A_PORT, BOARD_LR_ENC_A_PIN);
    if ((gpio_b & BOARD_LR_ENC_A_PIN) != 0U) {
        update_lr();
        DL_GPIO_clearInterruptStatus(BOARD_LR_ENC_A_PORT, BOARD_LR_ENC_A_PIN);
    }

    uint32_t gpio_a =
        DL_GPIO_getEnabledInterruptStatus(BOARD_RR_ENC_A_PORT, BOARD_RR_ENC_A_PIN);
    if ((gpio_a & BOARD_RR_ENC_A_PIN) != 0U) {
        update_rr();
        DL_GPIO_clearInterruptStatus(BOARD_RR_ENC_A_PORT, BOARD_RR_ENC_A_PIN);
    }
}

void GROUP1_IRQHandler(void)
{
    encoder_handle_gpio_interrupts();
}
