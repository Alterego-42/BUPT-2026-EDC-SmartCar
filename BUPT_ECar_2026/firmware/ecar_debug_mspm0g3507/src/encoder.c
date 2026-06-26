#include "encoder.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_left_ticks;
static volatile int32_t g_right_ticks;

static int8_t quadrature_delta(GPIO_Regs *port_a, uint32_t pin_a,
    GPIO_Regs *port_b, uint32_t pin_b, uint8_t invert)
{
    bool a = DL_GPIO_readPins(port_a, pin_a) != 0U;
    bool b = DL_GPIO_readPins(port_b, pin_b) != 0U;
    int8_t delta = (a == b) ? 1 : -1;
    return invert ? (int8_t)-delta : delta;
}

void encoder_init(void)
{
    encoder_reset();
    NVIC_ClearPendingIRQ(GPIO_ENCODER_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_GPIOB_INT_IRQN);
}

void encoder_reset(void)
{
    __disable_irq();
    g_left_ticks = 0;
    g_right_ticks = 0;
    __enable_irq();
}

encoder_snapshot_t encoder_get(void)
{
    encoder_snapshot_t snap;
    __disable_irq();
    snap.left_ticks = g_left_ticks;
    snap.right_ticks = g_right_ticks;
    __enable_irq();

    snap.avg_ticks = (snap.left_ticks + snap.right_ticks) / 2;
    snap.distance_mm = (int32_t)(((int64_t)snap.avg_ticks * BOARD_WHEEL_CIRCUMFERENCE_MM) /
        BOARD_ENCODER_TICKS_PER_REV);
    return snap;
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpio_a = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_ENCODER_RR_A_PIN);
    if ((gpio_a & GPIO_ENCODER_RR_A_PIN) == GPIO_ENCODER_RR_A_PIN) {
        g_right_ticks += quadrature_delta(GPIO_ENCODER_RR_A_PORT, GPIO_ENCODER_RR_A_PIN,
            GPIO_ENCODER_RR_B_PORT, GPIO_ENCODER_RR_B_PIN, BOARD_ENCODER_RIGHT_INVERT);
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_RR_A_PIN);
    }

    uint32_t gpio_b = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_ENCODER_LR_A_PIN);
    if ((gpio_b & GPIO_ENCODER_LR_A_PIN) == GPIO_ENCODER_LR_A_PIN) {
        g_left_ticks += quadrature_delta(GPIO_ENCODER_LR_A_PORT, GPIO_ENCODER_LR_A_PIN,
            GPIO_ENCODER_LR_B_PORT, GPIO_ENCODER_LR_B_PIN, BOARD_ENCODER_LEFT_INVERT);
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_LR_A_PIN);
    }
}
