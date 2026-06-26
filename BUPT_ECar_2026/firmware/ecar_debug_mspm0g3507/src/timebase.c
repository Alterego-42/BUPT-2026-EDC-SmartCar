#include "timebase.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t g_ms;

void timebase_init(void)
{
    g_ms = 0;
}

uint32_t millis(void)
{
    return g_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {
        __WFI();
    }
}

void delay_us(uint32_t us)
{
    while (us--) {
        delay_cycles(CPUCLK_FREQ / 1000000U);
    }
}

void SysTick_Handler(void)
{
    g_ms++;
}
