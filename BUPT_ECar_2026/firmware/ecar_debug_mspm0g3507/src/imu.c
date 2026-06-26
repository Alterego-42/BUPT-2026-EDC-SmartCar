#include "imu.h"

#include "board_config.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static volatile int16_t g_yaw_x10;
static volatile uint32_t g_last_yaw_ms;
static uint8_t g_frame[11];
static uint8_t g_index;

static int16_t wrap_error_x10(int16_t target, int16_t current)
{
    int16_t error = (int16_t)(target - current);
    while (error > 1800) {
        error = (int16_t)(error - 3600);
    }
    while (error < -1800) {
        error = (int16_t)(error + 3600);
    }
    return error;
}

static void parse_frame(void)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10U; i++) {
        sum = (uint8_t)(sum + g_frame[i]);
    }
    if (sum != g_frame[10] || g_frame[1] != 0x53U) {
        return;
    }

    int16_t raw_yaw = (int16_t)((uint16_t)g_frame[6] | ((uint16_t)g_frame[7] << 8));
    int16_t yaw_x10 = (int16_t)(((int32_t)raw_yaw * 1800L) / 32768L);
    if (BOARD_HEADING_INVERT) {
        yaw_x10 = (int16_t)-yaw_x10;
    }

    g_yaw_x10 = yaw_x10;
    g_last_yaw_ms = millis();
}

void imu_init(void)
{
    g_yaw_x10 = 0;
    g_last_yaw_ms = 0;
    g_index = 0;
    NVIC_ClearPendingIRQ(UART_IMU_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

void imu_feed_byte(uint8_t byte)
{
    if (g_index == 0U && byte != 0x55U) {
        return;
    }

    g_frame[g_index++] = byte;
    if (g_index >= sizeof(g_frame)) {
        parse_frame();
        g_index = 0U;
    }
}

bool imu_yaw_valid(void)
{
    return g_last_yaw_ms != 0U &&
        (uint32_t)(millis() - g_last_yaw_ms) <= BOARD_IMU_VALID_MS;
}

int16_t imu_get_yaw_x10(void)
{
    return g_yaw_x10;
}

int16_t imu_yaw_error_x10(int16_t target_x10)
{
    return wrap_error_x10(target_x10, imu_get_yaw_x10());
}

void UART_IMU_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
            imu_feed_byte(DL_UART_Main_receiveData(UART_IMU_INST));
        }
        break;
    default:
        break;
    }
}
