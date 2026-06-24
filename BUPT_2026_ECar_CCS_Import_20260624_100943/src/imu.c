#include "imu.h"
#include "board_pins.h"

static uint8_t g_frame[11];
static uint8_t g_frame_index;
static int16_t g_yaw_x10;
static uint32_t g_last_yaw_ms;
static bool g_has_yaw;

static int16_t wrap_1800(int32_t value)
{
    while (value > 1800) {
        value -= 3600;
    }
    while (value < -1800) {
        value += 3600;
    }
    return (int16_t) value;
}

static uint8_t checksum(const uint8_t *data)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 10U; i++) {
        sum += data[i];
    }
    return (uint8_t) sum;
}

static void parse_byte(uint8_t byte, uint32_t now_ms)
{
    if (g_frame_index == 0U) {
        if (byte == 0x55U) {
            g_frame[g_frame_index++] = byte;
        }
        return;
    }

    if (g_frame_index == 1U) {
        if (byte == 0x53U) {
            g_frame[g_frame_index++] = byte;
        } else {
            g_frame_index = 0U;
        }
        return;
    }

    g_frame[g_frame_index++] = byte;
    if (g_frame_index < sizeof(g_frame)) {
        return;
    }

    if (checksum(g_frame) == g_frame[10]) {
        int16_t raw_yaw = (int16_t) ((uint16_t) g_frame[7] << 8 | g_frame[6]);
        g_yaw_x10 = (int16_t) (((int32_t) raw_yaw * 1800) / 32768);
        g_last_yaw_ms = now_ms;
        g_has_yaw = true;
    }
    g_frame_index = 0U;
}

void imu_init(void)
{
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1};
    static const DL_UART_Main_Config uart_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE};

    DL_UART_Main_reset(BOARD_IMU_UART);
    DL_UART_Main_enablePower(BOARD_IMU_UART);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(
        BOARD_IMU_TX_IOMUX, BOARD_IMU_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        BOARD_IMU_RX_IOMUX, BOARD_IMU_RX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(BOARD_IMU_UART, &clock_config);
    DL_UART_Main_init(BOARD_IMU_UART, &uart_config);
    DL_UART_Main_configBaudRate(BOARD_IMU_UART, CPUCLK_FREQ, BOARD_IMU_BAUD);
    DL_UART_Main_enableFIFOs(BOARD_IMU_UART);
    DL_UART_Main_enable(BOARD_IMU_UART);
}

void imu_poll(uint32_t now_ms)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BOARD_IMU_UART)) {
        parse_byte((uint8_t) DL_UART_Main_receiveData(BOARD_IMU_UART), now_ms);
    }
}

bool imu_get_yaw_x10(uint32_t now_ms, int16_t *yaw_x10)
{
    if (!g_has_yaw || ((now_ms - g_last_yaw_ms) > BOARD_IMU_FRESH_MS)) {
        return false;
    }
    if (yaw_x10 != 0) {
        *yaw_x10 = g_yaw_x10;
    }
    return true;
}

int16_t imu_wrap_error_x10(int16_t target_x10, int16_t current_x10)
{
    return wrap_1800((int32_t) target_x10 - current_x10);
}
