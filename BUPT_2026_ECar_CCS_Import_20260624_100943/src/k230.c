#include "k230.h"
#include "board_pins.h"

static char g_buf[32];
static uint8_t g_pos;
static k230_target_t g_target;
static uint32_t g_last_ms;

static int16_t parse_signed3(const char *s)
{
    int16_t sign = 1;
    if (*s == '-') {
        sign = -1;
        s++;
    } else if (*s == '+') {
        s++;
    }
    int16_t value = 0;
    for (uint8_t i = 0; i < 3U; i++) {
        if (s[i] < '0' || s[i] > '9') {
            break;
        }
        value = (int16_t) (value * 10 + (s[i] - '0'));
    }
    return (int16_t) (sign * value);
}

static uint8_t parse_u3(const char *s)
{
    uint16_t value = 0;
    for (uint8_t i = 0; i < 3U; i++) {
        if (s[i] < '0' || s[i] > '9') {
            break;
        }
        value = (uint16_t) (value * 10U + (uint16_t) (s[i] - '0'));
    }
    return (value > 255U) ? 255U : (uint8_t) value;
}

static void parse_frame(uint32_t now_ms)
{
    if (g_buf[0] != 'X' || g_buf[6] != 'Y' || g_buf[12] != 'Q') {
        return;
    }
    g_target.x = parse_signed3(&g_buf[2]);
    g_target.y = parse_signed3(&g_buf[8]);
    g_target.quality = parse_u3(&g_buf[14]);
    g_target.fresh = true;
    g_last_ms = now_ms;
}

void k230_init(void)
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

    DL_UART_Main_reset(BOARD_K230_UART);
    DL_UART_Main_enablePower(BOARD_K230_UART);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(
        BOARD_K230_TX_IOMUX, BOARD_K230_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        BOARD_K230_RX_IOMUX, BOARD_K230_RX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(BOARD_K230_UART, &clock_config);
    DL_UART_Main_init(BOARD_K230_UART, &uart_config);
    DL_UART_Main_configBaudRate(BOARD_K230_UART, CPUCLK_FREQ, BOARD_K230_BAUD);
    DL_UART_Main_enableFIFOs(BOARD_K230_UART);
    DL_UART_Main_enable(BOARD_K230_UART);
}

void k230_poll(uint32_t now_ms)
{
    while (!DL_UART_Main_isRXFIFOEmpty(BOARD_K230_UART)) {
        char ch = (char) DL_UART_Main_receiveData(BOARD_K230_UART);
        if (ch == '$') {
            g_pos = 0U;
        } else if (ch == '#') {
            g_buf[g_pos] = '\0';
            parse_frame(now_ms);
            g_pos = 0U;
        } else if (g_pos < (sizeof(g_buf) - 1U)) {
            g_buf[g_pos++] = ch;
        }
    }
}

bool k230_get_target(uint32_t now_ms, k230_target_t *target)
{
    if (!g_target.fresh || ((now_ms - g_last_ms) > BOARD_K230_FRESH_MS)) {
        return false;
    }
    if (target != 0) {
        *target = g_target;
    }
    return true;
}
