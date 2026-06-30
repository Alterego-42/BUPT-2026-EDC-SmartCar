#include "k230_uart.h"

#include <stddef.h>

#include "timebase.h"
#include "ti_msp_dl_config.h"

#define K230_UART_INST                (UART1)
#define K230_UART_INT_IRQN            (UART1_INT_IRQn)
#define K230_UART_TX_IOMUX            (IOMUX_PINCM39)
#define K230_UART_RX_IOMUX            (IOMUX_PINCM40)
#define K230_UART_TX_FUNC             (IOMUX_PINCM39_PF_UART1_TX)
#define K230_UART_RX_FUNC             (IOMUX_PINCM40_PF_UART1_RX)
#define K230_UART_IBRD_115200_4MHZ    (2U)
#define K230_UART_FBRD_115200_4MHZ    (11U)
#define K230_RX_RING_SIZE             (128U)
#define K230_LINE_SIZE                (48U)

static volatile uint8_t g_rx_ring[K230_RX_RING_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static uint8_t g_line_len;
static char g_line[K230_LINE_SIZE];
static k230_frame_t g_latest;
static bool g_has_frame;

static int16_t clamp_cdeg(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static bool parse_uint_token(const char **cursor, uint8_t *out)
{
    uint32_t value = 0;
    bool any = false;
    const char *p = *cursor;

    while (*p >= '0' && *p <= '9') {
        any = true;
        value = (value * 10U) + (uint32_t)(*p - '0');
        if (value > 255U) {
            return false;
        }
        p++;
    }

    if (!any) {
        return false;
    }

    *out = (uint8_t)value;
    *cursor = p;
    return true;
}

static bool parse_cdeg_token(const char **cursor, int16_t *out)
{
    int32_t whole = 0;
    int32_t frac = 0;
    uint8_t frac_digits = 0;
    bool negative = false;
    bool any = false;
    const char *p = *cursor;

    if (*p == '-') {
        negative = true;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        any = true;
        whole = (whole * 10L) + (int32_t)(*p - '0');
        p++;
    }

    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            if (frac_digits < 2U) {
                frac = (frac * 10L) + (int32_t)(*p - '0');
                frac_digits++;
            }
            p++;
            any = true;
        }
    }

    if (!any) {
        return false;
    }

    while (frac_digits < 2U) {
        frac *= 10L;
        frac_digits++;
    }

    whole = (whole * 100L) + frac;
    if (negative) {
        whole = -whole;
    }

    *out = clamp_cdeg(whole);
    *cursor = p;
    return true;
}

static bool parse_k230_line(const char *line)
{
    static const char prefix[] = "$K230,";
    const char *p = line;
    uint8_t valid = 0;
    uint8_t target_ok = 0;
    int16_t yaw = 0;
    int16_t pitch = 0;

    for (uint8_t i = 0; prefix[i] != '\0'; i++) {
        if (p[i] != prefix[i]) {
            return false;
        }
    }
    p += sizeof(prefix) - 1U;

    if (!parse_uint_token(&p, &valid) || *p != ',') {
        return false;
    }
    p++;

    if (!parse_cdeg_token(&p, &yaw) || *p != ',') {
        return false;
    }
    p++;

    if (!parse_cdeg_token(&p, &pitch)) {
        return false;
    }

    target_ok = valid;
    if (*p == ',') {
        p++;
        if (!parse_uint_token(&p, &target_ok)) {
            return false;
        }
    }
    if (*p != '\0') {
        return false;
    }

    g_latest.valid = valid != 0U;
    g_latest.target_ok = target_ok != 0U;
    g_latest.yaw_cdeg = yaw;
    g_latest.pitch_cdeg = pitch;
    g_latest.received_ms = millis();
    g_latest.frame_count++;
    g_has_frame = true;
    return true;
}

static void feed_line_byte(uint8_t byte)
{
    if (byte == '$') {
        g_line_len = 0;
    }

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (g_line_len > 0U) {
            g_line[g_line_len] = '\0';
            if (!parse_k230_line(g_line)) {
                g_latest.parse_errors++;
            }
        }
        g_line_len = 0;
        return;
    }

    if (g_line_len >= (K230_LINE_SIZE - 1U)) {
        g_line_len = 0;
        g_latest.parse_errors++;
        return;
    }

    g_line[g_line_len++] = (char)byte;
}

static bool read_rx_byte(uint8_t *byte)
{
    uint16_t tail = g_rx_tail;
    if (tail == g_rx_head) {
        return false;
    }

    *byte = g_rx_ring[tail];
    g_rx_tail = (uint16_t)((tail + 1U) & (K230_RX_RING_SIZE - 1U));
    return true;
}

void k230_uart_init(void)
{
    g_rx_head = 0;
    g_rx_tail = 0;
    g_line_len = 0;
    g_latest.valid = false;
    g_latest.target_ok = false;
    g_latest.yaw_cdeg = 0;
    g_latest.pitch_cdeg = 0;
    g_latest.received_ms = 0;
    g_latest.frame_count = 0;
    g_latest.rx_byte_count = 0;
    g_latest.parse_errors = 0;
    g_latest.overflow_count = 0;
    g_has_frame = false;

    DL_UART_Main_reset(K230_UART_INST);
    DL_UART_Main_enablePower(K230_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(K230_UART_TX_IOMUX, K230_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(K230_UART_RX_IOMUX, K230_UART_RX_FUNC);

    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel = DL_UART_MAIN_CLOCK_MFCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uart_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };

    DL_UART_Main_setClockConfig(K230_UART_INST, (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(K230_UART_INST, (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_setOversampling(K230_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        K230_UART_INST, K230_UART_IBRD_115200_4MHZ, K230_UART_FBRD_115200_4MHZ);
    DL_UART_Main_enableInterrupt(K230_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(K230_UART_INST);

    NVIC_ClearPendingIRQ(K230_UART_INT_IRQN);
    NVIC_EnableIRQ(K230_UART_INT_IRQN);
}

void k230_uart_task(void)
{
    uint8_t byte;
    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
        g_latest.rx_byte_count++;
        feed_line_byte(DL_UART_Main_receiveData(K230_UART_INST));
    }

    while (read_rx_byte(&byte)) {
        feed_line_byte(byte);
    }
}

bool k230_uart_get_latest(k230_frame_t *frame)
{
    if (frame == NULL || !g_has_frame) {
        return false;
    }

    *frame = g_latest;
    return true;
}

bool k230_uart_get_fresh(k230_frame_t *frame, uint32_t max_age_ms)
{
    if (!k230_uart_get_latest(frame)) {
        return false;
    }

    return (uint32_t)(millis() - frame->received_ms) <= max_age_ms;
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
            uint16_t next;
            uint16_t head = g_rx_head;
            g_latest.rx_byte_count++;
            next = (uint16_t)((head + 1U) & (K230_RX_RING_SIZE - 1U));
            if (next == g_rx_tail) {
                g_latest.overflow_count++;
                (void)DL_UART_Main_receiveData(K230_UART_INST);
            } else {
                g_rx_ring[head] = DL_UART_Main_receiveData(K230_UART_INST);
                g_rx_head = next;
            }
        }
        break;
    default:
        break;
    }
}
