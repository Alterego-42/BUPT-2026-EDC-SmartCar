#include "oled.h"
#include "board_pins.h"
#include "i2c_bus.h"

static bool g_ready;

static void oled_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};
    (void) i2c_bus_write(BOARD_OLED_I2C_ADDR, data, 2U);
}

static void oled_data(uint8_t value)
{
    uint8_t data[2] = {0x40U, value};
    (void) i2c_bus_write(BOARD_OLED_I2C_ADDR, data, 2U);
}

static void set_cursor(uint8_t row, uint8_t col)
{
    if (row > 7U) {
        row = 7U;
    }
    if (col > 127U) {
        col = 127U;
    }
    oled_cmd((uint8_t) (0xB0U + row));
    oled_cmd((uint8_t) (0x00U + (col & 0x0FU)));
    oled_cmd((uint8_t) (0x10U + ((col >> 4) & 0x0FU)));
}

static uint16_t glyph_rows(char c)
{
    switch (c) {
    case '0': return 0x7B6FU;
    case '1': return 0x2492U;
    case '2': return 0x73E7U;
    case '3': return 0x73CFU;
    case '4': return 0x5BC9U;
    case '5': return 0x79CFU;
    case '6': return 0x79EFU;
    case '7': return 0x7249U;
    case '8': return 0x7BEFU;
    case '9': return 0x7BCFU;
    case 'A': return 0x2BEAU;
    case 'B': return 0x7BEFU;
    case 'C': return 0x7927U;
    case 'D': return 0x6B6EU;
    case 'E': return 0x79E7U;
    case 'F': return 0x79E4U;
    case 'G': return 0x79AFU;
    case 'H': return 0x5BEAU;
    case 'I': return 0x7497U;
    case 'J': return 0x124EU;
    case 'K': return 0x5ACAU;
    case 'L': return 0x4927U;
    case 'M': return 0x5F6BU;
    case 'N': return 0x5B6AU;
    case 'O': return 0x7B6FU;
    case 'P': return 0x7BE4U;
    case 'Q': return 0x7B7BU;
    case 'R': return 0x7BEAU;
    case 'S': return 0x79CFU;
    case 'T': return 0x7492U;
    case 'U': return 0x5B6FU;
    case 'V': return 0x5B54U;
    case 'W': return 0x5B7FU;
    case 'X': return 0x5AABU;
    case 'Y': return 0x5A92U;
    case 'Z': return 0x72A7U;
    case '-': return 0x01C0U;
    case ':': return 0x0820U;
    case '.': return 0x0002U;
    case '/': return 0x1248U;
    case ' ': return 0x0000U;
    default:
        if (c >= 'a' && c <= 'z') {
            return glyph_rows((char) (c - 32));
        }
        return 0x01C0U;
    }
}

static void write_char(char c)
{
    uint16_t rows = glyph_rows(c);
    uint8_t cols[4] = {0, 0, 0, 0};

    for (uint8_t y = 0; y < 5U; y++) {
        uint8_t row = (uint8_t) ((rows >> ((4U - y) * 3U)) & 0x07U);
        for (uint8_t x = 0; x < 3U; x++) {
            if ((row & (uint8_t) (1U << (2U - x))) != 0U) {
                cols[x] |= (uint8_t) (1U << y);
            }
        }
    }

    for (uint8_t i = 0; i < 4U; i++) {
        oled_data(cols[i]);
    }
}

static void print_uint(int32_t value)
{
    char buf[12];
    uint8_t pos = 0;
    bool neg = false;

    if (value < 0) {
        neg = true;
        value = -value;
    }

    do {
        buf[pos++] = (char) ('0' + (value % 10));
        value /= 10;
    } while ((value > 0) && (pos < sizeof(buf)));

    if (neg) {
        write_char('-');
    }
    while (pos > 0U) {
        write_char(buf[--pos]);
    }
}

void oled_init(void)
{
    i2c_bus_init();
    delay_cycles(CPUCLK_FREQ / 20U);

    oled_cmd(0xAEU);
    oled_cmd(0x20U);
    oled_cmd(0x02U);
    oled_cmd(0xB0U);
    oled_cmd(0xC8U);
    oled_cmd(0x00U);
    oled_cmd(0x10U);
    oled_cmd(0x40U);
    oled_cmd(0x81U);
    oled_cmd(0x7FU);
    oled_cmd(0xA1U);
    oled_cmd(0xA6U);
    oled_cmd(0xA8U);
    oled_cmd(0x3FU);
    oled_cmd(0xA4U);
    oled_cmd(0xD3U);
    oled_cmd(0x00U);
    oled_cmd(0xD5U);
    oled_cmd(0x80U);
    oled_cmd(0xD9U);
    oled_cmd(0xF1U);
    oled_cmd(0xDAU);
    oled_cmd(0x12U);
    oled_cmd(0xDBU);
    oled_cmd(0x40U);
    oled_cmd(0x8DU);
    oled_cmd(0x14U);
    oled_cmd(0xAFU);
    g_ready = true;
    oled_clear();
}

void oled_clear(void)
{
    if (!g_ready) {
        return;
    }
    for (uint8_t row = 0; row < 8U; row++) {
        set_cursor(row, 0U);
        for (uint8_t col = 0; col < 128U; col++) {
            oled_data(0x00U);
        }
    }
}

void oled_print(uint8_t row, uint8_t col, const char *text)
{
    if (!g_ready) {
        return;
    }
    set_cursor(row, (uint8_t) (col * 4U));
    while (*text != '\0') {
        write_char(*text++);
    }
}

void oled_print_i32(uint8_t row, uint8_t col, const char *label, int32_t value)
{
    oled_print(row, col, label);
    print_uint(value);
}

void oled_show_status(uint8_t task, const char *state, int16_t line_pos,
    uint8_t gray_mask, int16_t yaw_x10, int32_t distance_mm)
{
    oled_clear();
    oled_print(0, 0, "TASK");
    print_uint(task);
    oled_print(0, 6, state);
    oled_print_i32(1, 0, "LINE", line_pos);
    oled_print_i32(2, 0, "GRAY", gray_mask);
    oled_print_i32(3, 0, "YAW", yaw_x10);
    oled_print_i32(4, 0, "DIST", distance_mm);
}
