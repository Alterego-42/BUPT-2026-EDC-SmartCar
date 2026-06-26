#include "grayscale.h"

#include "board_config.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static const int16_t k_weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

static void set_channel(uint8_t channel)
{
    if (channel & 0x01U) {
        DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    }

    if (channel & 0x02U) {
        DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    }

    if (channel & 0x04U) {
        DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    }
}

static bool read_out(void)
{
    return DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN) != 0U;
}

void grayscale_init(void)
{
    set_channel(0U);
}

grayscale_sample_t grayscale_read(void)
{
    grayscale_sample_t sample = {0};
    int32_t weighted_sum = 0;

    for (uint8_t channel = 0; channel < 8U; channel++) {
        uint8_t logical_index = BOARD_GRAY_REVERSE_ORDER ? (uint8_t)(7U - channel) : channel;
        set_channel(channel);
        delay_us(70U);

        bool raw_high = read_out();
        bool is_black = BOARD_GRAY_BLACK_IS_HIGH ? raw_high : !raw_high;
        if (raw_high) {
            sample.raw_mask |= (uint8_t)(1U << logical_index);
        }
        if (is_black) {
            sample.black_mask |= (uint8_t)(1U << logical_index);
            sample.black_count++;
            weighted_sum += k_weights[logical_index];
        }
    }

    if (sample.black_count > 0U) {
        sample.line_seen = true;
        sample.error = (int16_t)(weighted_sum / sample.black_count);
    } else {
        sample.line_seen = false;
        sample.error = 0;
    }

    return sample;
}
