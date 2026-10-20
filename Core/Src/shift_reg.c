#include "shift_reg.h"
#include "stm32f1xx_ll_bus.h"

#define SR_DELAY()  do { __NOP(); __NOP(); } while (0)

static void out_pin(GPIO_TypeDef *port, uint32_t pin)
{
    LL_GPIO_ResetOutputPin(port, pin);                     /* mulai dari low, tanpa glitch */
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(port, pin, LL_GPIO_OUTPUT_PUSHPULL);
}

void shift_reg_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA |
                             LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);

    /* PA15 dan PB3 default dipakai JTAG: matikan JTAG, SWD (PA13/PA14) tetap hidup */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    out_pin(SER_PORT,    SER_PIN);
    out_pin(SR_CLK_PORT, SR_CLK);
    out_pin(R_CLK_PORT,  R_CLK);
}

void shift_reg_write(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        uint8_t v = data[i];
        for (uint8_t b = 0; b < 8u; b++) {
            if (v & 0x80u) {
                LL_GPIO_SetOutputPin(SER_PORT, SER_PIN);
            } else {
                LL_GPIO_ResetOutputPin(SER_PORT, SER_PIN);
            }
            v <<= 1;
            SR_DELAY();
            LL_GPIO_SetOutputPin(SR_CLK_PORT, SR_CLK);     /* rising edge: geser */
            SR_DELAY();
            LL_GPIO_ResetOutputPin(SR_CLK_PORT, SR_CLK);
        }
    }
    SR_DELAY();
    LL_GPIO_SetOutputPin(R_CLK_PORT, R_CLK);               /* rising edge: latch ke output */
    SR_DELAY();
    LL_GPIO_ResetOutputPin(R_CLK_PORT, R_CLK);
}
