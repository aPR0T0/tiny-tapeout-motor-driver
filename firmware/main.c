/*
 * Example firmware for tt_um_apr0t0_foc_mcu.
 *
 *  1. switches the flash to quad I/O reads (sets QE with atomic SPI bursts)
 *  2. runs the ISA self test and reports the signature in MBOX_C2H
 *  3. loads a default motor configuration (what a standalone controller does
 *     at power-up)
 *  4. serves a tiny mailbox protocol for the host and runs a speed ramp:
 *       MBOX_H2C = (cmd << 24) | arg, the firmware clears it when done
 *       cmd 1: ramp the speed reference to arg (signed 16 bit)
 *       cmd 2: echo -> MBOX_C2H = 0xEC000000 | (arg + 1)
 *       cmd 3: EBREAK (halts the CPU, for debugger tests)
 *
 * Only 128 bytes of RAM exist: keep globals tiny and avoid deep call chains.
 */
#include <stdint.h>
#include "foc_regs.h"
#include "selftest.h"

static void spi_burst(uint32_t cs, uint32_t b0, uint32_t b1, uint32_t n)
{
    SPI_CTRL = cs | (1u << 3);          /* no hold, SCK = clk/4 */
    SPI_BUF0 = b0;
    SPI_BUF1 = b1;
    SPI_XFER = n;
    while (SPI_XFER & 1u) { }
}

static void flash_quad_enable(void)
{
    spi_burst(0, 0x50, 0, 1);           /* volatile SR write enable */
    spi_burst(0, 0x02u << 8 | 0x31, 0, 2); /* write SR2: QE = 1 */
    FLASH_CFG = 1u | (4u << 1) | (0u << 4); /* quad I/O, 4 dummy, SCK = clk/2 */
}

static void motor_defaults(void)
{
    PWM_PERIOD = 1250;                  /* 20 kHz at 50 MHz */
    PWM_CFG    = 25;                    /* 500 ns dead time */
    DUTY_LIM   = 20u | (1230u << 16);   /* keep bootstrap caps charged */
    ADC_CFG    = 16u | (1u << 9) | (1u << 15) | (1u << 16);
    ADC_OFFS   = 0x80008000u;
    CUR_KP     = 10723u | (12u << 16);
    CUR_KI     = 1372u | (0u << 16);
    LIMITS     = 18918u | (30000u << 16);
    SPD_KP     = 25600u | (10u << 16);  /* see focctl.py speed_pi_gains() */
    SPD_KI     = 25850u | (0u << 16);
    IQ_LIM     = 4915u;
}

static int32_t speed_target;

int main(void)
{
    static volatile uint8_t scratch[8];

    flash_quad_enable();
    MBOX_C2H = selftest(selftest_inputs, SELFTEST_N, scratch);
    motor_defaults();

    int32_t sref = 0;
    uint32_t t_last = TIME;
    for (;;) {
        uint32_t m = MBOX_H2C;
        if (m) {
            uint32_t cmd = m >> 24, arg = m & 0xFFFFFFu;
            if (cmd == 1) speed_target = (int16_t)arg;
            else if (cmd == 2) MBOX_C2H = 0xEC000000u | ((arg + 1) & 0xFFFFFFu);
            else if (cmd == 3) __asm__ volatile("ebreak");
            MBOX_H2C = 0;
        }
        /* speed ramp: 1 unit every 2048 clocks */
        if ((uint32_t)(TIME - t_last) >= 2048u) {
            t_last += 2048u;
            if (sref < speed_target) sref++;
            else if (sref > speed_target) sref--;
            SPD_CFG = ((uint32_t)sref & 0xFFFFu) | (9u << 16);
        }
    }
}
