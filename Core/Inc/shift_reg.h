#ifndef SHIFT_REG_H
#define SHIFT_REG_H

#include <stdint.h>
#include "stm32f1xx_ll_gpio.h"

/* Konfigurasi pin (versi LL dari define Anda) */
#define SER_PIN       LL_GPIO_PIN_5
#define SER_PORT      GPIOB
#define R_CLK         LL_GPIO_PIN_15
#define R_CLK_PORT    GPIOA
#define SR_CLK        LL_GPIO_PIN_3
#define SR_CLK_PORT   GPIOB

void shift_reg_init(void);

/*
 * Geser `len` byte ke 74HC595 (MSB duluan), lalu latch (RCLK).
 * data[0] berakhir di chip PALING JAUH dari mikrokontroler bila di-cascade;
 * bit0 sebuah byte = output QA, bit7 = QH.
 */
void shift_reg_write(const uint8_t *data, uint8_t len);

#endif
