#ifndef USB_BULK_H
#define USB_BULK_H

#include <stdint.h>

/*
 * USB Full-Speed vendor-specific device, bulk only, register-level (tanpa HAL).
 *
 *   EP0        : control (enumerasi standar)
 *   EP1 OUT/IN : 0x01 / 0x81  -> data UART (PC->UART TX, UART RX->PC)
 *   EP2 OUT/IN : 0x02 / 0x82  -> command channel (shift register, baudrate, info)
 *
 * Semua endpoint bulk, max packet 64 byte.
 */

#define USB_EP_SIZE 64u

void    usb_init(void);
uint8_t usb_is_configured(void);

/* ---- IN (device -> host), ep = 1 atau 2 ---- */
uint8_t usb_tx_ready(uint8_t ep);                                  /* 1 = boleh kirim */
void    usb_send(uint8_t ep, const uint8_t *data, uint16_t len);   /* len 0..64, panggil hanya jika ready */

/* ---- OUT (host -> device), ep = 1 atau 2 ---- */
int16_t  usb_rx_pending(uint8_t ep);                  /* -1 = tidak ada paket, selain itu = panjang paket */
uint16_t usb_rx_read(uint8_t ep, uint8_t *dst);       /* salin paket (dst >= 64 byte) lalu buka lagi endpoint */

#endif
