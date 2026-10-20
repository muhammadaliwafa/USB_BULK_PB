#ifndef UART_DMA_H
#define UART_DMA_H

#include <stdint.h>

/*
 * USART1 (PA9 = TX, PA10 = RX) dengan DMA.
 *   RX : DMA1 Ch5, circular + IDLE line interrupt. Callback dipanggil begitu
 *        data berhenti datang (idle 1 frame), tanpa menunggu jumlah tertentu.
 *        Callback juga dipanggil saat buffer DMA setengah/penuh agar aliran
 *        data terus-menerus tidak tertahan.
 *   TX : DMA1 Ch4, ring buffer, non-blocking.
 */

#define UART_PCLK2_HZ      72000000UL
#define UART_RX_DMA_SIZE   256u      /* buffer DMA RX (bebas)               */
#define UART_TX_RING_SIZE  512u      /* ring TX, WAJIB pangkat 2            */

void     uart_dma_init(uint32_t baud);
void     uart_dma_set_baud(uint32_t baud);

/* Antrikan data untuk dikirim lewat DMA. Return = jumlah byte yang diterima ring. */
uint16_t uart_dma_write(const uint8_t *data, uint16_t len);
uint16_t uart_dma_tx_free(void);

/* Dipanggil dari ISR saat ada data RX baru (di-override oleh aplikasi).
   Data hanya valid selama callback berjalan -> salin jika perlu. */
void     uart_rx_callback(const uint8_t *data, uint16_t len);

#endif
