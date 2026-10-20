#include "uart_dma.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"

#define TX_MASK (UART_TX_RING_SIZE - 1u)

static uint8_t  rx_dma_buf[UART_RX_DMA_SIZE];
static uint16_t rx_old;

static uint8_t           tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_chunk;
static volatile uint8_t  tx_busy;

__attribute__((weak)) void uart_rx_callback(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

/* ------------------------------------------------------------------ */
/* RX: proses data baru di buffer DMA (dipanggil dari IDLE/HT/TC ISR)   */
/* ------------------------------------------------------------------ */
static void rx_check(void)
{
    uint16_t pos = (uint16_t)(UART_RX_DMA_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5));

    if (pos != rx_old) {
        if (pos > rx_old) {
            uart_rx_callback(&rx_dma_buf[rx_old], (uint16_t)(pos - rx_old));
        } else {                                   /* wrap-around */
            uart_rx_callback(&rx_dma_buf[rx_old], (uint16_t)(UART_RX_DMA_SIZE - rx_old));
            if (pos > 0u) {
                uart_rx_callback(&rx_dma_buf[0], pos);
            }
        }
        rx_old = pos;
    }
}

void USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);           /* baca SR lalu DR */
        rx_check();
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);
    }
    if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);
    }
    rx_check();
}

/* ------------------------------------------------------------------ */
/* TX                                                                  */
/* ------------------------------------------------------------------ */
static void tx_start(void)       /* panggil dengan IRQ DMA1_Ch4 tertahan / dari ISR-nya */
{
    uint16_t t = tx_tail;
    uint16_t h = tx_head;

    if (t == h) {
        tx_busy = 0;
        return;
    }
    uint16_t n = (h > t) ? (uint16_t)(h - t) : (uint16_t)(UART_TX_RING_SIZE - t);
    tx_chunk = n;
    tx_busy = 1;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&tx_ring[t]);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, n);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

void DMA1_Channel4_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        tx_tail = (uint16_t)((tx_tail + tx_chunk) & TX_MASK);
        tx_busy = 0;
        tx_start();                                /* sisa data (mis. setelah wrap) */
    }
}

uint16_t uart_dma_tx_free(void)
{
    return (uint16_t)((tx_tail - tx_head - 1u) & TX_MASK);
}

uint16_t uart_dma_write(const uint8_t *data, uint16_t len)
{
    uint16_t h = tx_head;
    uint16_t freeb = uart_dma_tx_free();

    if (len > freeb) {
        len = freeb;
    }
    for (uint16_t i = 0; i < len; i++) {
        tx_ring[(h + i) & TX_MASK] = data[i];
    }
    tx_head = (uint16_t)((h + len) & TX_MASK);

    if (len) {
        NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        if (!tx_busy) {
            tx_start();
        }
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    }
    return len;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */
void uart_dma_set_baud(uint32_t baud)
{
    while (tx_busy) { }
    while (!LL_USART_IsActiveFlag_TC(USART1)) { }
    LL_USART_Disable(USART1);
    LL_USART_SetBaudRate(USART1, UART_PCLK2_HZ, baud);
    LL_USART_Enable(USART1);
}

void uart_dma_init(uint32_t baud)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA | LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* PA9 = TX (AF push-pull), PA10 = RX (input pull-up) */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

    /* USART1 */
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_SetBaudRate(USART1, UART_PCLK2_HZ, baud);

    /* DMA1 Ch5 : USART1_RX -> memori, circular */
    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5,
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR |
                          LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                          LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE |
                          LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)rx_dma_buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, UART_RX_DMA_SIZE);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

    /* DMA1 Ch4 : memori -> USART1_TX, normal */
    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4,
                          LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL |
                          LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                          LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE |
                          LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    /* Prioritas sama untuk semua sumber RX agar rx_check() tidak ter-nest */
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_SetPriority(DMA1_Channel5_IRQn, 2);
    NVIC_SetPriority(DMA1_Channel4_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    LL_USART_Enable(USART1);
}
