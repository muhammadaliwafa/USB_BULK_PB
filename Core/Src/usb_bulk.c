#include "usb_bulk.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

/* ------------------------------------------------------------------ */
/* Identitas (ganti VID/PID sesuai kebutuhan)                          */
/* 0x1209/0x0001 = VID/PID uji coba dari pid.codes                      */
/* ------------------------------------------------------------------ */
#define USB_VID 0x5A5Au
#define USB_PID 0x0001u

#ifndef UID_BASE
#define UID_BASE 0x1FFFF7E8UL
#endif

/* ------------------------------------------------------------------ */
/* Register & bit USB (F103)                                           */
/* ------------------------------------------------------------------ */
#define EPR(n)          (*(volatile uint16_t *)(USB_BASE + 4u * (n)))
#define PMA_PTR(off)    ((volatile uint16_t *)(USB_PMAADDR + 2u * (off)))

/* Buffer descriptor table di awal PMA (8 byte per endpoint) */
#define BT_ADDR_TX(n)   (*PMA_PTR(8u * (n) + 0u))
#define BT_COUNT_TX(n)  (*PMA_PTR(8u * (n) + 2u))
#define BT_ADDR_RX(n)   (*PMA_PTR(8u * (n) + 4u))
#define BT_COUNT_RX(n)  (*PMA_PTR(8u * (n) + 6u))

/* Layout PMA: BTABLE 0..63, lalu per EP: TX 64B, RX 64B */
#define PMA_TXA(n)      (64u + 128u * (n))
#define PMA_RXA(n)      (128u + 128u * (n))
#define RXCNT_64B       0x8400u   /* BL_SIZE=1, NUM_BLOCK=1 -> 64 byte */

#define EP_CTR_RX       0x8000u
#define EP_DTOG_RX      0x4000u
#define EP_STAT_RX      0x3000u
#define EP_SETUP        0x0800u
#define EP_TYPE_BULK    0x0000u
#define EP_TYPE_CTRL    0x0200u
#define EP_CTR_TX       0x0080u
#define EP_DTOG_TX      0x0040u
#define EP_STAT_TX      0x0030u
#define EP_KEEP         0x070Fu   /* EP_TYPE | EP_KIND | EA (field rw biasa) */

#define STAT_RX_DIS     0x0000u
#define STAT_RX_STALL   0x1000u
#define STAT_RX_NAK     0x2000u
#define STAT_RX_VALID   0x3000u
#define STAT_TX_DIS     0x0000u
#define STAT_TX_STALL   0x0010u
#define STAT_TX_NAK     0x0020u
#define STAT_TX_VALID   0x0030u

#define ISTR_CTR        0x8000u
#define ISTR_RESET      0x0400u
#define CNTR_CTRM       0x8000u
#define CNTR_RESETM     0x0400u
#define CNTR_FRES       0x0001u
#define DADDR_EF        0x0080u

/* ------------------------------------------------------------------ */
/* Deskriptor                                                          */
/* ------------------------------------------------------------------ */
static const uint8_t dev_desc[18] = {
    18, 0x01,
    0x00, 0x02,                       /* bcdUSB 2.00 */
    0xFF, 0x00, 0x00,                 /* vendor specific */
    USB_EP_SIZE,
    USB_VID & 0xFF, USB_VID >> 8,
    USB_PID & 0xFF, USB_PID >> 8,
    0x00, 0x01,                       /* bcdDevice */
    1, 2, 3,                          /* iManufacturer, iProduct, iSerial */
    1                                 /* bNumConfigurations */
};

static const uint8_t cfg_desc[46] = {
    /* Configuration */
    9, 0x02, 46, 0, 1, 1, 0, 0x80, 50,
    /* Interface 0: vendor specific, 4 endpoint */
    9, 0x04, 0, 0, 4, 0xFF, 0x00, 0x00, 0,
    /* EP1 OUT */ 7, 0x05, 0x01, 0x02, USB_EP_SIZE, 0, 0,
    /* EP1 IN  */ 7, 0x05, 0x81, 0x02, USB_EP_SIZE, 0, 0,
    /* EP2 OUT */ 7, 0x05, 0x02, 0x02, USB_EP_SIZE, 0, 0,
    /* EP2 IN  */ 7, 0x05, 0x82, 0x02, USB_EP_SIZE, 0, 0
};

static const uint8_t lang_desc[4] = { 4, 0x03, 0x09, 0x04 };

static uint8_t strbuf[2 + 2 * 32];

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static volatile uint8_t configured;
static volatile uint8_t tx_busy[3];
static volatile int16_t rx_len[3] = { -1, -1, -1 };

static const uint8_t *ctl_ptr;
static uint16_t       ctl_rem;
static uint8_t        ctl_zlp;
static uint8_t        ctl_buf[2];
static uint8_t        addr_pending;
static uint8_t        addr_val;

/* ------------------------------------------------------------------ */
/* Helper PMA                                                          */
/* ------------------------------------------------------------------ */
static void pma_read(uint16_t off, uint8_t *dst, uint16_t len)
{
    volatile uint16_t *p = PMA_PTR(off);
    while (len > 1u) {
        uint16_t w = *p;
        p += 2;
        *dst++ = (uint8_t)w;
        *dst++ = (uint8_t)(w >> 8);
        len -= 2u;
    }
    if (len) {
        *dst = (uint8_t)*p;
    }
}

static void pma_write(uint16_t off, const uint8_t *src, uint16_t len)
{
    volatile uint16_t *p = PMA_PTR(off);
    while (len > 1u) {
        *p = (uint16_t)(src[0] | (src[1] << 8));
        p += 2;
        src += 2;
        len -= 2u;
    }
    if (len) {
        *p = src[0];
    }
}

/* ------------------------------------------------------------------ */
/* Helper register EPnR (bit toggle / rc_w0 butuh penanganan khusus)    */
/* ------------------------------------------------------------------ */
static void ep_config(uint8_t n, uint16_t type, uint16_t tx, uint16_t rx)
{
    uint16_t r = EPR(n);
    uint16_t v = (uint16_t)(type | n);
    v |= r & (EP_DTOG_RX | EP_DTOG_TX);          /* toggle jadi 0 */
    v |= (r ^ tx) & EP_STAT_TX;
    v |= (r ^ rx) & EP_STAT_RX;
    EPR(n) = v;                                  /* CTR_RX/CTR_TX ditulis 0 = clear */
}

static inline void ep_set_tx(uint8_t n, uint16_t st)
{
    uint16_t r = EPR(n);
    EPR(n) = (uint16_t)((r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | ((r ^ st) & EP_STAT_TX));
}

static inline void ep_set_rx(uint8_t n, uint16_t st)
{
    uint16_t r = EPR(n);
    EPR(n) = (uint16_t)((r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | ((r ^ st) & EP_STAT_RX));
}

static inline void ep_ack_rx(uint8_t n)
{
    EPR(n) = (uint16_t)((EPR(n) & EP_KEEP) | EP_CTR_TX);   /* CTR_RX = 0 */
}

static inline void ep_ack_tx(uint8_t n)
{
    EPR(n) = (uint16_t)((EPR(n) & EP_KEEP) | EP_CTR_RX);   /* CTR_TX = 0 */
}

/* ------------------------------------------------------------------ */
/* EP0 control                                                         */
/* ------------------------------------------------------------------ */
static void ep0_tx_next(void)
{
    uint16_t n = (ctl_rem > USB_EP_SIZE) ? USB_EP_SIZE : ctl_rem;
    pma_write(PMA_TXA(0), ctl_ptr, n);
    BT_COUNT_TX(0) = n;
    ctl_ptr += n;
    ctl_rem -= n;
    ep_set_tx(0, STAT_TX_VALID);
}

static void ep0_reply(const uint8_t *p, uint16_t len, uint16_t wlen)
{
    if (len > wlen) {
        len = wlen;
    }
    ctl_ptr = p;
    ctl_rem = len;
    ctl_zlp = (len != 0u && len < wlen && (len % USB_EP_SIZE) == 0u) ? 1u : 0u;
    ep0_tx_next();
}

static void ep0_stall(void)
{
    ep_set_tx(0, STAT_TX_STALL);
    ep_set_rx(0, STAT_RX_STALL);
}

static uint16_t build_string(uint8_t idx)
{
    char uid[25];
    const char *s;
    uint8_t n = 0;

    if (idx == 1u) {
        s = "MicroDiv";
    } else if (idx == 2u) {
        s = "Phonebank Controller";
    } else if (idx == 3u) {
        for (uint8_t i = 0; i < 3u; i++) {
            uint32_t w = *(volatile uint32_t *)(UID_BASE + 4u * i);
            for (uint8_t j = 0; j < 8u; j++) {
                uid[i * 8u + j] = "0123456789ABCDEF"[(w >> (28u - 4u * j)) & 0xFu];
            }
        }
        uid[24] = '\0';
        s = uid;
    } else {
        return 0;
    }

    while (s[n] && n < 31u) {
        strbuf[2u + 2u * n] = (uint8_t)s[n];
        strbuf[3u + 2u * n] = 0;
        n++;
    }
    strbuf[0] = (uint8_t)(2u + 2u * n);
    strbuf[1] = 0x03;
    return strbuf[0];
}

static void usb_set_config(uint8_t v)
{
    configured = v;
    for (uint8_t e = 1; e <= 2u; e++) {
        tx_busy[e] = 0;
        rx_len[e]  = -1;
        if (v) {
            ep_config(e, EP_TYPE_BULK, STAT_TX_NAK, STAT_RX_VALID);
        } else {
            ep_config(e, EP_TYPE_BULK, STAT_TX_DIS, STAT_RX_DIS);
        }
    }
}

static void ep0_setup(void)
{
    uint8_t s[8];
    pma_read(PMA_RXA(0), s, 8);

    uint8_t  bm = s[0];
    uint8_t  rq = s[1];
    uint16_t wv = (uint16_t)(s[2] | (s[3] << 8));
    uint16_t wi = (uint16_t)(s[4] | (s[5] << 8));
    uint16_t wl = (uint16_t)(s[6] | (s[7] << 8));

    ctl_rem = 0;
    ctl_zlp = 0;
    ep_set_rx(0, STAT_RX_VALID);          /* untuk status stage / SETUP berikutnya */

    if ((bm & 0x60u) != 0u) {             /* class/vendor request: tidak ada */
        ep0_stall();
        return;
    }

    switch (rq) {
    case 0x06: {                          /* GET_DESCRIPTOR */
        uint8_t t = (uint8_t)(wv >> 8);
        uint8_t i = (uint8_t)(wv & 0xFFu);
        if (t == 0x01u) {
            ep0_reply(dev_desc, sizeof dev_desc, wl);
        } else if (t == 0x02u) {
            ep0_reply(cfg_desc, sizeof cfg_desc, wl);
        } else if (t == 0x03u) {
            if (i == 0u) {
                ep0_reply(lang_desc, sizeof lang_desc, wl);
            } else if (build_string(i)) {
                ep0_reply(strbuf, strbuf[0], wl);
            } else {
                ep0_stall();
            }
        } else {
            ep0_stall();
        }
        break;
    }
    case 0x05:                            /* SET_ADDRESS (diterapkan setelah status stage) */
        addr_val = (uint8_t)(wv & 0x7Fu);
        addr_pending = 1;
        ep0_reply(0, 0, 0);
        break;
    case 0x09:                            /* SET_CONFIGURATION */
        usb_set_config((uint8_t)(wv & 0xFFu));
        ep0_reply(0, 0, 0);
        break;
    case 0x08:                            /* GET_CONFIGURATION */
        ctl_buf[0] = configured;
        ep0_reply(ctl_buf, 1, wl);
        break;
    case 0x00:                            /* GET_STATUS */
        ctl_buf[0] = 0;
        ctl_buf[1] = 0;
        ep0_reply(ctl_buf, 2, wl);
        break;
    case 0x0A:                            /* GET_INTERFACE */
        ctl_buf[0] = 0;
        ep0_reply(ctl_buf, 1, wl);
        break;
    case 0x0B:                            /* SET_INTERFACE */
        ep0_reply(0, 0, 0);
        break;
    case 0x01: {                          /* CLEAR_FEATURE(ENDPOINT_HALT): reset data toggle */
        uint8_t e = (uint8_t)(wi & 0x0Fu);
        if (bm == 0x02u && (e == 1u || e == 2u)) {
            uint16_t r = EPR(e);
            if (wi & 0x80u) {
                EPR(e) = (uint16_t)((r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | (r & EP_DTOG_TX));
            } else {
                EPR(e) = (uint16_t)((r & EP_KEEP) | EP_CTR_RX | EP_CTR_TX | (r & EP_DTOG_RX));
            }
        }
        ep0_reply(0, 0, 0);
        break;
    }
    default:
        ep0_stall();
        break;
    }
}

static void ep0_in_done(void)
{
    if (addr_pending) {
        USB->DADDR = (uint16_t)(DADDR_EF | addr_val);
        addr_pending = 0;
        return;
    }
    if (ctl_rem != 0u || ctl_zlp) {
        if (ctl_rem == 0u) {
            ctl_zlp = 0;
        }
        ep0_tx_next();
    }
}

/* ------------------------------------------------------------------ */
/* Reset & interrupt                                                   */
/* ------------------------------------------------------------------ */
static void usb_reset(void)
{
    for (uint8_t n = 0; n <= 2u; n++) {
        BT_ADDR_TX(n)  = PMA_TXA(n);
        BT_COUNT_TX(n) = 0;
        BT_ADDR_RX(n)  = PMA_RXA(n);
        BT_COUNT_RX(n) = RXCNT_64B;
    }
    ctl_rem = 0;
    ctl_zlp = 0;
    addr_pending = 0;

    ep_config(0, EP_TYPE_CTRL, STAT_TX_NAK, STAT_RX_VALID);
    usb_set_config(0);
    USB->DADDR = DADDR_EF;                /* alamat 0, USB aktif */
}

static void ep_correct_transfer(uint8_t n)
{
    uint16_t r = EPR(n);

    if (r & EP_CTR_RX) {
        if (n == 0u) {
            ep_ack_rx(0);
            if (r & EP_SETUP) {
                ep0_setup();
            } else {
                ep_set_rx(0, STAT_RX_VALID);          /* status OUT (ZLP) */
            }
        } else {
            rx_len[n] = (int16_t)(BT_COUNT_RX(n) & 0x3FFu);
            ep_ack_rx(n);                              /* STAT_RX tetap NAK sampai dibaca */
        }
    }
    if (r & EP_CTR_TX) {
        ep_ack_tx(n);
        if (n == 0u) {
            ep0_in_done();
        } else {
            tx_busy[n] = 0;
        }
    }
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    uint16_t s;
    while (((s = USB->ISTR) & (ISTR_CTR | ISTR_RESET)) != 0u) {
        if (s & ISTR_RESET) {
            USB->ISTR = (uint16_t)~ISTR_RESET;
            usb_reset();
            continue;
        }
        ep_correct_transfer((uint8_t)(s & 0x0Fu));
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */
void usb_init(void)
{
    /* Paksa host mendeteksi ulang device: tarik D+ (PA12) ke low sebentar.
       (Pull-up 1.5k eksternal ke 3V3 pada board tetap terpasang.) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
    for (volatile uint32_t i = 0; i < 150000u; i++) { }      /* ~10 ms @72MHz */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_FLOATING);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USB);
    USB->CNTR = CNTR_FRES;                /* PDWN=0, tahan reset */
    for (volatile uint32_t i = 0; i < 1000u; i++) { }        /* tSTARTUP */
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->BTABLE = 0;

    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    USB->CNTR = CNTR_CTRM | CNTR_RESETM;
}

uint8_t usb_is_configured(void)
{
    return configured;
}

uint8_t usb_tx_ready(uint8_t ep)
{
    return (uint8_t)(configured && !tx_busy[ep]);
}

void usb_send(uint8_t ep, const uint8_t *data, uint16_t len)
{
    pma_write(PMA_TXA(ep), data, len);
    BT_COUNT_TX(ep) = len;
    tx_busy[ep] = 1;

    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    ep_set_tx(ep, STAT_TX_VALID);
    __set_PRIMASK(pm);
}

int16_t usb_rx_pending(uint8_t ep)
{
    return rx_len[ep];
}

uint16_t usb_rx_read(uint8_t ep, uint8_t *dst)
{
    int16_t n = rx_len[ep];
    if (n < 0) {
        return 0;
    }
    pma_read(PMA_RXA(ep), dst, (uint16_t)n);
    rx_len[ep] = -1;

    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    ep_set_rx(ep, STAT_RX_VALID);
    __set_PRIMASK(pm);
    return (uint16_t)n;
}
