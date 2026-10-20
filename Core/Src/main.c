/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_bulk.h"
#include "uart_dma.h"
#include "shift_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VER_MAJOR      1u
#define FW_VER_MINOR      0u
#define SR_MAX_BYTES      60u          /* 64 - 2 byte header paket command */
#define UART_DEFAULT_BAUD 115200u

/* Command pada EP2 (OUT). Setiap command dibalas 1 paket di EP2 (IN): [cmd][status]... */
#define CMD_SR_WRITE   0x01u           /* [0x01][n][d0..dn-1]        -> geser n byte + latch */
#define CMD_UART_BAUD  0x02u           /* [0x02][baud LE 4 byte]     -> ubah baudrate UART   */
#define CMD_INFO       0x03u           /* [0x03]                     -> [03][00][maj][min][SR_MAX] */

#define ST_OK          0x00u
#define ST_BAD_ARG     0x01u
#define ST_UNKNOWN     0xFFu


#define U2U_SIZE 512u                  /* pangkat 2 */
#define U2U_MASK (U2U_SIZE - 1u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t           u2u_buf[U2U_SIZE];
static volatile uint16_t u2u_head;     /* ditulis ISR (callback)  */
static volatile uint16_t u2u_tail;     /* ditulis main loop       */
static uint8_t           zlp_pending;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Dipanggil dari ISR begitu UART berhenti menerima data (IDLE) / buffer DMA setengah penuh */
void uart_rx_callback(const uint8_t *data, uint16_t len)
{
    uint16_t h = u2u_head;
    uint16_t freeb = (uint16_t)((u2u_tail - h - 1u) & U2U_MASK);

    if (len > freeb) {
        len = freeb;                   /* ring penuh: data yang kelebihan dibuang */
    }
    for (uint16_t i = 0; i < len; i++) {
        u2u_buf[(h + i) & U2U_MASK] = data[i];
    }
    u2u_head = (uint16_t)((h + len) & U2U_MASK);
}

static void pump_uart_to_usb(void)
{
    if (!usb_tx_ready(1)) {
        return;
    }
    uint16_t t = u2u_tail;
    uint16_t avail = (uint16_t)((u2u_head - t) & U2U_MASK);

    if (avail) {
        uint8_t pkt[USB_EP_SIZE];
        uint16_t n = (avail > USB_EP_SIZE) ? USB_EP_SIZE : avail;
        for (uint16_t i = 0; i < n; i++) {
            pkt[i] = u2u_buf[(t + i) & U2U_MASK];
        }
        u2u_tail = (uint16_t)((t + n) & U2U_MASK);
        zlp_pending = (n == USB_EP_SIZE);
        usb_send(1, pkt, n);
    } else if (zlp_pending) {
        /* paket terakhir tepat 64 byte -> kirim ZLP agar transfer bulk di host selesai */
        zlp_pending = 0;
        usb_send(1, 0, 0);
    }
}

/* ------------------------------------------------------------------ */
/* USB EP1 OUT -> UART TX (DMA)                                        */
/* ------------------------------------------------------------------ */
static void pump_usb_to_uart(void)
{
    int16_t n = usb_rx_pending(1);
    if (n < 0) {
        return;
    }
    if (uart_dma_tx_free() < (uint16_t)n) {
        return;                        /* ring TX penuh: biarkan NAK (flow control ke host) */
    }
    uint8_t tmp[USB_EP_SIZE];
    uint16_t len = usb_rx_read(1, tmp);
    uart_dma_write(tmp, len);
}

/* ------------------------------------------------------------------ */
/* Command channel (EP2)                                               */
/* ------------------------------------------------------------------ */
static void handle_command(void)
{
    if (usb_rx_pending(2) < 0 || !usb_tx_ready(2)) {
        return;                        /* tunggu respons sebelumnya dibaca host */
    }

    uint8_t  buf[USB_EP_SIZE];
    uint8_t  rsp[8];
    uint16_t n = usb_rx_read(2, buf);
    uint16_t rl = 2;

    if (n == 0u) {
        return;                        /* ZLP, abaikan */
    }
    rsp[0] = buf[0];
    rsp[1] = ST_OK;

    switch (buf[0]) {
    case CMD_SR_WRITE: {
        uint8_t cnt = (n >= 2u) ? buf[1] : 0u;
        if (cnt == 0u || cnt > SR_MAX_BYTES || (uint16_t)(cnt + 2u) > n) {
            rsp[1] = ST_BAD_ARG;
        } else {
            shift_reg_write(&buf[2], cnt);
        }
        break;
    }
    case CMD_UART_BAUD: {
        if (n < 5u) {
            rsp[1] = ST_BAD_ARG;
        } else {
            uint32_t baud = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
                            ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
            if (baud < 1200u || baud > 4500000u) {
                rsp[1] = ST_BAD_ARG;
            } else {
                uart_dma_set_baud(baud);
            }
        }
        break;
    }
    case CMD_INFO:
        rsp[2] = FW_VER_MAJOR;
        rsp[3] = FW_VER_MINOR;
        rsp[4] = SR_MAX_BYTES;
        rl = 5;
        break;
    default:
        rsp[1] = ST_UNKNOWN;
        break;
    }
    usb_send(2, rsp, rl);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);   /* PLL 72 MHz / 1.5 = 48 MHz untuk USB */
  shift_reg_init();                  /* juga menonaktifkan JTAG (PA15/PB3 bebas) */
  uart_dma_init(UART_DEFAULT_BAUD);
  usb_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  pump_usb_to_uart();
	  pump_uart_to_usb();
	  handle_command();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
