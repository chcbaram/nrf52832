/*
 * uart.c
 *
 *  Created on: 2021. 5. 15.
 *      Author: baram
 */


#include "uart.h"
#include "qbuffer.h"
#include "bleuart.h"

#ifdef _USE_HW_UART




typedef struct
{
  bool     is_open;
  uint32_t baud;

  qbuffer_t q_rx;
  uint8_t   q_rx_buf[1024];
  uint8_t   dma_rx_buf[2];
  uint8_t   dma_index;
  NRF_UARTE_Type *uart;
} uart_tbl_t;


static uart_tbl_t uart_tbl[UART_MAX_CH];


static uint32_t uartGetBaudValue(uint32_t baud);





bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    uart_tbl[i].is_open = false;
    uart_tbl[i].baud = 57600;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;


  switch(ch)
  {
    case _DEF_UART1:
      uart_tbl[ch].is_open = true;
      uart_tbl[ch].baud = baud;
      ret = true;
      break;

    case _DEF_UART2:

      uart_tbl[ch].baud = baud;
      uart_tbl[ch].uart = NRF_UARTE0;

      NRF_UARTE0->PSEL.RXD = NRF_GPIO_PIN_MAP(0, 12);
      NRF_UARTE0->PSEL.TXD = NRF_GPIO_PIN_MAP(0, 11);

      NRF_UARTE0->BAUDRATE = uartGetBaudValue(baud);


      qbufferCreate(&uart_tbl[ch].q_rx, uart_tbl[ch].q_rx_buf, 1024);
      uart_tbl[ch].dma_index = 0;

      NRF_UARTE0->RXD.PTR = (uint32_t)&uart_tbl[ch].dma_rx_buf[uart_tbl[ch].dma_index];
      NRF_UARTE0->RXD.MAXCNT = 1;

      NRF_UARTE0->SHORTS   = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);
      NRF_UARTE0->INTENSET = (UARTE_INTENSET_RXSTARTED_Set << UARTE_INTENSET_RXSTARTED_Pos);
      NRF_UARTE0->INTENSET = (UARTE_INTENSET_ENDRX_Set     << UARTE_INTENSET_ENDRX_Pos);

      NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

      NRF_UARTE0->TASKS_STARTRX = 1;

      sd_nvic_SetPriority(UARTE0_UART0_IRQn, 6);
      sd_nvic_EnableIRQ(UARTE0_UART0_IRQn);

      uart_tbl[ch].is_open = true;
      ret = true;
      break;
  }

  return ret;
}

uint32_t uartGetBaudValue(uint32_t baud)
{
  uint32_t ret;

  switch(baud)
  {
    case 1200:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud1200;
      break;

    case 2400:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud2400;
      break;

    case 4800:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud4800;
      break;

    case 9600:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud9600;
      break;

    case 14400:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud14400;
      break;

    case 19200:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud19200;
      break;

    case 38400:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud38400;
      break;

    case 57600:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud57600;
      break;

    case 115200:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud115200;
      break;

    case 230400:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud230400;
      break;

    case 460800:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud460800;
      break;

    case 921600:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud921600;
      break;

    case 1000000:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud1M;
      break;

    default:
      ret = UARTE_BAUDRATE_BAUDRATE_Baud57600;
      break;
  }

  return ret;
}

bool uartClose(uint8_t ch)
{
  return true;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = bleUartAvailable();
      break;

    case _DEF_UART2:
      ret = qbufferAvailable(&uart_tbl[ch].q_rx);
      break;
  }

  return ret;
}

bool uartFlush(uint8_t ch)
{
  uint32_t pre_time;

  pre_time = millis();
  while(uartAvailable(ch))
  {
    if (millis()-pre_time >= 10)
    {
      break;
    }
    uartRead(ch);
  }

  return true;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = bleUartRead();
      break;

    case _DEF_UART2:
      qbufferRead(&uart_tbl[ch].q_rx, &ret, 1);
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  uint32_t pre_time;
  NRF_UARTE_Type *p_uart = uart_tbl[ch].uart;

  if (ch >= UART_MAX_CH) return 0;
  if (uart_tbl[ch].is_open != true) return 0;


  switch(ch)
  {
    case _DEF_UART1:
      ret = bleUartWrite(p_data, length);
      break;

    case _DEF_UART2:
      p_uart->TXD.PTR = (uint32_t)p_data;
      p_uart->TXD.MAXCNT = length;


      p_uart->EVENTS_ENDTX  = 0;
      p_uart->TASKS_STARTTX = 1;

      pre_time = millis();
      while(millis()-pre_time < 100)
      {
        if (p_uart->EVENTS_ENDTX > 0)
        {
          ret = length;
          break;
        }
      }
      break;
  }

  return ret;
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[256];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, 256, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  ret = uart_tbl[ch].baud;

  return ret;
}





void UARTE0_UART0_IRQHandler(void)
{
  uart_tbl_t *p_uart = &uart_tbl[_DEF_UART2];


  if (NRF_UARTE0->EVENTS_ENDRX > 0)
  {
    qbufferWrite(&p_uart->q_rx, &p_uart->dma_rx_buf[p_uart->dma_index^1], 1);

    NRF_UARTE0->EVENTS_ENDRX = 0;
  }

  if (NRF_UARTE0->EVENTS_RXSTARTED > 0)
  {
    p_uart->dma_index ^= 1;
    NRF_UARTE0->RXD.PTR = (uint32_t)&p_uart->dma_rx_buf[p_uart->dma_index];

    NRF_UARTE0->EVENTS_RXSTARTED = 0;
  }
}

#endif
