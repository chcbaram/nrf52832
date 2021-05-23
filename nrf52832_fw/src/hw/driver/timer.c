/*
 * timer.c
 *
 *  Created on: 2021. 5. 23.
 *      Author: baram
 */


#include "timer.h"


typedef struct
{
  bool     is_start;
  uint32_t freq;
  uint32_t period;
  void   (*isr_func)(void *);
  void    *isr_arg;

  NRF_TIMER_Type *timer;
} timer_tbl_t;


static bool is_init = false;
static timer_tbl_t timer_tbl[TIMER_MAX_CH];


void TIMER1_IRQHandler(void)
{
  if (NRF_TIMER1->EVENTS_COMPARE[0] > 0)
  {
    timer_tbl[_DEF_TIMER1].isr_func(timer_tbl[_DEF_TIMER1].isr_arg);

    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
  }
}



bool timerInit(void)
{
  timer_tbl_t *p_handle;

  for (int i=0; i<TIMER_MAX_CH; i++)
  {
    timer_tbl[i].is_start = false;
    timer_tbl[i].isr_func = NULL;
    timer_tbl[i].isr_arg = NULL;

    timer_tbl[i].freq = 1000;
    timer_tbl[i].period = 1000;
  }
  timer_tbl[_DEF_TIMER1].timer = NRF_TIMER1;

  p_handle = &timer_tbl[_DEF_TIMER1];

  p_handle->timer->MODE      = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  p_handle->timer->BITMODE   = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  p_handle->timer->PRESCALER = 4; // 2^4 = 16Mhz/16 = 1MHz, 1us
  p_handle->timer->CC[0]     = p_handle->period - 1;
  p_handle->timer->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;


  sd_nvic_SetPriority(UARTE0_UART0_IRQn, 6);
  sd_nvic_EnableIRQ(UARTE0_UART0_IRQn);

  is_init = true;

  return is_init;
}

void timerSetFreq(uint8_t ch, uint32_t freq)
{
  uint32_t period_us;

  if (freq == 0) return;

  period_us = 1000000 / freq;

  timerSetPeriod(ch, period_us);
}

void timerSetPeriod(uint8_t ch, uint32_t period_us)
{
  timer_tbl_t *p_handle = &timer_tbl[ch];

  if (ch >= TIMER_MAX_CH) return;
  if (period_us == 0) return;


  p_handle->freq = 1000000 / period_us;
  p_handle->period = period_us;
  p_handle->timer->CC[0] = p_handle->period - 1;
}

void timerSetISR(uint8_t ch, void (*func)(void *), void *arg)
{
  timer_tbl_t *p_handle = &timer_tbl[ch];

  if (ch >= TIMER_MAX_CH) return;

  p_handle->isr_func = func;
  p_handle->isr_arg = arg;
}

void timerStart(uint8_t ch)
{
  timer_tbl_t *p_handle = &timer_tbl[ch];

  if (ch >= TIMER_MAX_CH) return;
  if (p_handle->isr_func == NULL) return;


  if (p_handle->is_start == true)
  {
    timerStop(ch);
  }

  p_handle->timer->INTENSET = TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos;
  p_handle->timer->TASKS_START = 1;
}

void timerStop(uint8_t ch)
{
  timer_tbl_t *p_handle = &timer_tbl[ch];

  if (ch >= TIMER_MAX_CH) return;

  if (p_handle->is_start == true)
  {
    p_handle->timer->TASKS_STOP = 1;
    p_handle->timer->TASKS_CLEAR = 1;
    p_handle->timer->INTENCLR = TIMER_INTENCLR_COMPARE0_Clear << TIMER_INTENCLR_COMPARE0_Pos;

    p_handle->is_start = false;
  }
}
