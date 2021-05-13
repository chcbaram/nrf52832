/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(APP_TIMER)
#include "app_timer.h"
#include <stdlib.h>
#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "app_util_platform.h"




#define MAX_TIMER_COUNTER_VAL     0xFFFFFFFF



/**@brief Timer node type. The nodes will be used form a linked list of running timers. */
typedef struct
{
    uint32_t                    ticks_to_expire;                            /**< Number of ticks from previous timer interrupt to timer expiry. */
    uint32_t                    ticks_at_start;                             /**< Current RTC counter value when the timer was started. */
    uint32_t                    ticks_first_interval;                       /**< Number of ticks in the first timer interval. */
    uint32_t                    ticks_periodic_interval;                    /**< Timer period (for repeating timers). */
    bool                        is_running;                                 /**< True if timer is running, False otherwise. */
    app_timer_mode_t            mode;                                       /**< Timer mode. */
    app_timer_timeout_handler_t p_timeout_handler;                          /**< Pointer to function to be executed when the timer expires. */
    void *                      p_context;                                  /**< General purpose pointer. Will be passed to the timeout handler when the timer expires. */
    void *                      next;                                       /**< Pointer to the next node. */
} timer_node_t;

STATIC_ASSERT(sizeof(timer_node_t) == APP_TIMER_NODE_SIZE);





#define AP_TIMER_MAX_CH         32


typedef struct
{
  uint8_t  timer_en;
  uint32_t timer_pre;
  uint32_t timer_init;

  timer_node_t * p_node;
} timer_isr_t;

static timer_isr_t  timer_isr_tbl[AP_TIMER_MAX_CH];
static volatile uint32_t timer_isr_index = 0;


ret_code_t app_timer_init(void)
{
  uint32_t i;

  for (i=0; i<AP_TIMER_MAX_CH; i++)
  {
    timer_isr_tbl[i].timer_en = 0;
    timer_isr_tbl[i].p_node = NULL;
  }
  return NRF_SUCCESS;
}


ret_code_t app_timer_create(app_timer_id_t const *      p_timer_id,
                            app_timer_mode_t            mode,
                            app_timer_timeout_handler_t timeout_handler)
{
  if (timeout_handler == NULL)
  {
      return NRF_ERROR_INVALID_PARAM;
  }
  if (p_timer_id == NULL)
  {
      return NRF_ERROR_INVALID_PARAM;
  }
  if (((timer_node_t*)*p_timer_id)->is_running)
  {
      return NRF_ERROR_INVALID_STATE;
  }
  if ((timer_isr_index + 1) > AP_TIMER_MAX_CH)
  {
      return NRF_ERROR_INVALID_PARAM;
  }
  timer_node_t * p_node     = (timer_node_t *)*p_timer_id;
  p_node->is_running        = false;
  p_node->mode              = mode;
  p_node->p_timeout_handler = timeout_handler;


  timer_isr_tbl[timer_isr_index].p_node = p_node;


  timer_isr_index++;

  return NRF_SUCCESS;
}

static int32_t getIsrIndex(timer_node_t * p_node)
{
  int32_t ret = -1;
  uint32_t i;

  for (i=0; i<timer_isr_index; i++)
  {
    if (timer_isr_tbl[i].p_node == p_node)
    {
      ret = i;
      break;
    }
  }

  return ret;
}

ret_code_t app_timer_start(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context)
{
  timer_node_t * p_node = (timer_node_t*)timer_id;
  int32_t timer_index;

  if (timer_id == 0)
  {
    return NRF_ERROR_INVALID_STATE;
  }
  if ((timeout_ticks < APP_TIMER_MIN_TIMEOUT_TICKS) || (timeout_ticks > MAX_TIMER_COUNTER_VAL))
  {
    return NRF_ERROR_INVALID_PARAM;
  }
  if (p_node->p_timeout_handler == NULL)
  {
    return NRF_ERROR_INVALID_STATE;
  }
  timer_index = getIsrIndex(p_node);
  if (timer_index < 0)
  {
    return NRF_ERROR_INVALID_PARAM;
  }

  p_node->p_context = p_context;
  p_node->ticks_at_start = app_timer_cnt_get();
  p_node->ticks_to_expire = timeout_ticks;

  timer_isr_tbl[timer_index].timer_en = 1;


  return NRF_SUCCESS;
}


ret_code_t app_timer_stop(app_timer_id_t timer_id)
{
  timer_node_t * p_node = (timer_node_t*)timer_id;
  int32_t timer_index;

  if ((timer_id == NULL) || (p_node->p_timeout_handler == NULL))
  {
      return NRF_ERROR_INVALID_STATE;
  }
  timer_index = getIsrIndex(p_node);
  if (timer_index < 0)
  {
    return NRF_ERROR_INVALID_PARAM;
  }


  p_node->is_running = false;
  timer_isr_tbl[timer_index].timer_en = 0;

  return NRF_SUCCESS;
}


ret_code_t app_timer_stop_all(void)
{
  uint32_t i;

  for (i=0; i<AP_TIMER_MAX_CH; i++)
  {
    timer_isr_tbl[i].timer_en = 0;
  }
  return NRF_SUCCESS;
}


extern uint32_t millis();
uint32_t app_timer_cnt_get(void)
{
  return millis();
}


uint32_t app_timer_cnt_diff_compute(uint32_t   ticks_to,
                                    uint32_t   ticks_from)
{
  return (ticks_to-ticks_from);
}


void app_timer_pause(void)
{

}

void app_timer_resume(void)
{

}

void app_timer_update(void)
{
  uint32_t i;
  timer_node_t * p_node;
  uint32_t time_diff;
  uint32_t time_curr;

  time_curr = app_timer_cnt_get();

  for (i=0; i<timer_isr_index; i++)
  {
    if (timer_isr_tbl[i].timer_en == 1)
    {
      p_node = timer_isr_tbl[i].p_node;

      time_diff = time_curr - p_node->ticks_at_start;

      if (time_diff >= p_node->ticks_to_expire)
      {
        p_node->ticks_at_start = time_curr - (time_diff-p_node->ticks_to_expire);

        if (p_node->mode == APP_TIMER_MODE_SINGLE_SHOT)
        {
          timer_isr_tbl[i].timer_en = 0;
        }

        p_node->p_timeout_handler(p_node->p_context);
      }
    }
  }
}

#endif //NRF_MODULE_ENABLED(APP_TIMER)
