/*
 * button.c
 *
 *  Created on: 2021. 5. 16.
 *      Author: baram
 */


#include "button.h"
#include "cli.h"


#ifdef _USE_HW_BUTTON


typedef struct
{
  uint32_t   pin;
  uint8_t    on_state;
} button_tbl_t;


const button_tbl_t button_tbl[BUTTON_MAX_CH] =
    {
        {NRF_GPIO_PIN_MAP(0, 25), 0},
        {NRF_GPIO_PIN_MAP(0, 26), 0},
        {NRF_GPIO_PIN_MAP(0, 27), 0},
        {NRF_GPIO_PIN_MAP(0,  7), 0},
        {NRF_GPIO_PIN_MAP(0,  8), 0},
    };


#ifdef _USE_HW_CLI
static void cliButton(cli_args_t *args);
#endif


bool buttonInit(void)
{
  bool ret = true;


  for (int i=0; i<BUTTON_MAX_CH; i++)
  {
    nrf_gpio_cfg_input(button_tbl[i].pin, NRF_GPIO_PIN_NOPULL);
  }

#ifdef _USE_HW_CLI
  cliAdd("button", cliButton);
#endif

  return ret;
}

bool buttonGetPressed(uint8_t ch)
{
  bool ret = false;

  if (ch >= BUTTON_MAX_CH)
  {
    return false;
  }

  if (nrf_gpio_pin_read(button_tbl[ch].pin) == button_tbl[ch].on_state)
  {
    ret = true;
  }

  return ret;
}

void buttonObjCreate(button_obj_t *p_obj, uint8_t ch, uint32_t repeat_time)
{
  p_obj->ch = ch;
  p_obj->state = 0;
  p_obj->pre_time = millis();
  p_obj->repeat_time = repeat_time;
}

bool buttonObjGetClicked(button_obj_t *p_obj, uint32_t pressed_time)
{
  bool ret = false;


  switch(p_obj->state)
  {
    case 0:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        p_obj->state = 1;
        p_obj->pre_time = millis();
      }
      break;

    case 1:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        if (millis()-p_obj->pre_time >= pressed_time)
        {
          ret = true; // 버튼 클릭됨
          p_obj->state = 2;
          p_obj->pre_time = millis();
        }
      }
      else
      {
        p_obj->state = 0;
      }
      break;

    case 2:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        if (millis()-p_obj->pre_time >= p_obj->repeat_time)
        {
          p_obj->state = 1;
          p_obj->pre_time = millis();
        }
      }
      else
      {
        p_obj->state = 0;
      }
      break;
  }

  return ret;
}


#ifdef _USE_HW_CLI

void cliButton(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "show"))
  {
    while(cliKeepLoop())
    {
      for (int i=0; i<BUTTON_MAX_CH; i++)
      {
        cliPrintf("%d", buttonGetPressed(i));
      }
      cliPrintf("\n");

      delay(100);
    }

    ret = true;
  }


  if (ret != true)
  {
    cliPrintf("button show\n");
  }
}
#endif



#endif
