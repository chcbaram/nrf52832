/*
 * led.c
 *
 *  Created on: Dec 7, 2020
 *      Author: baram
 */


#include "led.h"
#include "cli.h"


#ifdef _USE_HW_LED

typedef struct
{
  uint32_t  pin;
  uint8_t   on_state;
  uint8_t   off_state;
} led_tbl_t;


led_tbl_t led_tbl[LED_MAX_CH] =
    {
        {NRF_GPIO_PIN_MAP(0, 17), 1, 0}, // LED1
        {NRF_GPIO_PIN_MAP(0, 16), 1, 0}, // LED2
        {NRF_GPIO_PIN_MAP(0, 15), 1, 0}, // LED3
        {NRF_GPIO_PIN_MAP(0, 14), 1, 0}, // LED4
        {NRF_GPIO_PIN_MAP(0, 13), 1, 0}, // LED5
    };


#ifdef _USE_HW_CLI
static void cliLed(cli_args_t *args);
#endif


bool ledInit(void)
{
  bool ret = true;


  for (int i=0; i<LED_MAX_CH; i++)
  {
    nrf_gpio_cfg_output(led_tbl[i].pin);
    ledOff(i);
  }

#ifdef _USE_HW_CLI
  cliAdd("led", cliLed);
#endif

  return ret;
}

bool ledToSleep(void)
{
  for (int i=0; i<LED_MAX_CH; i++)
  {
    ledOff(i);
    nrf_gpio_cfg_default(led_tbl[i].pin);
  }

  return true;
}

void ledOn(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  nrf_gpio_pin_write(led_tbl[ch].pin, led_tbl[ch].on_state);
}

void ledOff(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  nrf_gpio_pin_write(led_tbl[ch].pin, led_tbl[ch].off_state);
}

void ledToggle(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  nrf_gpio_pin_toggle(led_tbl[ch].pin);
}





#ifdef _USE_HW_CLI

void cliLed(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 3 && args->isStr(0, "toggle") == true)
  {
    uint8_t  led_ch;
    uint32_t toggle_time;
    uint32_t pre_time;

    led_ch      = (uint8_t)args->getData(1);
    toggle_time = (uint32_t)args->getData(2);

    if (led_ch > 0)
    {
      led_ch--;
    }

    pre_time = millis();
    while(cliKeepLoop())
    {
      if (millis()-pre_time >= toggle_time)
      {
        pre_time = millis();
        ledToggle(led_ch);
      }
    }

    ret = true;
  }


  if (ret != true)
  {
    cliPrintf("led toggle ch[1~%d] time_ms\n", LED_MAX_CH);
  }
}


#endif


#endif
