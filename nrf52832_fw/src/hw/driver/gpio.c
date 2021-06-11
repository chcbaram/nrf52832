/*
 * gpio.c
 *
 *  Created on: 2021. 6. 11.
 *      Author: baram
 */


#include "gpio.h"
#include "cli.h"


#ifdef _USE_HW_GPIO

typedef struct
{
  uint32_t pin;
  uint8_t  mode;
  uint8_t  on_state;
  uint8_t  off_state;
  bool     init_value;
} gpio_tbl_t;


const gpio_tbl_t gpio_tbl[GPIO_MAX_CH] =
    {
        {NRF_GPIO_PIN_MAP(0, 24),  _DEF_OUTPUT, _DEF_HIGH, _DEF_LOW, _DEF_HIGH},      // 0. SPI_FLASH_CS
    };



#ifdef _USE_HW_CLI
static void cliGpio(cli_args_t *args);
#endif



bool gpioInit(void)
{
  bool ret = true;


  for (int i=0; i<GPIO_MAX_CH; i++)
  {
    gpioPinMode(i, gpio_tbl[i].mode);
    gpioPinWrite(i, gpio_tbl[i].init_value);
  }

#ifdef _USE_HW_CLI
  cliAdd("gpio", cliGpio);
#endif

  return ret;
}

bool gpioPinMode(uint8_t ch, uint8_t mode)
{
  bool ret = true;


  if (ch >= GPIO_MAX_CH)
  {
    return false;
  }

  switch(mode)
  {
    case _DEF_INPUT:
      nrf_gpio_cfg_input(gpio_tbl[ch].pin, NRF_GPIO_PIN_NOPULL);
      break;

    case _DEF_INPUT_PULLUP:
      nrf_gpio_cfg_input(gpio_tbl[ch].pin, NRF_GPIO_PIN_PULLUP);
      break;

    case _DEF_INPUT_PULLDOWN:
      nrf_gpio_cfg_input(gpio_tbl[ch].pin, NRF_GPIO_PIN_PULLDOWN);
      break;

    case _DEF_OUTPUT:
      nrf_gpio_cfg( gpio_tbl[ch].pin,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_NOPULL,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_OUTPUT_PULLUP:
      nrf_gpio_cfg( gpio_tbl[ch].pin,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_PULLUP,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_OUTPUT_PULLDOWN:
      nrf_gpio_cfg( gpio_tbl[ch].pin,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_PULLDOWN,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);
      break;
  }

  return ret;
}

void gpioPinWrite(uint8_t ch, uint8_t value)
{
  if (ch >= GPIO_MAX_CH)
  {
    return;
  }

  if (value)
  {
    nrf_gpio_pin_write(gpio_tbl[ch].pin, _DEF_HIGH);
  }
  else
  {
    nrf_gpio_pin_write(gpio_tbl[ch].pin, _DEF_LOW);
  }
}

bool gpioPinRead(uint8_t ch)
{
  bool ret = false;

  if (ch >= GPIO_MAX_CH)
  {
    return false;
  }

  if (nrf_gpio_pin_read(gpio_tbl[ch].pin) > 0)
  {
    ret = true;
  }

  return ret;
}

void gpioPinToggle(uint8_t ch)
{
  if (ch >= GPIO_MAX_CH)
  {
    return;
  }

  nrf_gpio_pin_toggle(gpio_tbl[ch].pin);
}





#ifdef _USE_HW_CLI
void cliGpio(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    while(cliKeepLoop())
    {
      for (int i=0; i<GPIO_MAX_CH; i++)
      {
        cliPrintf("%d", gpioPinRead(i));
      }
      cliPrintf("\n");
      delay(100);
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "read") == true)
  {
    uint8_t ch;

    ch = (uint8_t)args->getData(1);

    while(cliKeepLoop())
    {
      cliPrintf("gpio read %d : %d\n", ch, gpioPinRead(ch));
      delay(100);
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "write") == true)
  {
    uint8_t ch;
    uint8_t data;

    ch   = (uint8_t)args->getData(1);
    data = (uint8_t)args->getData(2);

    gpioPinWrite(ch, data);

    cliPrintf("gpio write %d : %d\n", ch, data);
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("gpio show\n");
    cliPrintf("gpio read ch[0~%d]\n", GPIO_MAX_CH-1);
    cliPrintf("gpio write ch[0~%d] 0:1\n", GPIO_MAX_CH-1);
  }
}
#endif


#endif
