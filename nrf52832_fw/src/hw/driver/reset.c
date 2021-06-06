/*
 * reset.c
 *
 *  Created on: 2021. 6. 6.
 *      Author: baram
 */


#include "reset.h"
#include "cli.h"
#include "nrf_bootloader_info.h"


#ifdef _USE_HW_RESET

#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif

static reset_mode_t reset_init_mode = RESET_MODE_POWER;




bool resetInit(void)
{
  bool ret = true;
  uint32_t mode = 0;


  if (nrf_sdh_is_enabled() == true)
  {
    sd_power_gpregret_get(1, &mode);
    reset_init_mode = mode;
    sd_power_gpregret_clr(1, 0xffffffff);
  }
  else
  {
    reset_init_mode = NRF_POWER->GPREGRET2;
    NRF_POWER->GPREGRET2 = 0;
  }

#ifdef _USE_HW_CLI
  cliAdd("reset", cliCmd);
#endif

  return ret;
}

void resetLog(void)
{
  switch(reset_init_mode)
  {
    case RESET_MODE_POWER:
      logPrintf("reset mode \t: RESET_FROM_POWER\r\n");
      break;

    case RESET_MODE_REBOOT:
      logPrintf("reset mode \t: RESET_FROM_REBOOT\r\n");
      break;

    case RESET_MODE_SLEEP:
      logPrintf("reset mode \t: RESET_FROM_SLEEP\r\n");
      break;

    default:
      logPrintf("reset mode \t: UNKNOWN(0x%02X)\r\n", reset_init_mode);
      break;
  }
}

void resetToBoot(uint32_t timeout)
{
  sd_power_gpregret_clr(0, 0xffffffff);
  sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
  NVIC_SystemReset();
}

void resetToReboot(reset_mode_t mode)
{
  resetSetMode(mode);
  NVIC_SystemReset();
}

void resetSetMode(reset_mode_t mode)
{
  if (nrf_sdh_is_enabled() == true)
  {
    sd_power_gpregret_set(1, mode);
  }
  else
  {
    NRF_POWER->GPREGRET2 = mode;
  }
}

void resetGetMode(reset_mode_t *p_mode)
{
  if (nrf_sdh_is_enabled() == true)
  {
    uint32_t mode = 0;
    sd_power_gpregret_get(1, &mode);
    *p_mode = mode;
  }
  else
  {
    *p_mode = NRF_POWER->GPREGRET2;
  }
}



#ifdef _USE_HW_CLI

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "boot"))
  {
    resetToBoot(0);
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "reboot"))
  {
    resetToReboot(RESET_MODE_REBOOT);
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("reset boot\n");
    cliPrintf("reset reboot\n");
  }
}
#endif


#endif
