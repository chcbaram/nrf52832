/*
 * sleep.c
 *
 *  Created on: 2021. 6. 6.
 *      Author: baram
 */


#include "sleep.h"
#include "cli.h"
#include "reset.h"
#include "led.h"
#include "button.h"


#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif





bool sleepInit(void)
{
  bool ret = true;



#ifdef _USE_HW_CLI
  cliAdd("sleep", cliCmd);
#endif

  return ret;
}

bool sleepEnter(void)
{
  ledToSleep();
  buttonToSleep();

  resetSetMode(RESET_MODE_SLEEP);

  if (sd_power_system_off() != NRF_SUCCESS)
  {
    return false;
  }

  return true;
}



#ifdef _USE_HW_CLI

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "enter"))
  {
    sleepEnter();
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "reset"))
  {
    resetToReboot(RESET_MODE_SLEEP);
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("sleep enter\n");
    cliPrintf("sleep reset\n");
  }
}
#endif

