/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "hw.h"





void hwInit(void)
{
  bspInit();

  swtimerInit();
  logInit();
  cliInit();
  ledInit();
  adcInit();
  buttonInit();
  bleUartInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  logPrintf("\n");
  logPrintf("[ Firmware Begin... ]\r\n");
}
