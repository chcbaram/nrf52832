/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "hw.h"


static void timerISR(void *arg);




void hwInit(void)
{
  bspInit();

  timerInit();
  timerSetPeriod(_DEF_TIMER1, 1000);
  timerSetISR(_DEF_TIMER1, timerISR, NULL);
  timerStart(_DEF_TIMER1);

  swtimerInit();
  logInit();
  cliInit();
  ledInit();
  adcInit();
  buttonInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  logPrintf("\n");
  logPrintf("[ Firmware Begin... ]\r\n");

  flashInit();
  fsInit();
  bleUartInit();
}

void timerISR(void *arg)
{
  swtimerISR();
}
