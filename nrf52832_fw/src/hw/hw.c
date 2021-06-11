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
  uint32_t pre_time;
  uint32_t exe_time;

  pre_time = millis();
  bspInit();

  timerInit();
  timerSetPeriod(_DEF_TIMER1, 1000);
  timerSetISR(_DEF_TIMER1, timerISR, NULL);
  timerStart(_DEF_TIMER1);


  swtimerInit();
  logInit();
  cliInit();
  resetInit();
  ledInit();
  gpioInit();
  adcInit();
  buttonInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  logPrintf("\n");
  logPrintf("[ Firmware Begin... ]\r\n");
  resetLog();
  flashInit();
  fsInit();
  spiFlashInit();
  bleUartInit();

  sleepInit();

  exe_time = millis()-pre_time;

  logPrintf("boot time \t: %d ms\r\n", exe_time);
}

void timerISR(void *arg)
{
  swtimerISR();
}
