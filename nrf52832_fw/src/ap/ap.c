/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"


void ledISR(void *args)
{
  ledToggle(_DEF_LED3);
}

void apInit(void)
{
  swtimer_handle_t h_led;

  cliOpen(_DEF_UART2, 57600);

  h_led = swtimerGetHandle();
  swtimerSet(h_led, 1000, LOOP_TIME, ledISR, NULL);
  swtimerStart(h_led);
}

void apMain(void)
{
  uint32_t pre_time;
  button_obj_t btn;
  uint16_t     btn_count = 0;


  buttonObjCreate(&btn, 0, 200);


  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }
    cliMain();

    if (buttonObjGetClicked(&btn, 60) == true)
    {
      btn_count++;
      logPrintf("ButtonClicked %d\n", btn_count);
    }
  }
}

