/*
 * log.c
 *
 *  Created on: 2021. 5. 16.
 *      Author: baram
 */


#include "log.h"
#include "uart.h"


#ifdef _USE_HW_LOG


static uint8_t log_ch = LOG_CH;
static char print_buf[256];

#ifdef _USE_HW_ROTS
static osMutexId mutex_lock;
#endif


bool logInit(void)
{
#ifdef _USE_HW_ROTS
  osMutexDef(mutex_lock);
  mutex_lock = osMutexCreate (osMutex(mutex_lock));
#endif

  return true;
}

void logPrintf(const char *fmt, ...)
{
#ifdef _USE_HW_ROTS
  osMutexWait(mutex_lock, osWaitForever);
#endif

  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(print_buf, 256, fmt, args);

  uartWrite(log_ch, (uint8_t *)print_buf, len);
  va_end(args);

#ifdef _USE_HW_ROTS
  osMutexRelease(mutex_lock);
#endif
}



#endif
