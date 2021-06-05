/*
 * bsp.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_


#include "def.h"

#if 0
#define _USE_LOG_PRINT    0

#if _USE_LOG_PRINT
#define logPrintf(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define logPrintf(fmt, ...)
#endif
#else
void logPrintf(const char *fmt, ...);
#endif


#include "nrf_systick.h"
#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"
#include "nrf_nvic.h"
#include "nrf_sdh_soc.h"

#include "nrfx_clock.h"

void bspInit(void);

void delay(uint32_t ms);
uint32_t millis(void);


#endif /* SRC_BSP_BSP_H_ */
