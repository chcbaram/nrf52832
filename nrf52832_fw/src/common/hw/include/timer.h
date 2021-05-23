/*
 * timer.h
 *
 *  Created on: 2021. 5. 23.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_TIMER_H_
#define SRC_COMMON_HW_INCLUDE_TIMER_H_

#include "hw_def.h"


#ifdef _USE_HW_TIMER

#define TIMER_MAX_CH          HW_TIMER_MAX_CH


bool timerInit(void);
void timerSetFreq(uint8_t ch, uint32_t freq);
void timerSetPeriod(uint8_t ch, uint32_t period_us);
void timerSetISR(uint8_t ch, void (*func)(void *), void *arg);
void timerStart(uint8_t ch);
void timerStop(uint8_t ch);


#endif

#endif /* SRC_COMMON_HW_INCLUDE_TIMER_H_ */
