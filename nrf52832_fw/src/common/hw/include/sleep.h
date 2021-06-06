/*
 * sleep.h
 *
 *  Created on: 2021. 6. 6.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_SLEEP_H_
#define SRC_COMMON_HW_INCLUDE_SLEEP_H_

#include "hw_def.h"


#ifdef _USE_HW_SLEEP


bool sleepInit(void);
bool sleepEnter(void);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_SLEEP_H_ */
