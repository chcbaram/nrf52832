/*
 * hw.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_


#include "hw_def.h"


#include "led.h"
#include "bleuart.h"
#include "uart.h"
#include "cli.h"
#include "log.h"
#include "swtimer.h"
#include "button.h"
#include "adc.h"
#include "timer.h"
#include "flash.h"
#include "fs.h"
#include "sleep.h"
#include "reset.h"
#include "gpio.h"
#include "spi_flash.h"



void hwInit(void);


#endif /* SRC_HW_HW_H_ */
