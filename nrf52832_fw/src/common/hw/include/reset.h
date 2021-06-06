/*
 * reset.h
 *
 *  Created on: 2020. 12. 9.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_RESET_H_
#define SRC_COMMON_HW_INCLUDE_RESET_H_

#include "hw_def.h"


#ifdef _USE_HW_RESET

typedef enum
{
  RESET_MODE_POWER,
  RESET_MODE_REBOOT,
  RESET_MODE_SLEEP,
} reset_mode_t;



bool resetInit(void);
void resetLog(void);
void resetSetMode(reset_mode_t mode);
void resetGetMode(reset_mode_t *p_mode);
void resetToBoot(uint32_t timeout);
void resetToReboot(reset_mode_t mode);


uint32_t resetGetCount(void);

#endif


#endif /* SRC_COMMON_HW_INCLUDE_RESET_H_ */
