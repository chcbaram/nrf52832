/*
 * esp.h
 *
 *  Created on: 2021. 2. 13.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_ESP_H_
#define SRC_COMMON_HW_INCLUDE_ESP_H_

#include "hw_def.h"


#ifdef _USE_HW_ESP


bool espInit(void);
bool espOpen(uint8_t ch, uint32_t baud);
bool espIsOpen(void);

uint32_t espAvailable(void);
uint32_t espWrite(uint8_t *p_data, uint32_t length);
uint8_t  espRead(void);
uint32_t espPrintf(char *fmt, ...);

void espLogEnable(void);
void espLogDisable(void);
bool espCmd(const char *cmd_str, uint32_t timeout);
bool espPing(uint32_t timeout);
bool espWaitOK(uint32_t timeout);
bool espConnectWifi(char *ssd_str, char *pswd_str, uint32_t timeout);
bool espClientBegin(char *ip_str, char *port_str, uint32_t timeout);
bool espClientEnd(void);

#endif


#endif /* SRC_COMMON_HW_INCLUDE_ESP_H_ */
