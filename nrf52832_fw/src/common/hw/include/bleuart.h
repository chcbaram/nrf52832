/*
 * bleuart.h
 *
 *  Created on: 2021. 5. 13.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_BLEUART_H_
#define SRC_COMMON_HW_INCLUDE_BLEUART_H_


#include "hw_def.h"


#ifdef _USE_HW_BLEUART

#define BLEUART_MAX_BUF_LEN     HW_BLEUART_MAX_BUF_LEN


typedef struct
{
  int8_t  rssi;
  uint8_t ch_index;
} ble_uart_rssi_t;

bool bleUartInit(void);
bool bleUartIsInit(void);
bool bleUartIsConnect(void);
bool bleUartGetRssi(ble_uart_rssi_t *p_rssi);

uint32_t bleUartAvailable(void);
bool     bleUartFlush(void);
uint8_t  bleUartRead(void);
uint32_t bleUartWrite(uint8_t *p_data, uint32_t length);

#endif


#endif /* SRC_COMMON_HW_INCLUDE_BLEUART_H_ */
