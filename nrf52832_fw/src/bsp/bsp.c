/*
 * bsp.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "bsp.h"



extern uint32_t __isr_vector_addr;
volatile uint32_t systick_ms = 0;




void SysTick_Handler(void)
{
  systick_ms++;
}


void bspInit(void)
{
  nrfx_clock_enable();

  //nrfx_clock_hfclk_start();
  //nrfx_clock_lfclk_start();

  if (SD_VERSION_GET(MBR_SIZE) == SD_VERSION)
  {
    sd_softdevice_disable();
    sd_softdevice_vector_table_base_set((uint32_t)&__isr_vector_addr);
  }

  nrf_systick_load_set(SystemCoreClock/1000);
  nrf_systick_csr_set(
      NRF_SYSTICK_CSR_CLKSOURCE_CPU |
      NRF_SYSTICK_CSR_TICKINT_ENABLE |
      NRF_SYSTICK_CSR_ENABLE);
}

void delay(uint32_t ms)
{
  uint32_t pre_time = systick_ms;

  while(systick_ms-pre_time < ms);
}

uint32_t millis(void)
{
  return systick_ms;
}

