/*
 * flash.c
 *
 *  Created on: 2021. 5. 30.
 *      Author: baram
 */


#include "flash.h"
#include "cli.h"


#ifdef _USE_HW_FLASH
#ifdef _USE_HW_QSPI
#include "qspi.h"
#endif


#define FLASH_ADDR_OFFSET         0x00000000
#define FLASH_MAX_SIZE            (512*1024)
#define FLASH_SECTOR_SIZE         (4*1024)
#define FLASH_PAGE_SIZE           (256)
#define FLASH_MAX_SECTOR          (FLASH_MAX_SIZE / FLASH_SECTOR_SIZE)


typedef enum
{
  FLASH_EVT_IDLE,
  FLASH_EVT_OK,
  FLASH_EVT_ERROR,
} FlashEvt_t;


static volatile FlashEvt_t flash_evt = FLASH_EVT_IDLE;



#ifdef _USE_HW_CLI
static void cliFlash(cli_args_t *args);
#endif


static bool flashEraseSector(uint32_t sec_number);
static bool flashWritePage(uint32_t dst_addr, uint32_t src_addr);
static void handler_soc_evt(uint32_t evt_id, void * p_context);


NRF_SDH_SOC_OBSERVER(m_soc_observer, 1, handler_soc_evt, NULL);



bool flashInit(void)
{


#ifdef _USE_HW_CLI
  cliAdd("flash", cliFlash);
#endif
  return true;
}

bool flashErase(uint32_t addr, uint32_t length)
{
  bool ret = false;

  int32_t start_sector = -1;
  int32_t end_sector = -1;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiErase(addr - qspiGetAddr(), length);
    return ret;
  }
#endif



  start_sector = -1;
  end_sector = -1;


  for (int i=0; i<FLASH_MAX_SECTOR; i++)
  {
    bool update = false;
    uint32_t start_addr;
    uint32_t end_addr;


    start_addr = i * FLASH_SECTOR_SIZE;
    end_addr   = start_addr + FLASH_SECTOR_SIZE - 1;

    if (start_addr >= addr && start_addr < (addr+length))
    {
      update = true;
    }
    if (end_addr >= addr && end_addr < (addr+length))
    {
      update = true;
    }

    if (addr >= start_addr && addr <= end_addr)
    {
      update = true;
    }
    if ((addr+length-1) >= start_addr && (addr+length-1) <= end_addr)
    {
      update = true;
    }


    if (update == true)
    {
      if (start_sector < 0)
      {
        start_sector = i;
      }
      end_sector = i;
    }
  }

  if (start_sector >= 0)
  {
    for (int i=start_sector; i<=end_sector; i++)
    {
      ret = flashEraseSector(i);
      if (ret != true)
      {
        break;
      }
    }
  }

  return ret;
}

bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint32_t index;
  uint32_t write_length;
  uint32_t write_addr;
  uint32_t buf_page[FLASH_PAGE_SIZE/4];
  uint8_t *buf = (uint8_t *)buf_page;
  uint32_t offset;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiWrite(addr - qspiGetAddr(), p_data, length);
    return ret;
  }
#endif



  index = 0;
  offset = addr%FLASH_PAGE_SIZE;

  if (offset != 0 || length < FLASH_PAGE_SIZE)
  {
    write_addr = addr - offset;
    memcpy(&buf[0], (void *)write_addr, FLASH_PAGE_SIZE);
    memcpy(&buf[offset], &p_data[0], constrain(FLASH_PAGE_SIZE-offset, 0, length));

    ret = flashWritePage(write_addr, (uint32_t)buf);
    if (ret != true)
    {
      return false;
    }

    if (offset == 0 && length < FLASH_PAGE_SIZE)
    {
      index += length;
    }
    else
    {
      index += (FLASH_PAGE_SIZE - offset);
    }
  }


  while(index < length)
  {
    write_length = constrain(length - index, 0, FLASH_PAGE_SIZE);

    ret = flashWritePage(addr + index, (uint32_t)&p_data[index]);
    if (ret != true)
    {
      break;
    }

    index += write_length;

    if ((length - index) > 0 && (length - index) < FLASH_PAGE_SIZE)
    {
      offset = length - index;
      write_addr = addr + index;
      memcpy(&buf[0], (void *)write_addr, FLASH_PAGE_SIZE);
      memcpy(&buf[0], &p_data[index], offset);

      ret = flashWritePage(write_addr, (uint32_t)buf);
      break;
    }
  }


  return ret;
}

bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t *p_byte = (uint8_t *)addr;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiRead(addr - qspiGetAddr(), p_data, length);
    return ret;
  }
#endif

  for (int i=0; i<length; i++)
  {
    p_data[i] = p_byte[i];
  }

  return ret;
}

bool flashEraseSector(uint32_t sec_number)
{
  bool ret = false;
  uint32_t nrf_ret;
  uint32_t pre_time;

  flash_evt = FLASH_EVT_IDLE;

  nrf_ret = sd_flash_page_erase(sec_number);
  if (nrf_ret == NRF_SUCCESS)
  {
    pre_time = millis();
    while(millis()-pre_time < 500)
    {
      if (flash_evt != FLASH_EVT_IDLE)
      {
        break;
      }
    }
  }

  if (flash_evt == FLASH_EVT_OK)
  {
    ret = true;
  }

  return ret;
}

bool flashWritePage(uint32_t dst_addr, uint32_t src_addr)
{
  bool ret = false;
  uint32_t nrf_ret;
  uint32_t pre_time;

  flash_evt = FLASH_EVT_IDLE;

  nrf_ret = sd_flash_write((uint32_t *)dst_addr, (uint32_t const *)src_addr, FLASH_PAGE_SIZE/4);
  if (nrf_ret == NRF_SUCCESS)
  {
    pre_time = millis();
    while(millis()-pre_time < 500)
    {
      if (flash_evt != FLASH_EVT_IDLE)
      {
        break;
      }
    }
  }

  if (flash_evt == FLASH_EVT_OK)
  {
    ret = true;
  }

  return ret;
}

void handler_soc_evt(uint32_t evt_id, void * p_context)
{
  switch (evt_id)
  {
    case NRF_EVT_FLASH_OPERATION_SUCCESS:
      flash_evt = FLASH_EVT_OK;
      break;

    case NRF_EVT_FLASH_OPERATION_ERROR:
      flash_evt = FLASH_EVT_ERROR;
      break;

    default:
      // No implementation needed.
      break;
  }
}



#ifdef _USE_HW_CLI
void cliFlash(cli_args_t *args)
{
  bool ret = true;
  uint32_t i;
  uint32_t addr;
  uint32_t length;
  uint8_t  data;
  uint32_t pre_time;
  bool flash_ret;


  if (args->argc == 1)
  {
    if(args->isStr(0, "info") == true)
    {
      cliPrintf("flash addr  : 0x%X\n", 0x0000000);
    }
    else
    {
      ret = false;
    }
  }
  else if (args->argc == 3)
  {
    if(args->isStr(0, "read") == true)
    {
      addr   = (uint32_t)args->getData(1);
      length = (uint32_t)args->getData(2);

      for (i=0; i<length; i++)
      {
        flash_ret = flashRead(addr+i, &data, 1);

        if (flash_ret == true)
        {
          cliPrintf( "addr : 0x%X\t 0x%02X\n", addr+i, data);
        }
        else
        {
          cliPrintf( "addr : 0x%X\t Fail\n", addr+i);
        }
      }
    }
    else if(args->isStr(0, "erase") == true)
    {
      addr   = (uint32_t)args->getData(1);
      length = (uint32_t)args->getData(2);

      pre_time = millis();
      flash_ret = flashErase(addr, length);

      cliPrintf( "addr : 0x%X\t len : %d %d ms\n", addr, length, (millis()-pre_time));
      if (flash_ret)
      {
        cliPrintf("OK\n");
      }
      else
      {
        cliPrintf("FAIL\n");
      }
    }
    else if(args->isStr(0, "write") == true)
    {
      addr = (uint32_t)args->getData(1);
      data = (uint8_t )args->getData(2);

      pre_time = millis();
      flash_ret = flashWrite(addr, &data, 1);

      cliPrintf( "addr : 0x%X\t 0x%02X %dms\n", addr, data, millis()-pre_time);
      if (flash_ret)
      {
        cliPrintf("OK\n");
      }
      else
      {
        cliPrintf("FAIL\n");
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }


  if (ret == false)
  {
    cliPrintf( "flash info\n");
    cliPrintf( "flash read  [addr] [length]\n");
    cliPrintf( "flash erase [addr] [length]\n");
    cliPrintf( "flash write [addr] [data]\n");
  }

}
#endif


#endif
