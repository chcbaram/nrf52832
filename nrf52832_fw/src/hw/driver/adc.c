/*
 * adc.c
 *
 *  Created on: 2021. 5. 17.
 *      Author: baram
 */


#include "adc.h"
#include "cli.h"



#ifdef _USE_HW_ADC



static bool is_init = false;



static int16_t adc_dma_data[ADC_MAX_CH];
static int16_t adc_data[ADC_MAX_CH];


#ifdef _USE_HW_CLI
static void cliAdc(cli_args_t *args);
#endif


bool adcInit(void)
{
  uint32_t i;
  uint32_t reg_config;
  NRF_SAADC_Type *p_adc = NRF_SAADC;


  for (i=0; i<ADC_MAX_CH; i++)
  {
    adc_dma_data[i] = 0;
    adc_data[i] = 0;
  }

  for (i=0; i<8; i++)
  {
    p_adc->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
    p_adc->CH[i].PSELN = SAADC_CH_PSELN_PSELN_NC;
  }

  reg_config = (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
               (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
               (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
               (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
               (SAADC_CH_CONFIG_TACQ_40us       << SAADC_CH_CONFIG_TACQ_Pos) |
               (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos) |
               (SAADC_CH_CONFIG_BURST_Disabled  << SAADC_CH_CONFIG_BURST_Pos);

  //-- Battery Voltage
  //
  p_adc->CH[0].PSELP  = SAADC_CH_PSELP_PSELP_AnalogInput0;
  p_adc->CH[0].PSELN  = SAADC_CH_PSELN_PSELN_NC;
  p_adc->CH[0].CONFIG = reg_config;


  //-- VDD/2
  //
  p_adc->CH[1].PSELP  = SAADC_CH_PSELP_PSELP_AnalogInput1;
  p_adc->CH[1].PSELN  = SAADC_CH_PSELN_PSELN_NC;
  p_adc->CH[1].CONFIG = reg_config;


  //-- VDD
  //
  p_adc->CH[2].PSELP  = SAADC_CH_PSELP_PSELP_VDD;
  p_adc->CH[2].PSELN  = SAADC_CH_PSELN_PSELN_NC;
  p_adc->CH[2].CONFIG = reg_config;


  p_adc->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
  p_adc->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
  p_adc->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;


  p_adc->RESULT.PTR = (uint32_t)&adc_dma_data[0];
  p_adc->RESULT.MAXCNT = ADC_MAX_CH;

  p_adc->EVENTS_DONE = 0;
  p_adc->EVENTS_END = 0;
  p_adc->EVENTS_RESULTDONE = 0;
  p_adc->EVENTS_STARTED = 0;
  p_adc->EVENTS_STOPPED = 0;


  p_adc->ENABLE = 1;


  //-- PPI
  //
  NRF_PPI->CH[_USE_PPI_ADC_0].EEP = (uint32_t)&p_adc->EVENTS_END;
  NRF_PPI->CH[_USE_PPI_ADC_0].TEP = (uint32_t)&p_adc->TASKS_START;
  NRF_PPI->CH[_USE_PPI_ADC_1].EEP = (uint32_t)&p_adc->EVENTS_DONE;
  NRF_PPI->CH[_USE_PPI_ADC_1].TEP = (uint32_t)&p_adc->TASKS_SAMPLE;


  NRF_PPI->CHENSET = (1<<0) |
                     (1<<1);

  p_adc->TASKS_START = 1;
  delay(1);
  p_adc->TASKS_SAMPLE = 1;


#ifdef _USE_HW_CLI
  cliAdd("adc", cliAdc);
#endif

  is_init = true;

  return true;
}

uint32_t adcRead(uint8_t ch)
{
  uint32_t adc_value;


  for (int i=0; i<ADC_MAX_CH; i++)
  {
    if (adc_dma_data[i] > 0)
    {
      adc_data[i] = adc_dma_data[i];
    }
    else
    {
      adc_data[i] = 0;
    }
  }

  adc_value = adc_data[ch];

  return adc_value;
}

uint32_t adcRead8(uint8_t ch)
{
  return adcRead(ch)>>4;
}

uint32_t adcRead10(uint8_t ch)
{
  return adcRead(ch)>>2;
}

uint32_t adcRead12(uint8_t ch)
{
  return adcRead(ch);
}

uint32_t adcRead16(uint8_t ch)
{
  return adcRead(ch)<<4;
}

uint32_t adcReadVoltage(uint8_t ch)
{
  return adcConvVoltage(ch, adcRead(ch));
}

uint32_t adcReadCurrent(uint8_t ch)
{

  return adcConvCurrent(ch, adcRead(ch));
}

uint32_t adcConvVoltage(uint8_t ch, uint32_t adc_value)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case 0:
    case 1:
      ret  = (uint32_t)((adc_value * 3600 * 2 * 10) / (4095*10));
      ret += 5;
      ret /= 10;
      break;

    case 2:
      ret  = (uint32_t)((adc_value * 3600 * 1 * 10) / (4095*10));
      ret += 5;
      ret /= 10;
      break;
  }

  return ret;
}

uint32_t adcConvCurrent(uint8_t ch, uint32_t adc_value)
{
  return 0;
}

uint8_t  adcGetRes(uint8_t ch)
{
  return 12;
}





#ifdef _USE_HW_CLI
void cliAdc(cli_args_t *args)
{
  bool ret = true;


  if (args->argc == 1)
  {
    if (args->isStr(0, "show") == true)
    {
      while(cliKeepLoop())
      {
        for (int i=0; i<ADC_MAX_CH; i++)
        {
          cliPrintf("%04d ", adcRead(i));
        }
        cliPrintf("\r\n");
        delay(50);
      }
    }
    else
    {
      ret = false;
    }
  }
  else if (args->argc == 2)
  {
    if (args->isStr(0, "show") == true && args->isStr(1, "voltage") == true)
    {
      while(cliKeepLoop())
      {
        for (int i=0; i<ADC_MAX_CH; i++)
        {
          uint32_t adc_data;

          adc_data = adcReadVoltage(i);

          cliPrintf("%d.%02dV ", adc_data/100, adc_data%100);
        }
        cliPrintf("\r\n");
        delay(50);
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
    cliPrintf( "adc show\n");
    cliPrintf( "adc show voltage\n");
  }
}
#endif


#endif
