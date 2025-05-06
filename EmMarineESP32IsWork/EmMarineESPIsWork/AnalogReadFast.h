#include "hal/adc_types.h"
	
#pragma once
#include <driver/adc.h>
uint16_t analogReadFast(byte pin) //pin просто для совместимости
{
 
 // adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11);
  return adc1_get_raw(ADC1_CHANNEL_4); //4 пин
}

/*oid analogReadFast0init()
{
  adc_digi_initialize();
  adc_digi_controller_config()
}*/


void analogSetup()
{
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11);
}
uint16_t analogReadFast0() //pin просто для совместимости
{
  
  //adc1_config_width(1);
  
  int a=adc1_get_raw(ADC1_CHANNEL_0);
  //local_adc1_read()
  //Serial.println(a);
  return a ;//0 пин
}



