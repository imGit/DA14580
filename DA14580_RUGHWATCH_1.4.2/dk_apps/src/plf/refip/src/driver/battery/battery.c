/**
****************************************************************************************
*
* @file battery.c
*
* @brief Battery driver. Provides Battery level. Uses ADC module to get current voltage.
*
* Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
*
* <bluetooth.support@diasemi.com> and contributors.
*
****************************************************************************************
*/
#include "adc.h"
#include "battery.h"

#define JPLUS_BAT_TYPE	1//gsx,custom battery type

uint8_t old_batt_level __attribute__((section("retention_mem_area0"), zero_init)); 

/**
 ****************************************************************************************
 * ADC module Functions
 ****************************************************************************************
*/

/**
 ****************************************************************************************
 * @brief Calculates battery level percentage for CR2032 batteries
 *
 * @param[in] adc_sample  adc sample 
 *
 * @return Battery level. 0 - 100
 ****************************************************************************************
 */

uint8_t batt_cal_cr2032(uint16_t adc_sample)
{
    uint8_t batt_lvl;
	
    //1705=3.0V, 1137=2V
    if(adc_sample >= 1137)
        batt_lvl = (adc_sample - 1137)*100/568;
    else
        batt_lvl = 0;
    return batt_lvl;
}

#ifdef JPLUS_BAT_TYPE
uint8_t batt_cal_jplus(uint16_t adc_sample)
{
    uint8_t batt_lvl;
     //810=2.5V,1370=4.2V
    if(adc_sample >= 806)
        batt_lvl = (adc_sample-806)*100/558;//558Ϊ����(100%),,gsx
    else
        batt_lvl = 0;
		
		if(adc_sample >= 1370)	//��������
				batt_lvl = 100;
		
    return batt_lvl;
}
#endif

/**
 ****************************************************************************************
 * @brief Reads current voltage from adc module and returns battery level. 
 *
 * @param[in] batt_type     Battery type. Supported types defined in battery.h
 *
 * @return Battery level. 0 - 100%
 ****************************************************************************************
 */

uint8_t battery_get_lvl(uint8_t batt_type)
{
	uint8_t batt_lvl;
	uint16_t adc_sample;
	volatile int i;

	adc_init(GP_ADC_SE, GP_ADC_SIGN);
	
#if JPLUS_BAT_TYPE
	adc_enable_channel(ADC_CHANNEL_P01);//ADC_CHANNEL_VBAT3V
#else	
	adc_enable_channel(ADC_CHANNEL_VBAT3V);//ADC_CHANNEL_P00
#endif
	
  adc_sample = adc_get_sample();

	adc_init(GP_ADC_SE, 0);
  
#if JPLUS_BAT_TYPE
	adc_enable_channel(ADC_CHANNEL_P01);//ADC_CHANNEL_VBAT3V
#else	
	adc_enable_channel(ADC_CHANNEL_VBAT3V);//ADC_CHANNEL_P00
#endif

	adc_sample += adc_get_sample();
	
	adc_disable();
	
    adc_sample >>= 4;
    adc_sample <<= 4;
    
    if(old_batt_level == 0)
        old_batt_level = 0xFF;
    
    if(batt_lvl > old_batt_level)
        batt_lvl = old_batt_level;
    
    switch (batt_type)
    {
        case BATT_CR2032:
            batt_lvl = batt_cal_cr2032(adc_sample);
            break;
				case BATT_JPLUS:		//gsx,custom.
            batt_lvl = batt_cal_jplus(adc_sample);
            break;
        default:
            batt_lvl = 0;
    }   
    
	return batt_lvl;
}
