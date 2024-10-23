/******************************************************************************
* File Name: adc.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: 
*
* 31.05.2024
*
*****************************************************************************
* Copyright (C) 2024 Infineon Technologies AG. All rights reserved.
*
* Infineon Technologies AG (INFINEON) is supplying this file for use
* exclusively with Infineon's products. This file can be freely
* distributed within development tools and software supporting such microcontroller
* products.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT, INCIDENTAL,
* ASPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "gameController_boardControl.h"

/*******************************************************************************
* Defines
*******************************************************************************/

#define MICRO_TO_MILLI_CONV_RATIO (1000u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* ADC Objects */
cyhal_adc_t adc_obj;
cyhal_adc_t adc_obj_2;


/* VBat ADC Channel Object */
cyhal_adc_channel_t adc_chan_VBat_obj;
cyhal_adc_channel_t adc_obj_tmr_lx;
cyhal_adc_channel_t adc_obj_tmr_ly;
cyhal_adc_channel_t adc_obj_tmr_rx;
cyhal_adc_channel_t adc_obj_tmr_ry;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

void adc_Vbat_channel_init(void)
{
	cy_rslt_t result;
	result = cyhal_adc_init(&adc_obj, PIN_VBAT_ADC_DIVIDER, NULL);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("VBat ADC initialization failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}

	/* ADC channel configuration */
	const cyhal_adc_channel_config_t channel_config = {
			.enable_averaging = true,  // Disable averaging for channel
			.min_acquisition_ns = 300000u, // Minimum acquisition time set to 1us
			.enabled = true };          // Sample this channel when ADC performs a scan

    result  = cyhal_adc_channel_init_diff(&adc_chan_VBat_obj, &adc_obj, PIN_VBAT_ADC_DIVIDER,
                                          CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("VBat ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }
    printf("VBAT ADC is configured in single channel configuration\r\n\n");
}

void adc_TMR_joystick_channels_init(void)
{
	cy_rslt_t result;

	/* ADC channel configuration */
	const cyhal_adc_channel_config_t channel_config = {
			.enable_averaging = false,  // Disable averaging for channel
			.min_acquisition_ns = 300000u, // Minimum acquisition time set to 1us
			.enabled = true };          // Sample this channel when ADC performs a scan

	result  = cyhal_adc_channel_init_diff(&adc_obj_tmr_lx, &adc_obj, PIN_JOYSTICK_LEFT_X,
										  CYHAL_ADC_VNEG, &channel_config);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("TMR lx ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}
	result  = cyhal_adc_channel_init_diff(&adc_obj_tmr_ly, &adc_obj, PIN_JOYSTICK_LEFT_Y,
										  CYHAL_ADC_VNEG, &channel_config);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("TMR ly ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}
//	result  = cyhal_adc_channel_init_diff(&adc_obj_tmr_rx, &adc_obj, PIN_JOYSTICK_RIGHT_X,
//										  CYHAL_ADC_VNEG, &channel_config);
//	if(result != CY_RSLT_SUCCESS)
//	{
//		printf("TMR rx ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
//		CY_ASSERT(0);
//	}
	result  = cyhal_adc_channel_init_diff(&adc_obj_tmr_ry, &adc_obj, PIN_JOYSTICK_RIGHT_Y,
										  CYHAL_ADC_VNEG, &channel_config);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("TMR ry ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}
	printf("VBAT ADC is configured in single channel configuration\r\n\n");
}



int32_t get_Vbat()
{
	int32_t adc_result_0 = 0;
	/* Read input voltage, convert it to millivolts and print input voltage */
	adc_result_0 = cyhal_adc_read_uv(&adc_chan_VBat_obj) / MICRO_TO_MILLI_CONV_RATIO;
	adc_result_0 = adc_result_0*2; //Voltage divider 1:2
	return adc_result_0;
}

uint8_t get_SOCBat(uint32_t vBat)
{
	uint8_t SOC = 0;

	uint32_t SocTable[20] = {
			4150, 4110, 4080, 4020,
			3980, 3950, 3910, 3870, 3850,
			3840, 3820, 3800, 3790, 3770,
			3750, 3730, 3710, 3690, 3610, 3270
	};

	for(int8_t i = 19; i>=0; i--)
	{

		if(vBat>=SocTable[i])
		{
			SOC = (20-i)*5;
		}
	}
	return SOC;
}

uint8_t getSOCBat_Mean()
{
	int32_t vBat = 0;
	for(uint8_t i = 0; i<10; i++)
	{
		int32_t vBatMeas = get_Vbat();
		if(vBatMeas == 0) vBatMeas = 4220;
		vBat = vBat + vBatMeas;
	}
	vBat = vBat/10;
	uint8_t SOC = get_SOCBat(vBat);
	if(SOC==0) SOC = 100;
	return SOC;
}
