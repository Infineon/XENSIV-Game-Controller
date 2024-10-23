/******************************************************************************
* File Name: adc.h
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
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

#ifndef ADC_H_
#define ADC_H_

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define MICRO_TO_MILLI_CONV_RATIO (1000u)

/*******************************************************************************
* External variables
*******************************************************************************/
extern cyhal_adc_channel_t adc_chan_VBat_obj;
extern cyhal_adc_channel_t adc_obj_tmr_rx;
extern cyhal_adc_channel_t adc_obj_tmr_ry;
extern cyhal_adc_channel_t adc_obj_tmr_lx;
extern cyhal_adc_channel_t adc_obj_tmr_ly;

/*******************************************************************************
* Funciton Declarations
*******************************************************************************/

void adc_Vbat_channel_init(void);

void adc_TMR_joystick_channels_init(void);

int32_t get_Vbat();

uint8_t get_SOCBat(uint32_t vBat);

uint8_t getSOCBat_Mean();

#endif /* ADC_H_ */
