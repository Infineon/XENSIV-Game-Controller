/*
* trigger_switch.h
*
*  Created on: 22.12.2023
*      Author: ErnsMatthias
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
*/


#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#ifndef TRIGGER_SWITCH_H_
#define TRIGGER_SWITCH_H_

/*******************************************************************************
* Macros
*******************************************************************************/
#define TRIGGERL1_INTERRUPT_PRIORITY (7u)
#define TRIGGERR1_INTERRUPT_PRIORITY (7u)



/*******************************************************************************
* Global Variables
*******************************************************************************/
extern volatile bool triggerL1_intr_flag;
extern volatile bool triggerR1_intr_flag;
extern cyhal_gpio_callback_data_t triggerL1_btn_callback_data;
extern cyhal_gpio_callback_data_t triggerR1_btn_callback_data;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void triggerR1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void triggerL1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

void trigger_R1L1_init();
void trigger_R2L2_init();

uint16_t trigger_getButtonStates();

bool get_triggerR2_state();
bool get_triggerL2_state();

bool get_triggerL1_state();
bool get_triggerR1_state();


#endif /* TRIGGER_SWITCH_H_ */
