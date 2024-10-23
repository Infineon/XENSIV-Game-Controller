/*
* trigger_switch.c
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "trigger.h"

#include "gameController_boardControl.h"
#include "TLV493D.h"
#include "TLE75008.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/

/*global trigger switch variables*/
volatile bool triggerL1_intr_flag = false;
volatile bool triggerR1_intr_flag = false;
cyhal_gpio_callback_data_t triggerL1_btn_callback_data;
cyhal_gpio_callback_data_t triggerR1_btn_callback_data;

TLx493D_t tlv493d_R2;
TLx493D_t tlv493d_L2;

uint8_t L1_rumble_flag = 0;
uint8_t R1_rumble_flag = 0;
uint8_t L2_rumble_flag = 0;
uint8_t R2_rumble_flag = 0;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

void trigger_R2L2_init()
{
	uint8_t error_flag = 0;
	uint8_t init_retry = 0;
	printf("L2 trigger ");
	error_flag = TLV493D_init(&tlv493d_L2, PIN_POWER_L2, TLV493D_IIC_ADDR_A1);
	while(error_flag&&init_retry<5)
	{
		TLV493D_deinit(&tlv493d_L2, PIN_POWER_L2, TLV493D_IIC_ADDR_A1);
		cyhal_system_delay_ms(5);
		printf("Retry L2 trigger ");
		error_flag = TLV493D_init(&tlv493d_L2, PIN_POWER_L2, TLV493D_IIC_ADDR_A1);
		init_retry++;
	}
	init_retry = 0;
	if(error_flag)
	{
		printf("Trigger L2 can not be initialized!\r\n");
		TLV493D_deinit(&tlv493d_L2, PIN_POWER_L2, TLV493D_IIC_ADDR_A1);
		tlx493d_init_error_flag |= TLX493D_0X22_L2_ERROR;
		error_flag = 0;
	}

	cyhal_system_delay_ms(5);

	printf("R2 trigger ");
	error_flag = TLV493D_init(&tlv493d_R2, PIN_POWER_R2, TLV493D_IIC_ADDR_A0);
	while(error_flag&&init_retry<5)
	{
		TLV493D_deinit(&tlv493d_R2, PIN_POWER_R2, TLV493D_IIC_ADDR_A0);
		cyhal_system_delay_ms(5);
		printf("Retry R2 trigger ");
		error_flag = TLV493D_init(&tlv493d_R2, PIN_POWER_R2, TLV493D_IIC_ADDR_A0);
		init_retry++;
	}
	init_retry = 0;
	if(error_flag)
	{
		printf("Trigger R2 can not be initialized!\r\n");
		TLV493D_deinit(&tlv493d_R2, PIN_POWER_R2, TLV493D_IIC_ADDR_A0);
		tlx493d_init_error_flag |= TLX493D_0X35_R2_ERROR;
		error_flag = 0;
	}
}


void trigger_R1L1_init()
{
	cy_rslt_t result;
	printf("Switch L1 initialization: ");
	/*Init GPIO as input pin*/
    result = cyhal_gpio_init(PIN_L1_TRIGGER_SWITCH, CYHAL_GPIO_DIR_INPUT,
    		CYHAL_GPIO_DRIVE_PULLUP, 1U);
    if(result != CY_RSLT_SUCCESS)
    {
    	printf("failed! \r\n");
    }
    /* Configure GPIO interrupt */
    triggerL1_btn_callback_data.callback = triggerL1_interrupt_handler;
    cyhal_gpio_register_callback(PIN_L1_TRIGGER_SWITCH,
								&triggerL1_btn_callback_data);
    cyhal_gpio_enable_event(PIN_L1_TRIGGER_SWITCH, CYHAL_GPIO_IRQ_FALL,
    							TRIGGERL1_INTERRUPT_PRIORITY, true);
    if(result == CY_RSLT_SUCCESS)
    {
    	printf("Success! \r\n");
    }

    printf("Switch R1 initialization: ");
	/*Init GPIO as input pin*/
	result = cyhal_gpio_init(PIN_R1_TRIGGER_SWITCH, CYHAL_GPIO_DIR_INPUT,
			CYHAL_GPIO_DRIVE_PULLUP, 1U);
	if(result != CY_RSLT_SUCCESS)
	{
		printf("failed! \r\n");
	}
	/* Configure GPIO interrupt */
	triggerR1_btn_callback_data.callback = triggerR1_interrupt_handler;
	cyhal_gpio_register_callback(PIN_R1_TRIGGER_SWITCH,
								&triggerR1_btn_callback_data);
	cyhal_gpio_enable_event(PIN_R1_TRIGGER_SWITCH, CYHAL_GPIO_IRQ_FALL,
								TRIGGERR1_INTERRUPT_PRIORITY, true);
	if(result == CY_RSLT_SUCCESS)
	{
		printf("Success! \r\n");
	}

}

uint16_t trigger_getButtonStates()
{
	uint16_t trigger_R1_pressed = (get_triggerR1_state());
	uint16_t trigger_L1_pressed = (get_triggerL1_state());
	uint16_t trigger_R2_pressed = (get_triggerR2_state());
	uint16_t trigger_L2_pressed = (get_triggerL2_state());


	uint16_t buttonStates = 0;

	buttonStates|=((trigger_L1_pressed^=1)<<4);
	buttonStates|=((trigger_R1_pressed^=1)<<5);
	buttonStates|=((trigger_L2_pressed)<<6);
	buttonStates|=((trigger_R2_pressed)<<7);

	return buttonStates;
}

bool get_triggerL2_state()
{
	int16_t x, y, z;
	TLV493D_getRawMagneticField(&tlv493d_L2, &x, &y, &z);
	int16_t l = sqrt(x*x+y*y+z*z);

	//printf("x: %i, y: %i, z: %i, l: %i\r\n", x, y, z, l);

	bool L2_state = 0;
	if(l>350)
	{
		L2_state = 1;
		if(!L2_rumble_flag)
		{
			rumble_activate(RUMBLE_LEFT, 70);
			cyhal_system_delay_ms(35);
			rumble_deactivate();
			L2_rumble_flag = 1;
		}
	}else if(l<330&&L2_rumble_flag==1)
	{
		L2_rumble_flag = 0;
		rumble_activate(RUMBLE_LEFT, 70);
		cyhal_system_delay_ms(20);
		rumble_deactivate();
	}



	return L2_state;
}

bool get_triggerR2_state()
{
	int16_t x, y, z;
	TLV493D_getRawMagneticField(&tlv493d_R2, &x, &y, &z);
	int16_t l = sqrt(x*x+y*y+z*z);

	bool R2_state = 0;
	if(l>350)
	{
		R2_state = 1;
		if(!R2_rumble_flag)
		{
			rumble_activate(RUMBLE_RIGHT, 70);
			cyhal_system_delay_ms(35);
			rumble_deactivate();
			R2_rumble_flag = 1;
		}
	}else if(l<330&&R2_rumble_flag==1)
	{
		R2_rumble_flag = 0;
		rumble_activate(RUMBLE_RIGHT, 70);
		cyhal_system_delay_ms(20);
		rumble_deactivate();
	}

	//printf("x: %i, y: %i, z: %i, l: %i\r\n", x, y, z, l);

	return R2_state;
}

bool get_triggerL1_state()
{
	uint8_t state = cyhal_gpio_read(PIN_L1_TRIGGER_SWITCH);
	if(!state)
	{
		if(!L1_rumble_flag)
		{
			rumble_activate(RUMBLE_LEFT, 70);
			cyhal_system_delay_ms(35);
			rumble_deactivate();
			L1_rumble_flag = 1;
		}
	}else if(state&&L1_rumble_flag==1)
	{
		L1_rumble_flag = 0;
		rumble_activate(RUMBLE_LEFT, 70);
		cyhal_system_delay_ms(20);
		rumble_deactivate();
	}
	return state;
}

bool get_triggerR1_state()
{
	uint8_t state = cyhal_gpio_read(PIN_R1_TRIGGER_SWITCH);
	if(!state)
	{
		if(!R1_rumble_flag)
		{
			rumble_activate(RUMBLE_RIGHT, 70);
			cyhal_system_delay_ms(35);
			rumble_deactivate();
			R1_rumble_flag = 1;
		}
	}else if(state&&R1_rumble_flag==1)
	{
		R1_rumble_flag = 0;
		rumble_activate(RUMBLE_RIGHT, 70);
		cyhal_system_delay_ms(20);
		rumble_deactivate();
	}
	return state;
}

void triggerR1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	triggerR1_intr_flag = true;
}


void triggerL1_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	triggerL1_intr_flag = true;
}
