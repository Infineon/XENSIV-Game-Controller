/******************************************************************************
* File Name: gameController_boardControl.h
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
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

#ifndef GAMECONTROLLER_BOARDCONTROL_H_
#define GAMECONTROLLER_BOARDCONTROL_H_

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/******************************************************************************
 * Macros
 ******************************************************************************/
/*Pin definitions v1.1*/
//#define PIN_POWER_JOYSTICK_RIGHT	CYBSP_GPIO9
//#define PIN_POWER_JOYSTICK_LEFT		CYBSP_GPIOA2
//#define PIN_POWER_R2				CYBSP_GPIO10
//#define PIN_POWER_L2				CYBSP_GPIOA3
//#define PIN_LED_HOME_RED			CYBSP_GPIO12
//#define PIN_LED_HOME_GREEN			CYBSP_GPIO13
//#define PIN_LED_HOME_BLUE			CYBSP_GPIO11
//#define PIN_VBAT_ADC_DIVIDER		CYBSP_GPIOA5
//#define PIN_SPIDER_IDLE				CYBSP_GPIOA4
//#define PIN_R1_TRIGGER_SWITCH		CYBSP_GPIO5
//#define PIN_L1_TRIGGER_SWITCH		CYBSP_GPIOA1
//#define PIN_RUMBLE_PWM				CYBSP_GPIOA0

/*Pin definitions v1.2 and v1.3*/
#define PIN_R1_TRIGGER_SWITCH		CYBSP_GPIO5
#define PIN_SPIDER_IDLE				CYBSP_GPIO6
#define PIN_POWER_JOYSTICK_RIGHT	CYBSP_GPIO9
#define PIN_POWER_R2				CYBSP_GPIO10
#define PIN_POWER_L2				CYBSP_GPIO11
#define PIN_POWER_JOYSTICK_LEFT		CYBSP_GPIO12
#define PIN_L1_TRIGGER_SWITCH		CYBSP_GPIO13
#define PIN_RUMBLE_PWM				CYBSP_GPIOA0
#define PIN_JOYSTICK_RIGHT_X		CYBSP_GPIOA1
#define PIN_JOYSTICK_RIGHT_Y		CYBSP_GPIOA2
#define PIN_JOYSTICK_LEFT_X			CYBSP_GPIOA3
#define PIN_JOYSTICK_LEFT_Y			CYBSP_GPIOA4
#define PIN_VBAT_ADC_DIVIDER		CYBSP_GPIOA5

#define PIN_I2C_SCL_DISPLAY (P6_4)
#define PIN_I2C_SDA_DISPLAY (P6_5)

/*Joystick sensitivity*/
#define JOYSTICK_SENSITIVITY	(1U)

// Delay in ms for HID task notify wait
#define TASK_MAX_WAIT  1000



/******************************************************************************
 * Extern variables
 ******************************************************************************/
extern uint16_t buttonStates;

extern int8_t xMotion;
extern int8_t yMotion;
extern int8_t zMotion;
extern int8_t rxMotion;

extern uint8_t home_led_state_change_flag;


enum tlx493d_init_error_flag_states
{
	TLX493D_NO_ERRORS			= 0x00,
	TLX493D_0X35_R2_ERROR		= 0x01,
	TLX493D_0X22_L2_ERROR		= 0x02,
	TLX493D_0X78_JR_ERRORS		= 0x03,
	TLX493D_0X44_JL_ERROR		= 0x04,
};

/*tlx493d_init_error_flag states*/
/*0x00: no init errors; 0x01: 0x6A/0x35  0x02: 0x44/0x22; 0x03: 0xF0/0x78; 0x04: 0x88: */
extern uint8_t tlx493d_init_error_flag;

/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
void gc_initBoard();

void gc_activateGreenLED(float dutyCylce);

void gc_deactivateGreenLED();


#endif /* GAMECONTROLLER_BOARDCONTROL_H_ */
