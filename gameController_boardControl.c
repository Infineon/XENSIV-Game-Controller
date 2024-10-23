/******************************************************************************
* File Name: gameController_boardControl.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This file initializes general, board specific hardware.
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
*
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "gameController_boardControl.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

uint16_t buttonStates;

cyhal_pwm_t pwm_led_control_red;
cyhal_pwm_t pwm_led_control_green;
cyhal_pwm_t pwm_led_control_blue;

int8_t xMotion = 0;
int8_t yMotion = 0;
int8_t zMotion = 0;
int8_t rxMotion = 0;

uint8_t home_led_state_change_flag = 0;

uint8_t tlx493d_init_error_flag = TLX493D_NO_ERRORS;


/*******************************************************************************
* Function Definitions
*******************************************************************************/
void gc_initBoard()
{
	/*Init power pin of TLV493D sensors */
	cyhal_gpio_init(PIN_POWER_JOYSTICK_LEFT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_init(PIN_POWER_JOYSTICK_RIGHT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_init(PIN_POWER_R2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_init(PIN_POWER_L2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

	/*RGB LED pins init*/
/*TODO: Map RGB LED to SPIDER+
	cyhal_gpio_init(PIN_LED_HOME_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	cyhal_gpio_init(PIN_LED_HOME_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
	cyhal_gpio_init(PIN_LED_HOME_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

	cyhal_pwm_init(&pwm_led_control_red, PIN_LED_HOME_RED, NULL);
	cyhal_pwm_init(&pwm_led_control_green, PIN_LED_HOME_GREEN, NULL);
	cyhal_pwm_init(&pwm_led_control_blue, PIN_LED_HOME_BLUE, NULL);
*/
	buttonStates = 0;
}


void gc_activateGreenLED(float dutyCylce)
{
	cyhal_pwm_set_duty_cycle(&pwm_led_control_green, dutyCylce, 1000);
	cyhal_pwm_start(&pwm_led_control_green);
}

void gc_deactivateGreenLED()
{
	cyhal_pwm_stop(&pwm_led_control_green);
}
