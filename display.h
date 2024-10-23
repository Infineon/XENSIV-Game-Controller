

/******************************************************************************
* File Name: display.h
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description:
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
#ifndef DISPLAY_H_
#define DISPLAY_H_


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "mtb_ssd1306.h"
#include "GUI.h"

#include "images/ifx_logo.h"
#include "images/gameControllerIcon.h"
#include "images/gearIcon.h"
#include "images/arrow.h"

#include "PROGBAR.h"


/******************************************************************************
 * Constants
 ******************************************************************************/
typedef enum{
    	DISPLAY_MENU_1,
		DISPLAY_MENU_2,
		DISPLAY_MENU_3,
}display_menu_structure;

/******************************************************************************
 * Externs
 ******************************************************************************/


/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
void display_init(void);

void app_display_task(void *pvParameters);

void display_startupSequence(void);

void display_startupSequence_static(void);

void display_powerDown(void);
void display_showBattery();
void display_updateVbatProgbar(PROGBAR_Handle hProg);
void display_show_powerDown();
void display_showExit(void);
uint8_t display_capsense_isIdle();

#endif /* DISPLAY_H_ */
