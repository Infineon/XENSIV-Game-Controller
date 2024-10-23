/******************************************************************************
* File Name: gc_GameController.h
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
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

#ifndef GC_GAMECONTROLLER_H_
#define GC_GAMECONTROLLER_H_



/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "gc_bt_hid.h"
#include "gameController_boardControl.h"


/******************************************************************************
 * Macros
 ******************************************************************************/



/******************************************************************************
 * Extern variables
 ******************************************************************************/



/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
void app_gameController_task(void *pvParameters);



#endif /* GC_GAMECONTROLLER_H_ */
