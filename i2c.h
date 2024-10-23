/******************************************************************************
* File Name: i2c.h
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
#ifndef I2C_H_
#define I2C_H_


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"


/******************************************************************************
 * Constants
 ******************************************************************************/

/******************************************************************************
 * Extern variables
 ******************************************************************************/
extern cyhal_i2c_t mI2C;

extern SemaphoreHandle_t i2c_semaphore;


/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
void i2c_init(void);

void i2c_2_init(void);


#endif /* DISPLAY_H_ */
