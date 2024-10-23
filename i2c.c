/******************************************************************************
* File Name: i2c.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This is the i2c file handle.
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
#include "i2c.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "gameController_boardControl.h"





/*******************************************************************************
* Macros
*******************************************************************************/

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)
#define I2C_2_FREQ              (400000UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
cyhal_i2c_t mI2C;
cyhal_i2c_cfg_t mI2C_cfg;

cyhal_i2c_t mI2C_2;
cyhal_i2c_cfg_t mI2C_cfg_2;

SemaphoreHandle_t i2c_semaphore;


/*******************************************************************************
* Function Definitions
*******************************************************************************/

void i2c_init(void)
{
	cy_rslt_t cy_result;
    printf("I2C master initialization: ");
	mI2C_cfg.is_slave = false;
	mI2C_cfg.address = 0;
	mI2C_cfg.frequencyhal_hz = I2C_FREQ;
	cy_result = cyhal_i2c_init(&mI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	if (cy_result != CY_RSLT_SUCCESS)
	{
		printf("init failed!\r\n");
		CY_ASSERT(0);
	}
	else
	{
		cy_result = cyhal_i2c_configure(&mI2C, &mI2C_cfg);
		if (cy_result != CY_RSLT_SUCCESS)
		{
			printf("config failed!\r\n");
			CY_ASSERT(0);
		}else{
			printf("Success! \r\n");
		}
	}

	i2c_semaphore = xSemaphoreCreateBinary();
	if (i2c_semaphore == NULL)
	{
		printf("Error: Failed to create I2C Semaphore\r\n");
	}else{
		xSemaphoreGive(i2c_semaphore);
	}
}


void i2c_2_init(void)
{
	cy_rslt_t cy_result;
	printf("I2C display master initialization: ");
	mI2C_cfg_2.is_slave = false;
	mI2C_cfg_2.address = 0;
	mI2C_cfg_2.frequencyhal_hz = I2C_2_FREQ;
	cy_result = cyhal_i2c_init(&mI2C_2, PIN_I2C_SDA_DISPLAY, PIN_I2C_SCL_DISPLAY, NULL);
	if (cy_result != CY_RSLT_SUCCESS)
	{
		printf("init failed!\r\n");
		CY_ASSERT(0);
	}
	else
	{
		cy_result = cyhal_i2c_configure(&mI2C_2, &mI2C_cfg_2);
		if (cy_result != CY_RSLT_SUCCESS)
		{
			printf("config failed!\r\n");
			CY_ASSERT(0);
		}else{
			printf("Success! \r\n");
		}
	}
}

