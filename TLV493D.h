

/******************************************************************************
* File Name: tli493d.h
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

#ifndef TLV493D_H_
#define TLV493D_H_

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

#include "TLx493D_inc.h"


/******************************************************************************
 * Macros
 ******************************************************************************/
/*TLV493D iic configurable addresses*/
#define TLV493D_IIC_ADDR_A0		(0x6A)
#define TLV493D_IIC_ADDR_A1		(0x44)
#define TLV493D_IIC_ADDR_A2		(0xF0)
#define TLV493D_IIC_ADDR_A3		(0x88)

/*TLx493D official registers*/
#define TLV493D_BX_REG			(0x00)
#define TLV493D_BY_REG			(0x01)
#define TLV493D_BZ_REG			(0x02)
#define TLV493D_TEMP_REG		(0x03)
#define TLV493D_BX2_REG			(0x04)
#define TLV493D_TEMP2_REG		(0x05)
#define TLV493D_DIAG_REG		(0x06)
#define TLV493D_XL_REG			(0x07)
#define TLV493D_XH_REG			(0x08)
#define TLV493D_YL_REG			(0x09)
#define TLV493D_YH_REG			(0x0A)
#define TLV493D_ZL_REG			(0x0B)
#define TLV493D_ZH_REG			(0x0C)
#define TLV493D_WU_REG			(0x0D)
#define TLV493D_TMODE_REG		(0x0E)
#define TLV493D_TPHASE_REG		(0x0F)
#define TLV493D_CONFIG_REG		(0x10)
#define TLV493D_MOD1_REG		(0x11)
/*Register 0x12 is reserved*/
#define TLV493D_MOD2_REG		(0x13)
#define TLV493D_CONFIG2_REG		(0x14)
/*Register 0x15 is reserved*/
#define TLV493D_VER_REG			(0x16)

/******************************************************************************
 * Extern variables
 ******************************************************************************/

/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
uint8_t TLV493D_init(TLx493D_t *sensor, cyhal_gpio_t powerPin, uint8_t address);

uint8_t TLV493D_deinit(TLx493D_t *sensor, cyhal_gpio_t powerPin, uint8_t address);

void TLV493D_setShortRangeSensitivity(TLx493D_t *sensor);

void TLV493D_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);

#endif /* TLV493D_H_ */
