

/******************************************************************************
* File Name: joystick.h
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

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "TLx493D_inc.h"
#include "TLV493D.h"


/*******************************************************************************
* Macros
*******************************************************************************/

#define TLV493D_XAXIS (0U)
#define TLV493D_YAXIS (1U)
#define TLV493D_ZAXIS (2U)
#define TLV493D_RXAXIS (3U)


/*******************************************************************************
* External variables
*******************************************************************************/

extern uint8_t R3_pressed;
extern uint8_t L3_pressed;

extern TLx493D_t tlv493d_joystick_right;
extern TLx493D_t tlv493d_joystick_left;

extern int16_t joystick_right_xOffset;
extern int16_t joystick_right_yOffset;
extern int16_t joystick_left_xOffset;
extern int16_t joystick_left_yOffset;

extern uint8_t joystick_calibrate_flag;
/*******************************************************************************
* Funciton Declarations
*******************************************************************************/
void joysticks_init();

int8_t *TLV493D_get_reg_values();

double TLV493D_get_val(uint8_t axis);

bool joystick_get_val(TLx493D_t *joystick, int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState);

bool joystick_get_val_polarToTiltFitted(TLx493D_t *joystick, int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState);

void joystick_calibrate(TLx493D_t *joystick);

double TLV5593_left_get_val(int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState);

double TLV5593_right_get_val(int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState);

#endif /* JOYSTICK_H_ */
