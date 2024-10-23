/*
 * joystick.c
 *
 *  Created on: 21.12.2023
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
#include "joystick.h"
#include "i2c.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"

#include "stdlib.h"

#include "TLx493D_inc.h"
#include "TLV493D.h"
#include "TLE75008.h"

#include "gameController_boardControl.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x6BUL)




/*******************************************************************************
* Global Variables
*******************************************************************************/
uint8_t TLV493D_read_data[6];

uint8_t TLV493D_write_data[2];

TLx493D_t tlv493d_joystick_right;
TLx493D_t tlv493d_joystick_left;

uint8_t R3_pressed;
uint8_t L3_pressed;

volatile uint8_t R3_rumble_flag = 0;
volatile uint8_t L3_rumble_flag = 0;

uint8_t pole_switch_left = false;
uint8_t pole_switch_right = false;

int16_t joystick_right_xOffset = 0;
int16_t joystick_right_yOffset = 0;
int16_t joystick_left_xOffset = 0;
int16_t joystick_left_yOffset = 0;

int16_t joystick_right_xFactor_pos = 400;
int16_t joystick_right_xFactor_neg = 400;
int16_t joystick_right_yFactor_pos = 400;
int16_t joystick_right_yFactor_neg = 400;
int16_t joystick_left_xFactor_pos = 400;
int16_t joystick_left_xFactor_neg = 400;
int16_t joystick_left_yFactor_pos = 400;
int16_t joystick_left_yFactor_neg = 400;
double joystickMaxZone = 127/0.93;

uint8_t joystick_calibrate_flag = 0;

uint8_t joystickLeftOldMagn = 0;
uint8_t joystickRightOldMagn = 0;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

void joysticks_init()
{
	uint8_t error_flag = 0;
	uint8_t init_retry = 0;
	printf("Joystick left ");
	error_flag = TLV493D_init(&tlv493d_joystick_left, PIN_POWER_JOYSTICK_LEFT, TLV493D_IIC_ADDR_A3);
	while(error_flag&&init_retry<10)
	{
		TLV493D_deinit(&tlv493d_joystick_left, PIN_POWER_JOYSTICK_LEFT, TLV493D_IIC_ADDR_A3);
		cyhal_system_delay_ms(5);
		printf("Retry Joystick left ");
		error_flag = TLV493D_init(&tlv493d_joystick_left, PIN_POWER_JOYSTICK_LEFT, TLV493D_IIC_ADDR_A3);
		init_retry++;
	}
	init_retry = 0;
	if(error_flag)
	{
		printf("Joystick left can not be initialized!\r\n");
		TLV493D_deinit(&tlv493d_joystick_left, PIN_POWER_JOYSTICK_LEFT, TLV493D_IIC_ADDR_A3);
		tlx493d_init_error_flag |= TLX493D_0X44_JL_ERROR;
		error_flag = 0;
	}
	joystick_calibrate(&tlv493d_joystick_left);
	printf("Magnet offset: x=%i, y=%i\r\n", joystick_left_xOffset, joystick_left_yOffset);

	cyhal_system_delay_ms(3);

	printf("Joystick right ");
	error_flag = TLV493D_init(&tlv493d_joystick_right, PIN_POWER_JOYSTICK_RIGHT, TLV493D_IIC_ADDR_A2);
	while(error_flag&&init_retry<10)
	{
		TLV493D_deinit(&tlv493d_joystick_right, PIN_POWER_JOYSTICK_RIGHT, TLV493D_IIC_ADDR_A2);
		cyhal_system_delay_ms(5);
		printf("Retry Joystick right ");
		error_flag = TLV493D_init(&tlv493d_joystick_right, PIN_POWER_JOYSTICK_RIGHT, TLV493D_IIC_ADDR_A2);
		init_retry++;
	}
	init_retry = 0;
	if(error_flag)
	{
		printf("Joystick right can not be initialized!\r\n");
		TLV493D_deinit(&tlv493d_joystick_right, PIN_POWER_JOYSTICK_RIGHT, TLV493D_IIC_ADDR_A2);
		tlx493d_init_error_flag |= TLX493D_0X78_JR_ERRORS;
		error_flag = 0;
	}
	joystick_calibrate(&tlv493d_joystick_right);
	printf("Magnet offset: x=%i, y=%i\r\n", joystick_right_xOffset, joystick_right_yOffset);
	//TLV493D_setShortRangeSensitivity(&tlv493d_joystick_right);

	int16_t x, y, z;
	TLV493D_getRawMagneticField(&tlv493d_joystick_left, &x, &y, &z);
	if(z<0) pole_switch_left = 1;
	if(sqrt(z*z)<480) joystickLeftOldMagn = 1;
	TLV493D_getRawMagneticField(&tlv493d_joystick_right, &x, &y, &z);
	if(z<0) pole_switch_right = 1;
	if(sqrt(z*z)<480) joystickRightOldMagn = 1;
	R3_pressed = 0;
	L3_pressed = 0;
}



int8_t *TLV493D_get_reg_values()
{
	cy_rslt_t cy_result;
	for(int i=0; i<6; i++){
		TLV493D_read_data[i] = 0;
	}
	/* Read response packet from the slave */
	cy_result=cyhal_i2c_master_read(&mI2C, 0x35, TLV493D_read_data, 6, 3, true);
	if(cy_result!=CY_RSLT_SUCCESS)
	{
		//printf("TLV493D I2C read error");
	}
	return TLV493D_read_data;
}


double TLV493D_get_val(uint8_t axis)
{

//	TLV493D_get_reg_values();
//	int8_t x = TLV493D_read_data[0];
//	int8_t y = TLV493D_read_data[1];
//	int8_t z = TLV493D_read_data[2];

	int16_t x, y, z;

	TLx493D_t joystick;

	uint8_t joystick_side;

	int16_t xCalib = 0;
	int16_t yCalib = 0;

	typedef enum
	{
		JOYSTICK_LEFT,
		JOYSTICK_RIGHT
	}Joystick;

	switch (axis)
	{
		case TLV493D_XAXIS:
			joystick = tlv493d_joystick_left;
			joystick_side = JOYSTICK_LEFT;
			xCalib = joystick_left_xOffset;
			yCalib = joystick_left_yOffset;
			break;
		case TLV493D_YAXIS:
			joystick = tlv493d_joystick_left;
			joystick_side = JOYSTICK_LEFT;
			xCalib = joystick_left_xOffset;
			yCalib = joystick_left_yOffset;
			break;
		case TLV493D_ZAXIS:
			joystick = tlv493d_joystick_right;
			joystick_side = JOYSTICK_RIGHT;
			xCalib = joystick_right_xOffset;
			yCalib = joystick_right_yOffset;
			break;
		case TLV493D_RXAXIS:
			joystick = tlv493d_joystick_right;
			joystick_side = JOYSTICK_RIGHT;
			xCalib = joystick_right_xOffset;
			yCalib = joystick_right_yOffset;
			break;
		default:
			return 0;
			break;
	}

	TLV493D_getRawMagneticField(&joystick, &x, &y, &z);

	x-=xCalib;
	y-=yCalib;

	/*pole correction: if southpole faces sensor topside -> invert all components to have the same behavior as with the northpole*/
	if((pole_switch_left&&joystick_side==JOYSTICK_LEFT)||(pole_switch_right&&joystick_side==JOYSTICK_RIGHT))
	{
	x = -x;
	y = -y;
	z = -z;
	}

	//x+=33;
	//y-=5;


	//calculate br and theta
	double br = sqrt(x*x+y*y);
	double theta = atan2(br, z);



	/*use this for detecting button press?*/
	int16_t l = sqrt(x*x+y*y+z*z);

	if (joystick_side==JOYSTICK_RIGHT)
	{
		//printf("theta: %f\r\n", theta);
//		printf("x: %i, y: %i, z: %i, l: %i\r\n", x, y, z, l);
	}

	//printf("vector: %i\r\n", l);
	if(l>500)
	{
		switch (joystick_side)
		{
			case JOYSTICK_LEFT:
				if(!L3_rumble_flag)
				{
					L3_rumble_flag = 1;
					rumble_activate(RUMBLE_LEFT, 70);
					cyhal_system_delay_ms(35);
					rumble_deactivate();

				}
				L3_pressed = 1;
				break;
			case JOYSTICK_RIGHT:
				if(!R3_rumble_flag)
				{
					R3_rumble_flag = 1;
					rumble_activate(RUMBLE_RIGHT, 70);
					cyhal_system_delay_ms(35);
					rumble_deactivate();

				}
				R3_pressed = 1;
				break;
		}

	}

	if (l<=450)
	{
		switch (joystick_side)
		{
			case JOYSTICK_LEFT:
				if(L3_rumble_flag==1)
				{
					L3_rumble_flag = 0;
					rumble_activate(RUMBLE_LEFT, 70);
					cyhal_system_delay_ms(20);
					rumble_deactivate();

				}
				L3_pressed = 0;

				break;
			case JOYSTICK_RIGHT:
				if(R3_rumble_flag==1)
				{
					R3_rumble_flag = 0;
					rumble_activate(RUMBLE_RIGHT, 70);
					cyhal_system_delay_ms(20);
					rumble_deactivate();
				}
				R3_pressed = 0;
				break;
		}
	}
	//cyhal_system_delay_ms(100);

	double xVal = x * theta / br;
	double yVal = y * theta / br;

	/*Jakobs joystick*/
	//xVal = xVal*30+3.5;
	//yVal = yVal*30+0.5;

	/*New Joysticks*/
	xVal = xVal*210;
	yVal = yVal*210;


	if (joystick_side==JOYSTICK_RIGHT)
		{
			//printf("TLE values: X=%f, Y=%f\r\n", xVal, yVal);
		}


	switch(axis)
	{
		case(TLV493D_XAXIS):
			return xVal;
		case(TLV493D_YAXIS):
			return yVal;
		case(TLV493D_ZAXIS):
			return xVal;
		case(TLV493D_RXAXIS):
			return yVal;
		default:
			return 0;
	}
}

void joystick_calibrate(TLx493D_t *joystick)
{
	int16_t x=0, y=0, z=0;
	int32_t xAgg=0, yAgg=0, zAgg=0;
	uint8_t iterations=10;
	for(uint8_t i = 0; i<iterations; i++)
	{
		TLV493D_getRawMagneticField(joystick, &x, &y, &z);
		xAgg+=x;
		yAgg+=y;
		zAgg+=z;
	}
	x=xAgg/iterations;
	y=yAgg/iterations;
	z=zAgg/iterations;

	if(joystick==&tlv493d_joystick_left)
	{
		joystick_left_xOffset = x;
		joystick_left_yOffset = y;
	}
	else if(joystick==&tlv493d_joystick_right)
	{
		joystick_right_xOffset = x;
		joystick_right_yOffset = y;
	}
}


bool joystick_get_val(TLx493D_t *joystick, int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState)
{
	int16_t x=0, y=0, z=0;
	int32_t xAgg=0, yAgg=0, zAgg=0;
	uint8_t iterations=10;
	int16_t xCalib = 0, yCalib = 0;


	/*first sensor read to trigger measurement in master controlled mode*/
	TLV493D_getRawMagneticField(joystick, &x, &y, &z);

	if(joystick==&tlv493d_joystick_left)
	{
		xCalib=joystick_left_xOffset;
		yCalib=joystick_left_yOffset;
	}
	else if(joystick==&tlv493d_joystick_right)
	{
		xCalib=joystick_right_xOffset;
		yCalib=joystick_right_yOffset;
	}


	for(uint8_t i = 0; i<iterations; i++)
	{
		TLV493D_getRawMagneticField(joystick, &x, &y, &z);
		xAgg=xAgg+x-xCalib;
		yAgg=yAgg+y-yCalib;
		zAgg+=z;
	}

	x=xAgg/iterations;
	//xAgg-=33;
	y=yAgg/iterations;
	z=zAgg/iterations;


	//printf("xAgg= %i, yAgg= %i, zAgg= %i\r\n", xAgg, yAgg, zAgg);

	/*pole correction: if southpole faces sensor topside -> invert all components to have the same behavior as with the northpole*/
	if(z < 0)
	{
	x = -x;
	y = -y;
	z = -z;
	}

	//calculate br and theta
	double br = sqrt(x*x+y*y);
	double theta = atan2(br, z);
	//printf("theta: %f\r\n", theta);


	/*use vector length for detecting button press*/
	int16_t l = sqrt(x*x+y*y+z*z);


	if(l>500)
	{
		if(joystick==&tlv493d_joystick_left)
		{
			if(!L3_rumble_flag)
			{
				L3_rumble_flag = 1;
				rumble_activate(RUMBLE_LEFT, 70);
				cyhal_system_delay_ms(35);
				rumble_deactivate();
			}
				L3_pressed = 1;
		}
		else if(joystick==&tlv493d_joystick_right)
		{
			if(!R3_rumble_flag)
			{
				R3_rumble_flag = 1;
				rumble_activate(RUMBLE_RIGHT, 70);
				cyhal_system_delay_ms(35);
				rumble_deactivate();
			}
			R3_pressed = 1;
		}
	}
	else if (l<=450)
	{
		if(joystick==&tlv493d_joystick_left)
		{
			if(L3_rumble_flag==1)
			{
				L3_rumble_flag = 0;
				rumble_activate(RUMBLE_LEFT, 70);
				cyhal_system_delay_ms(20);
				rumble_deactivate();

			}
			L3_pressed = 0;
		}
		else if(joystick==&tlv493d_joystick_right)
		{
			if(R3_rumble_flag==1)
			{
				R3_rumble_flag = 0;
				rumble_activate(RUMBLE_RIGHT, 70);
				cyhal_system_delay_ms(20);
				rumble_deactivate();
			}
			R3_pressed = 0;
		}
	}

	double xs = (-x * theta / br)*370;
	double ys = (y * theta / br)*344;

	int16_t xM=(int16_t)xs;
	int16_t yM=(int16_t)ys;

	if(abs(xM)<5) xM = 0;
	if(xM>127) xM = 127;
	if(xM<-127) xM = -127;

	if(abs(yM)<5) yM = 0;
	if(yM>127) yM = 127;
	if(yM<-127) yM = -127;

	*xMotion = (int8_t)(xM);
	*yMotion = (int8_t)(yM);

	//printf("Xmotion: %i, yMotion: %i\r\n", xM, yM);

	return true;

}


bool joystick_get_val_polarToTiltFitted(TLx493D_t *joystick, int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState)
{
	int16_t x=0, y=0, z=0;
	int32_t xAgg=0, yAgg=0, zAgg=0;
	uint8_t iterations=10;
	int16_t xCalib = 0, yCalib = 0;


	/*first sensor read to trigger measurement in master controlled mode*/
	TLV493D_getRawMagneticField(joystick, &x, &y, &z);

//	if(joystick==&tlv493d_joystick_left)
//	{
//		printf("LEFT: x: %i, y: %i, z: %i ", x, y, z);
//	}
//
//	if(joystick==&tlv493d_joystick_right)
//	{
//		printf("   RIGHT: x: %i, y: %i, z: %i \r\n", x, y, z);
//	}
	if(joystick==&tlv493d_joystick_left)
	{
		xCalib=joystick_left_xOffset;
		yCalib=joystick_left_yOffset;
	}
	else if(joystick==&tlv493d_joystick_right)
	{
		xCalib=joystick_right_xOffset;
		yCalib=joystick_right_yOffset;
	}


	for(uint8_t i = 0; i<iterations; i++)
	{
		TLV493D_getRawMagneticField(joystick, &x, &y, &z);
		xAgg=xAgg+x-xCalib;
		yAgg=yAgg+y-yCalib;
//		xAgg=xAgg+x;
//		yAgg=yAgg+y;
		zAgg+=z;
	}

	x=xAgg/iterations;
	//xAgg-=33;
	y=yAgg/iterations;
	z=zAgg/iterations;


	//printf("xAgg= %i, yAgg= %i, zAgg= %i\r\n", xAgg, yAgg, zAgg);

	/*pole correction: if southpole faces sensor topside -> invert all components to have the same behavior as with the northpole*/
	if(z < 0)
	{
	x = -x;
	y = -y;
	z = -z;
	}

	//calculate br and theta
	double br = sqrt(x*x+y*y+z*z);
	double phi = atan2(y, x);
	double theta = acos(z/br);
	double theta_fitted = theta;
	double ys = cos(phi)*theta_fitted;
	double xs = sin(phi)*theta_fitted;


	if(joystick==&tlv493d_joystick_left)
	{
		//xs = -xs;			//HID-protocol sees z axis reversed, sensor/magnet is same orientation on both joysticks
		if(xs>0)
		{
			xs = xs*joystick_left_xFactor_pos;
			if(xs>(joystickMaxZone))
			{
				joystick_left_xFactor_pos*=joystickMaxZone/xs;
			}
		}
		else if(xs<0)
		{
			xs = xs*joystick_left_xFactor_neg;
			if(xs<(-joystickMaxZone))
			{
				joystick_left_xFactor_neg*=-joystickMaxZone/xs;
			}
		}


		if(ys>=0)
		{
			ys = ys*joystick_left_yFactor_pos;
			if(ys>(joystickMaxZone))
			{
				joystick_left_yFactor_pos*=joystickMaxZone/ys;
			}
		}
		else if(ys<0)
		{
			ys = ys*joystick_left_yFactor_neg;
			if(ys<(-joystickMaxZone))
			{
				joystick_left_yFactor_neg*=-joystickMaxZone/ys;
			}
		}


	}
	else if(joystick==&tlv493d_joystick_right)
	{
		//xs = -xs;			//HID-protocol sees z axis reversed, sensor/magnet is same orientation on both joysticks
		if(xs>0)
		{
			xs = xs*joystick_right_xFactor_pos;
			if(xs>(joystickMaxZone))
			{
				joystick_right_xFactor_pos*=joystickMaxZone/xs;
			}
		}
		else if(xs<0)
		{
			xs = xs*joystick_right_xFactor_neg;
			if(xs<(-joystickMaxZone))
			{
				joystick_right_xFactor_neg*=-joystickMaxZone/xs;
			}
		}
		if(ys>0)
		{
			ys = ys*joystick_right_yFactor_pos;
			if(ys>(joystickMaxZone))
			{
				joystick_right_yFactor_pos*=joystickMaxZone/ys;
			}
		}
		else if(ys<0)
		{
			ys = ys*joystick_right_yFactor_neg;
			if(ys<(-joystickMaxZone))
			{
				joystick_right_yFactor_neg*=-joystickMaxZone/ys;
			}
		}

	}

	int16_t xM=(int16_t)xs;
	int16_t yM=(int16_t)ys;
	xs = ys = 0;

	//printf("Xmotion: %i, yMotion: %i\r\n", xM, yM);

	if(abs(xM)<6) xM = 0;
	if(xM>127) xM = 127;
	if(xM<-127) xM = -127;

	if(abs(yM)<6) yM = 0;
	if(yM>127) yM = 127;
	if(yM<-127) yM = -127;

	*xMotion = (int8_t)(xM);
	*yMotion = (int8_t)(yM);


	/*use vector length for detecting button press*/
	int16_t l = sqrt(x*x+y*y+z*z);
	int16_t vibrVec_on = 650;
	int16_t vibrVec_off = 600;

	if(joystickLeftOldMagn && joystick==&tlv493d_joystick_left)
	{
		vibrVec_on = 530;
		vibrVec_off = 480;
	}

	if(joystickRightOldMagn && joystick==&tlv493d_joystick_right)
	{
		vibrVec_on = 530;
		vibrVec_off = 480;
	}



	if(l>vibrVec_on)
	{
		if(joystick==&tlv493d_joystick_left)
		{

			if(!L3_rumble_flag)
			{
				L3_rumble_flag = 1;
				rumble_activate(RUMBLE_LEFT, 70);
				cyhal_system_delay_ms(35);
				rumble_deactivate();
			}
				L3_pressed = 1;
		}
		else if(joystick==&tlv493d_joystick_right)
		{
			if(!R3_rumble_flag)
			{
				R3_rumble_flag = 1;
				rumble_activate(RUMBLE_RIGHT, 70);
				cyhal_system_delay_ms(35);
				rumble_deactivate();
			}
			R3_pressed = 1;
		}
	}
	else if (l<=vibrVec_off)
	{
		if(joystick==&tlv493d_joystick_left)
		{
			if(L3_rumble_flag==1)
			{
				L3_rumble_flag = 0;
				rumble_activate(RUMBLE_LEFT, 70);
				cyhal_system_delay_ms(20);
				rumble_deactivate();

			}
			L3_pressed = 0;
		}
		else if(joystick==&tlv493d_joystick_right)
		{
			if(R3_rumble_flag==1)
			{
				R3_rumble_flag = 0;
				rumble_activate(RUMBLE_RIGHT, 70);
				cyhal_system_delay_ms(20);
				rumble_deactivate();
			}
			R3_pressed = 0;
		}
	}



	//printf("Xmotion: %i, yMotion: %i\r\n", xM, yM);

	return true;

}


double TLV5593_right_get_val(int8_t *xMotion, int8_t *yMotion, uint8_t *buttonState)
{
	return 0;
}
