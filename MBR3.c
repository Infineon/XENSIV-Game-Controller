/******************************************************************************
* File Name: MBR3.c
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "MBR3.h"
#include "i2c.h"
#include "gameController_boardControl.h"
#include "TLE75008.h"
#include "WM.h"

/*******************************************************************************
* Macros
*******************************************************************************/



/*******************************************************************************
* Global Variables
*******************************************************************************/
uint8_t buffer[PACKET_SIZE];

unsigned char dummyData[2] = {
		0x00, 0x00
};

unsigned char configData[129] = {
		0xFFu, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
		0xFCu, 0x03u, 0x0Fu, 0x00u, 0x80u, 0xC8u, 0xC8u, 0xC8u,
		0xC8u, 0xC8u, 0xC8u, 0xC8u, 0x1Fu, 0x1Fu, 0x7Fu, 0x7Fu,
		0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x01u, 0x00u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x80u,
		0x06u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x1Eu, 0x1Eu, 0x00u,
		0x00u, 0x1Eu, 0x1Eu, 0x00u, 0x00u, 0x1Bu, 0x01u, 0x01u,
		0x05u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x0Fu, 0x0Fu,
		0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x50u,
		0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
		0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xAFu, 0x10u
};

unsigned char saveCmd[2] = {
    		CTRL_CMD, SAVE_CHECK_CRC
    };

unsigned char swResetCmd[2] = {
    		CTRL_CMD, SW_RESET
    };

uint8_t pressedButton = 0;
uint8_t buttonReg = 0;

uint8_t homePressed = 0;
uint8_t controlActivated = 0;
uint8_t controlFlag = 0;
uint8_t Capsense_presence = 1;
uint8_t Capsense_absence_counter = 0;
uint8_t Capsense_presence_activated = 0;

uint8_t menu_item_id_selected = 0;
uint8_t menu_item_id_entered = ITEM_ID_MAINMENU;

uint8_t menu_enter = 0;
uint8_t menu_enter_flag = 0;


mbr3_button_rumble_state_t button0_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button1_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button2_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button3_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button8_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button12_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button13_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button14_state =  	BUTTON_STATE_INACTIVE;
mbr3_button_rumble_state_t button15_state =  	BUTTON_STATE_INACTIVE;

/*******************************************************************************
* Function Definitions
*******************************************************************************/


void MBR3_init(void)
{
	/*Dummy writes to wake up MBR3 via i2c*/
	cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, dummyData, 2, 0, true);
    cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, dummyData, 2, 0, true);

    uint8_t uploadCounter = 0;

    printf("MBR3 configuration at 0x%02x: ", MBR3_IIC_ADDR);
    while (uploadCounter<3 && CY_RSLT_SUCCESS != cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, configData, 129, 10, true))
	{
    	uploadCounter++;
    	if(uploadCounter>=2)
    	{
    		printf("upload failed!\r\n");
    	}
	}

    if (uploadCounter<=2 && CY_RSLT_SUCCESS == cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, saveCmd, 2, 10, true))
	{
		printf("Success! \r\n");
	}
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, swResetCmd, 2, 10, true))
	{
	}

}

uint16_t MBR3_getButtonStates()
{
	uint8_t readValues[2];
	unsigned char readCmd[1] = {
			PROX_STAT
	};

	readCmd[0] = BUTTON_STATUS;
	if (CY_RSLT_SUCCESS == cyhal_i2c_master_write(&mI2C, MBR3_IIC_ADDR, readCmd, 1, 0, true))
		{

		}
	cyhal_i2c_master_read(&mI2C, MBR3_IIC_ADDR, readValues, 2, 3, true);

	//printf("pressed: %i, %i\r\n", readValues[0], readValues[1]);

	buttonStates = 0;
	uint8_t states = readValues[0];
	buttonRumble((states&=0x02), &button1_state);
	buttonStates |= states;	//Button X
	states = readValues[0];
	buttonRumble((states&=0x04), &button0_state);
	buttonStates |= states>>2;	//Button F
	states = readValues[0];
	buttonRumble((states&=0x08), &button2_state);
	buttonStates |= states>>1;	//Button I
	states = readValues[0];
	buttonRumble((states&=0x10), &button3_state);
	buttonStates |= states>>1;	//Button T
	/*Following buttons have double use: BLE HID + Display menu*/
	if(!controlActivated) //BLE HID
	{
		states = readValues[0];
		buttonRumble((states&=0x20), &button12_state);
		buttonStates |= states<<7;	//Button Up
		states = readValues[0];
		buttonRumble((states&=0x40), &button14_state);
		buttonStates |= states<<8;	//Button Left
		states = readValues[0];
		buttonRumble((states&=0x80), &button13_state);
		buttonStates |= states<<6;	//Button Down
		states = readValues[1];
		buttonRumble((states&=0x01), &button15_state);
		buttonStates |= states<<15;	//Button Right
	}
	else if(controlActivated) //Display control
	{
		states = readValues[0];
		if((states&=0x20)&&!button12_state) GUI_SendKeyMsg(GUI_KEY_UP, 1);	//Button Up
		buttonRumble((states), &button12_state);
		states = readValues[0];
		if((states&=0x40)&&!button14_state) menu_item_id_entered = ITEM_ID_MAINMENU;	//Button Left
		buttonRumble(states, &button14_state);
		states = readValues[0];
		if((states&=0x80)&&!button13_state) GUI_SendKeyMsg(GUI_KEY_DOWN, 1);	//Button Down
		buttonRumble((states), &button13_state);
		states = readValues[1];
		if((states&=0x01)&&!button15_state){									//Button Right
			menu_item_id_entered = menu_item_id_selected;
			if(!menu_enter_flag) menu_enter = 1;
		}else
		{
			menu_enter_flag = 0;
		}
		buttonRumble(states, &button15_state);
	}

	states = readValues[1];
	buttonRumble((states&=0x02), &button8_state);
	buttonStates |= states<<7;	//Button HOME

	states = readValues[1];
	if(states&=0x02) homePressed = 1;
	if(states&&controlActivated == 0&&controlFlag==0)
	{
		printf("control activated\r\n");
		controlActivated = 1;
		controlFlag = 1;
	}
	else if((states)&&controlActivated == 1&&controlFlag==0)
	{
		printf("control deactivated\r\n");
		controlActivated = 0;
		controlFlag = 1;
	}else if(states == 0)
	{
		controlFlag = 0;
	}

	states = readValues[0];
	if(states&=0x1E)
	{
		TLE75008_LED_activate(LED_BUTTONS_RIGHT);
	}
	else
	{
		TLE75008_LED_deactivate(LED_BUTTONS_RIGHT);
	}
	states = readValues[0];
	uint8_t states2 = readValues[1];
	if((states&=0xE0)!=0||(states2&=0x01)!=0)
	{
		TLE75008_LED_activate(LED_BUTTONS_LEFT);
	}
	else
	{
		TLE75008_LED_deactivate(LED_BUTTONS_LEFT);
	}

	/*Capsense presence detection*/
	states = readValues[0];
	states&=0x01;
	if(states == 0 && Capsense_presence_activated == 1)
	{
		Capsense_absence_counter++;
		if(Capsense_absence_counter >70)
		{
			if(Capsense_presence)
			{
				home_led_state_change_flag = 1;
				printf("change to sleep\r\n");
			}
			else if(!Capsense_presence)
			{
				Capsense_absence_counter = 0;
			}
			Capsense_presence = 0;
		}
	}
	else
	{
		if(!Capsense_presence)
		{
			home_led_state_change_flag = 1;
		}
		Capsense_presence = 1;
		Capsense_absence_counter = 0;
	}


	return buttonStates;

}

uint8_t getHomePressedState(void)
{
	return homePressed;
}

void resetHomePressedState(void)
{
	homePressed = 0;
}


void buttonRumble(uint8_t buttonState, mbr3_button_rumble_state_t *rumbleState)
{
	if(buttonState!=0&&*rumbleState!=BUTTON_STATE_ACTIVE_RUMBLED)
	{
		buttonPress_haptic_feedback();
		*rumbleState = BUTTON_STATE_ACTIVE_RUMBLED;
	}
	else if(buttonState==0)
	{
		*rumbleState = BUTTON_STATE_INACTIVE;
	}
}


