/******************************************************************************
* File Name: display.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This file is for GUI and display control
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
#include "display.h"
#include "i2c.h"
#include <FreeRTOS.h>

#include "../mtb_shared/emwin/release-v6.32.0/Tool/WindowDLG.c"
#include "gameController_boardControl.h"
#include "adc.h"
#include "MBR3.h"
#include "TLE75008.h"
#include "joystick.h"
#include "gc_bt_management.h"

#include "GUI.h"
#include "LISTBOX.h"
#include "FRAMEWIN.h"
#include "PROGBAR.h"
#include "WM.h"
#include <stddef.h>

#include "gc_bt_bonding.h"
#include "wiced_bt_stack.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gatt_db.h"

#include "GUIDRV_SPage.h"



/*******************************************************************************
* Macros
*******************************************************************************/
#define SCREEN_WIDTH	128
#define SCREEN_HEIGHT	64

/*******************************************************************************
* Global Variables
*******************************************************************************/

cyhal_i2c_t mI2C_display;
cyhal_i2c_cfg_t mI2C_cfg_display;


 display_menu_structure display_main_menu;

 char* display_menu_names[]= {
		 "Show info",
		 "Rumble strength",
		 "Turn off display",
		 "Power off"
 };

uint8_t display_joystick_centerX 		= 28;
uint8_t display_joystick_centerY 		= 24;
uint8_t display_joystick_drawR	 		= 24;
uint8_t display_joystick_drawR_calib 	= 14;
uint8_t display_joystick_ratio	 		= 7;
uint8_t display_joystick_curserUpFlag 	= 0;

int8_t disp_xMotion = 0;
int8_t disp_yMotion = 0;
int8_t disp_zMotion = 0;
int8_t disp_rxMotion = 0;

int8_t disp_xMotion_norm = 0;
int8_t disp_yMotion_norm = 0;
int8_t disp_zMotion_norm = 0;
int8_t disp_rxMotion_norm = 0;

int8_t disp_xMotion_norm_old = 0;
int8_t disp_yMotion_norm_old = 0;
int8_t disp_zMotion_norm_old = 0;
int8_t disp_rxMotion_norm_old = 0;

uint32_t VbatProgbarCounter = 0;

uint8_t R3_pressed_flag = 0;
uint8_t L3_pressed_flag = 0;

uint8_t show_powerdown_flag = 1;

double zrxVecLen;
double xyVecLen;


/*******************************************************************************
* Function Definitions
*******************************************************************************/


extern WM_HWIN CreateWindow(void);

void display_init(void)
{
    cy_rslt_t cy_result;

	mI2C_cfg_display.is_slave = false;
	mI2C_cfg_display.address = 0;
	mI2C_cfg_display.frequencyhal_hz = 1000000UL;

	cy_result = cyhal_i2c_init(&mI2C_display, PIN_I2C_SDA_DISPLAY, PIN_I2C_SCL_DISPLAY, NULL);
	if (cy_result != CY_RSLT_SUCCESS)
	{
		printf("Display i2c init failed!\r\n");
		CY_ASSERT(0);
	}
	else
	{
		cy_result = cyhal_i2c_configure(&mI2C_display, &mI2C_cfg_display);
		if (cy_result != CY_RSLT_SUCCESS)
		{
			printf("Display i2c config failed!\r\n");
			CY_ASSERT(0);
		}else{
			printf("Display i2c Success! \r\n");
		}
	}

    //printf("Display initialization: ");
    cy_result = mtb_ssd1306_init_i2c(&mI2C_display);
    if(cy_result!=CY_RSLT_SUCCESS){
    	printf("Display initialization: SSD1306 init failed\r\n");
    }
    LCD_SetSize(128, 64);

  	GUI_Init();
    GUI_Clear();
   	GUI_SetOrientation(GUI_MIRROR_X|GUI_MIRROR_Y);
   	printf("Display initialization: Success!\r\n");
}

void display_startupSequence(void)
{
   	/*Startup sequence*/
	GUI_SetColor(GUI_WHITE);
   	GUI_DrawBitmap(&bmifx_logo, 0, 5);
   	for(uint8_t i=0; i<120; i++)
   	{
   		if(controlActivated) return;
   		GUI_Delay(10);
   	}
   	GUI_Clear();
   	GUI_SetFont(&GUI_Font24B_1);
   	GUI_DispStringHCenterAt("XENSIV", 57, 2);
   	GUI_SetFont(&GUI_Font13_1);
   	GUI_DispStringHCenterAt("TM", 104, 0);
   	GUI_DispStringHCenterAt("Game Controller", 64, 25);
   	GUI_DrawBitmap(&bmgameControllerIcon, 49, 36);
   	for(uint8_t i=0; i<100; i++)
   	{
   		if(controlActivated) return;
   		GUI_Delay(10);
   	}
   	/*Menu Sequence*/
   	GUI_Clear();
   	GUI_SetFont(&GUI_Font13_1);
   	GUI_DispStringHCenterAt("Press", 64, 1);
   	GUI_DrawBitmap(&bmgear, 53, 20);
   	GUI_DrawCircle(64, 32, 16);
   	GUI_DispStringHCenterAt("to enter menu", 64, 50);
   	for(uint16_t i=0; i<400; i++)
   	{
   		if(controlActivated) return;
   		GUI_Delay(10);
   	}
   	//GUI_Delay(2000);
   	GUI_Clear();
}


void display_startupSequence_static(void)
{
   	/*Startup sequence*/
   	GUI_DrawBitmap(&bmifx_logo, 0, 5);
   	GUI_Delay(500);
   	GUI_Clear();
   	GUI_DispStringAt("", 0, 20);
	GUI_SetFont(&GUI_Font20B_1);
	GUI_DispString("Welcome back");
   	GUI_Delay(800);
   	GUI_Clear();
   	GUI_SetFont(&GUI_Font24B_1);
   	GUI_DispStringHCenterAt("XENSIV", 57, 2);
   	GUI_SetFont(&GUI_Font13_1);
   	GUI_DispStringHCenterAt("TM", 104, 0);
   	GUI_DispStringHCenterAt("Game Controller", 64, 25);
   	GUI_DrawBitmap(&bmgameControllerIcon, 49, 36);
   	GUI_Delay(800);
   	GUI_Clear();
}



void display_showBattery()
{
	//CreateWindow();
	int32_t vBat = 0;
	for(uint8_t i = 0; i<100; i++)
	{
		vBat = vBat + get_Vbat();
	}
	vBat = vBat/100;
	float vBatFloat = (float) vBat;
	vBatFloat = vBatFloat/1000;
	uint8_t SOC = get_SOCBat(vBat);

	if(vBatFloat>=4.21||vBatFloat==0)
	{
		vBatFloat = 4.22;
		SOC = 100;
	}

	GUI_DispStringAt("", 0, 0);
	GUI_SetFont(&GUI_Font24B_1);
	GUI_DispFloat(vBatFloat, 4);
	GUI_DispString("V (");
	if(SOC == 100)
	{
		GUI_DispDec(SOC, 3);
		GUI_DispString("%)");
	}
	else
	{
		GUI_DispString("  ");
		GUI_DispDec(SOC, 2);
		GUI_DispString("%)");
	}
	GUI_DrawBitmap(&bmgameControllerIcon, 49, 36);
	GUI_Delay(10);
}


void display_updateVbatProgbar(PROGBAR_Handle hProg)
{
	int32_t vBat = 0;
	for(uint8_t i = 0; i<10; i++)
	{
		int32_t vBatMeas = get_Vbat();
		if(vBatMeas == 0) vBatMeas = 4220;
		vBat = vBat + vBatMeas;
	}
	vBat = vBat/10;
	float vBatFloat = (float) vBat;

	uint8_t SOC = get_SOCBat(vBat);
	//PROGBAR_SetValue(hProg, SOC);
	PROGBAR_SetValue(hProg, SOC);

}

void display_show_powerDown()
{
	GUI_Clear();
	GUI_DispStringAt("                ", 0, 0);
	GUI_SetFont(&GUI_Font16B_1);
	GUI_DispStringAt("Entering idle state...", 0, 0);
	GUI_DispStringAt("Power Down: ", 0, 20);
	int32_t idleTime = 4;
	GUI_DispDecAt(idleTime, 90, 20, 1);
	GUI_DispString("s");
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	idleTime--;
	GUI_DispDecAt(idleTime, 90, 20, 1);
	GUI_DispString("s");
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	idleTime--;
	GUI_DispDecAt(idleTime, 90, 20, 1);
	GUI_DispString("s");
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	idleTime--;
	GUI_DispDecAt(idleTime, 90, 20, 1);
	GUI_DispString("s");
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	if(display_capsense_isIdle()) return;
	GUI_Delay(500);
	if(display_capsense_isIdle()) return;
	GUI_Clear();
}

uint8_t display_capsense_isIdle()
{
	if(Capsense_presence)
	{
		GUI_Clear();
		home_led_state_change_flag = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

void display_powerDown(void)
{
	return;
}


void display_showExit(void)
{
	GUI_SetFont(&GUI_Font8_1);
	GUI_DrawBitmap(&bmarrowLeft, 2, 54);
	GUI_DispStringAt("Exit", 13, 55);
}

uint8_t exitFlag = 0;
TickType_t xLastWakeTime;



void app_display_task(void *pvParameters)
{

    uint32_t ulNotifiedValue;
	printf("display task is started \r\n");

	cyhal_system_delay_ms(300);

	display_init();
	display_startupSequence();

	WM_HWIN wm_handle = CreateWindow();
	PROGBAR_Handle hProg = WM_GetDialogItem(wm_handle, ID_PROGBAR_0);
	LISTBOX_Handle hList = WM_GetDialogItem(wm_handle, ID_LISTBOX_0);

	PROGBAR_SKINFLEX_PROPS pProps;
	PROGBAR_GetSkinFlexProps(&pProps, 0);
	pProps.ColorText = GUI_BLACK; // @suppress("Field cannot be resolved")
	pProps.aColorLowerL[0] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorLowerL[1] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorUpperL[0] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorUpperL[1] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorLowerR[0] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorLowerR[1] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorUpperR[0] = GUI_WHITE; // @suppress("Field cannot be resolved")
	pProps.aColorUpperR[1] = GUI_WHITE; // @suppress("Field cannot be resolved")
	PROGBAR_SetSkinFlexProps(&pProps, 0);


	int currentMenuSel = 0;

	while(1)
	{

		if(controlActivated)
		{
			if(WM_IsWindow(wm_handle)&&(menu_item_id_entered!=ITEM_ID_MAINMENU)){
				WM_DeleteWindow(wm_handle);
				GUI_Clear();
				currentMenuSel = menu_item_id_entered;
			}

			switch(menu_item_id_entered){
			case ITEM_ID_INFORMATION:
				display_showExit();
				GUI_SetFont(&GUI_Font8_1);

				GUI_DispStringAt("Status: ", 0, 0);
				switch(app_bt_adv_conn_state){
				case APP_BT_ADV_OFF_CONN_OFF:
					if(ble_boot_status) GUI_DispString("No device found");
					if(!ble_boot_status) GUI_DispString("Booting BLE ...");
					break;
				case APP_BT_ADV_ON_CONN_OFF:
					GUI_DispString("Advertising ...              ");
					break;
				case APP_BT_ADV_OFF_CONN_ON:
					GUI_DispString("Connected                    ");
					break;
				}

				char str[18];
				uint8_t bdadr[6];
				for(uint8_t i = 0; i<6; i++)
				{
					bdadr[i] = 	ble_conn_adr[i];
				}
				sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", bdadr[0],bdadr[1],bdadr[2],bdadr[3],bdadr[4],bdadr[5]);
				GUI_DispStringAt("Remote: ", 0, 12);
				if(ble_conn_id==0)
				{
					GUI_DispString("No connection          ");
				}
				else
				{
					GUI_DispString(str);
				}
				for(uint8_t i = 0; i<6; i++)
				{
					bdadr[i] = 	cy_bt_device_address[i];
				}
				sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", bdadr[0],bdadr[1],bdadr[2],bdadr[3],bdadr[4],bdadr[5]);
				GUI_DispStringAt("Local: ", 0, 24);
				GUI_DispString(str);

				GUI_DispStringAt("Vibration: ", 0, 36);
				if(rumble_active) GUI_DispString("activated");
				if(!rumble_active) GUI_DispString("deactivated");

				break;
			case ITEM_ID_BATTERY:
				display_showExit();
				GUI_SetColor(GUI_WHITE);
				display_showBattery();

				break;
			case ITEM_ID_JOYSTICKS:
				if(!display_joystick_curserUpFlag)
				{
					GUI_Clear();
					GUI_DrawCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					display_joystick_curserUpFlag = 1;
				}
//				GUI_DrawCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
//				GUI_DrawCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);

				//GUI_CURSOR_SetPosition(display_joystick_centerX+disp_xMotion/display_joystick_ratio, display_joystick_centerY+disp_yMotion/display_joystick_ratio);

				disp_xMotion = xMotion;
				disp_yMotion = yMotion;
				xyVecLen = sqrt(disp_xMotion*disp_xMotion+disp_yMotion*disp_yMotion)/127;
				if(xyVecLen>1)
				{
					disp_xMotion = disp_xMotion/xyVecLen;
					disp_yMotion = disp_yMotion/xyVecLen;
				}
				disp_xMotion_norm = display_joystick_centerX+disp_xMotion/display_joystick_ratio;
				disp_yMotion_norm = display_joystick_centerY+disp_yMotion/display_joystick_ratio;

				disp_zMotion = zMotion;
				disp_rxMotion = rxMotion;
				zrxVecLen = sqrt(disp_zMotion*disp_zMotion+disp_rxMotion*disp_rxMotion)/127;
				if(zrxVecLen>1)
				{
					disp_zMotion = disp_zMotion/zrxVecLen;
					disp_rxMotion = disp_rxMotion/zrxVecLen;
				}
				disp_zMotion_norm = 127-display_joystick_centerX+disp_zMotion/display_joystick_ratio;
				disp_rxMotion_norm = display_joystick_centerY+disp_rxMotion/display_joystick_ratio;

				if(disp_xMotion_norm!=disp_xMotion_norm_old||disp_yMotion_norm!=disp_yMotion_norm_old)
				{
					GUI_SetColor(GUI_BLACK);
					GUI_DrawCircle(disp_xMotion_norm_old, disp_yMotion_norm_old, 5);
					GUI_SetColor(GUI_WHITE);
					GUI_DrawCircle(disp_xMotion_norm, disp_yMotion_norm, 5);
				}

				if(disp_zMotion_norm!=disp_zMotion_norm_old||disp_rxMotion_norm!=disp_rxMotion_norm_old)
				{
					GUI_SetColor(GUI_BLACK);
					GUI_DrawCircle(disp_zMotion_norm_old, disp_rxMotion_norm_old, 5);
					GUI_SetColor(GUI_WHITE);
					GUI_DrawCircle(disp_zMotion_norm, disp_rxMotion_norm, 5);
				}

				if(L3_pressed)
				{
					GUI_SetColor(GUI_WHITE);
					GUI_FillCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(disp_xMotion_norm_old, disp_yMotion_norm_old, 5);
					GUI_SetColor(GUI_BLACK);
					GUI_DrawCircle(disp_xMotion_norm, disp_yMotion_norm, 5);
					L3_pressed_flag = 1;
				}else if(L3_pressed==0&&L3_pressed_flag)
				{
					GUI_SetColor(GUI_BLACK);
					GUI_FillCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					L3_pressed_flag = 0;
					GUI_SetColor(GUI_WHITE);
					GUI_DrawCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(disp_xMotion_norm, disp_yMotion_norm, 5);
				}

				if(R3_pressed)
				{
					GUI_SetColor(GUI_WHITE);
					GUI_FillCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(disp_zMotion_norm_old, disp_rxMotion_norm_old, 5);
					GUI_SetColor(GUI_BLACK);
					GUI_DrawCircle(disp_zMotion_norm, disp_rxMotion_norm, 5);
					R3_pressed_flag = 1;
				}else if(R3_pressed==0&&R3_pressed_flag)
				{
					GUI_SetColor(GUI_BLACK);
					GUI_FillCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					R3_pressed_flag = 0;
					GUI_SetColor(GUI_WHITE);
					GUI_DrawCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(disp_zMotion_norm, disp_rxMotion_norm, 5);
				}

				disp_xMotion_norm_old = disp_xMotion_norm;
				disp_yMotion_norm_old = disp_yMotion_norm;
				disp_zMotion_norm_old = disp_zMotion_norm;
				disp_rxMotion_norm_old = disp_rxMotion_norm;

				//GUI_CURSOR_SetPosition(127-display_joystick_centerX+disp_zMotion/display_joystick_ratio, display_joystick_centerY+disp_rxMotion/display_joystick_ratio);
				GUI_SetColor(GUI_WHITE);
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringAt("", 0, 53);
				GUI_DispFloat((float)xMotion/127, 5);
				GUI_DispString("/");
				GUI_DispFloat((float)yMotion/127, 5);
				GUI_DispString("   ");
				GUI_DispFloat((float)zMotion/127, 5);
				GUI_DispString("/");
				GUI_DispFloat((float)rxMotion/127, 5);
				break;

			case ITEM_ID_CALIBRATION:
				menu_enter = 0;
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Current joystick offset x/y", 63, 3);
				GUI_DispStringAt("L: ", 2, 15);
				GUI_DispDec(joystick_left_xOffset, 3);
				GUI_DispString("/");
				GUI_DispDec(joystick_left_yOffset, 3);
				GUI_DispString("    R: ");
				GUI_DispDec(joystick_right_xOffset, 3);
				GUI_DispString("/");
				GUI_DispDec(joystick_right_yOffset, 3);
				GUI_DispStringHCenterAt("Recalibrate?", 63, 29);
				GUI_DispStringAt("Yes", 49, 42);
				GUI_DrawBitmap(&bmarrow, 70, 41);
				display_showExit();
				if(menu_enter)
				{
					menu_enter = 0;
					GUI_Clear();
					GUI_DispStringHCenterAt("Keep joysticks in", 63, 3);
					GUI_DispStringHCenterAt("zero position", 63, 15);
					GUI_SetFont(&GUI_Font13_1);
					GUI_DispStringAt("3", 62, 32);
					GUI_Delay(1000);
					GUI_DispStringAt("2", 62, 32);
					GUI_Delay(1000);
					GUI_DispStringAt("1", 62, 32);
					GUI_Delay(1000);
					joystick_calibrate_flag = 1;
					GUI_DispStringHCenterAt("Offset calibrated", 62, 47);
					GUI_Delay(1000);
					GUI_Clear();
					GUI_SetFont(&GUI_Font8_1);

					GUI_DrawCircle(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DrawCircle(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR);
					GUI_DispStringHCenterAt("Tilt and rotate joysticks", 63, 53);
					uint8_t calibTimeCounter = 0;
					uint8_t calibTime = 9;
					GUI_DispDecAt(calibTime, 62, 22, 1);
					for(uint8_t i = 0; i<100; i++)
					{
						GUI_DrawArc(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR_calib, display_joystick_drawR_calib, 120+(i*15), 240+(i*15));
						GUI_DrawArc(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR_calib, display_joystick_drawR_calib, 120+(i*15), 240+(i*15));
						GUI_Delay(100);
						GUI_SetColor(GUI_BLACK);
						GUI_DrawArc(display_joystick_centerX, display_joystick_centerY, display_joystick_drawR_calib, display_joystick_drawR_calib, 120+(i*15), 240+(i*15));
						GUI_DrawArc(127-display_joystick_centerX, display_joystick_centerY, display_joystick_drawR_calib, display_joystick_drawR_calib, 120+(i*15), 240+(i*15));
						GUI_SetColor(GUI_WHITE);
						calibTimeCounter++;
						if(calibTimeCounter == 10){
							calibTimeCounter = 0;
							calibTime--;
							GUI_DispDecAt(calibTime, 62, 22, 1);
						}
					}
					GUI_Clear();
					GUI_DispStringHCenterAt("Calibration done!", 63, 30);
					GUI_Delay(1000);
					GUI_Clear();
					menu_item_id_entered = ITEM_ID_MAINMENU;
				}
				break;
			case ITEM_ID_CONNECTION:
				menu_enter = 0;
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringAt("Do you want to delete ", 10, 3);
				GUI_DispStringAt("all BLE connections? ", 12, 15);
				GUI_DispStringAt("Yes", 49, 29);
				GUI_DrawBitmap(&bmarrow, 70, 28);
				GUI_DispStringAt("No", 52, 42);
				GUI_DrawBitmap(&bmarrowLeft, 70, 41);
				display_showExit();

				if(menu_enter)
				{
					menu_enter = 0;
					app_bt_bond_delete_info();
					GUI_Clear();
					GUI_DispStringAt("All BLE bond information", 10, 20);
					GUI_DispStringAt("deleted", 44, 32);
					GUI_Delay(2000);
					menu_item_id_entered = ITEM_ID_MAINMENU;
				}



				break;
			case ITEM_ID_VIBRATION:
				menu_enter = 0;
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Vibration is currently ", 63, 3);
				GUI_SetFont(&GUI_Font8x13_1);
				if(rumble_active)
				{
					GUI_DispStringHCenterAt("  activated  ", 63, 14);
					GUI_SetFont(&GUI_Font8_1);
					GUI_DispStringHCenterAt(" Deactivate vibration? ", 63, 32);
					GUI_DispStringAt("Yes", 49, 46);
					GUI_DrawBitmap(&bmarrow, 70, 45);
					display_showExit();

				}
				else if(!rumble_active)
				{
					GUI_DispStringHCenterAt(" deactivated ", 63, 14);
					GUI_SetFont(&GUI_Font8_1);
					GUI_DispStringHCenterAt("  Activate vibration?  ", 63, 32);
					GUI_DispStringAt("Yes", 49, 46);
					GUI_DrawBitmap(&bmarrow, 70, 45);
					display_showExit();
				}
				if(menu_enter)
				{
					menu_enter = 0;
					rumble_active = rumble_active^0x01;
					menu_item_id_entered = ITEM_ID_MAINMENU;
				}


				break;
			case ITEM_ID_PRESENCE:
				menu_enter = 0;
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Presence detection: ", 63, 3);
				GUI_SetFont(&GUI_Font8x13_1);
				if(Capsense_presence_activated)
				{
					GUI_DispStringHCenterAt("  activated  ", 63, 14);
					GUI_SetFont(&GUI_Font8_1);
					GUI_DispStringHCenterAt(" Deactivate? ", 63, 32);
					GUI_DispStringAt("Yes", 49, 46);
					GUI_DrawBitmap(&bmarrow, 70, 45);
					display_showExit();

				}
				else if(!Capsense_presence_activated)
				{
					GUI_DispStringHCenterAt(" deactivated ", 63, 14);
					GUI_SetFont(&GUI_Font8_1);
					GUI_DispStringHCenterAt("  Activate?  ", 63, 32);
					GUI_DispStringAt("Yes", 49, 46);
					GUI_DrawBitmap(&bmarrow, 70, 45);
					display_showExit();
				}
				if(menu_enter)
				{
					menu_enter = 0;
					Capsense_presence_activated = Capsense_presence_activated^0x01;
					menu_item_id_entered = ITEM_ID_MAINMENU;
				}
				break;
			case ITEM_ID_MAINMENU:
				if(display_joystick_curserUpFlag)
				{
					display_joystick_curserUpFlag = 0;
					GUI_CURSOR_Hide();
				}

				if(!WM_IsWindow(wm_handle))
				{
					GUI_Clear();
					wm_handle = CreateWindow();
					hProg = WM_GetDialogItem(wm_handle, ID_PROGBAR_0);
					hList = WM_GetDialogItem(wm_handle, ID_LISTBOX_0);
					LISTBOX_SetSel(hList, currentMenuSel);
					display_updateVbatProgbar(hProg);
					home_led_state_change_flag = 1;
				}
				GUI_Exec();
				VbatProgbarCounter++;
				if(VbatProgbarCounter>100000)
				{
					display_updateVbatProgbar(hProg);
					VbatProgbarCounter = 0;
				}
				break;
			}
		}
		else if(!controlActivated)
		{
			if(WM_IsWindow(wm_handle)||menu_item_id_entered != ITEM_ID_MAINMENU)
			{
				WM_DeleteWindow(wm_handle);
				GUI_CURSOR_Hide();
				GUI_Clear();
				menu_item_id_entered = ITEM_ID_MAINMENU;
				home_led_state_change_flag = 1;
			}

		}

		if(!controlActivated&&menu_connection_item_id_entered!=MENU_CONNECTION_ITEM_DEFAULT)
		{
			switch(menu_connection_item_id_entered){
			case MENU_CONNECTION_ITEM_ADVERTISING:
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Bluetooth advertising...", 63, 3);
				GUI_DispStringHCenterAt("Please connect to", 63, 20);
				GUI_DispStringHCenterAt("'XENSIV Gamepad'", 63, 32);
				break;
			case MENU_CONNECTION_ITEM_PAIRSUCCESS:
				GUI_Clear();
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Successfully paired to", 63, 3);
				char str[18];
				uint8_t bdadr[6];
				for(uint8_t i = 0; i<6; i++)
				{
					bdadr[i] = 	bondinfo.link_keys[0].bd_addr[i];
				}
				sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", bdadr[0],bdadr[1],bdadr[2],bdadr[3],bdadr[4],bdadr[5]);
				GUI_DispStringHCenterAt(str, 63, 20);
				GUI_Delay(1000);
				for(uint16_t i=0; i<300; i++)
				{
					if(controlActivated||menu_connection_item_id_entered!=MENU_CONNECTION_ITEM_PAIRSUCCESS)
					{
						menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;
						GUI_Clear();
						break;
					}
					GUI_Delay(10);
				}
				menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;
				GUI_Clear();
				break;
			case MENU_CONNECTION_ITEM_PAIRERROR:
				GUI_Clear();
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Could not find any", 63, 3);
				GUI_DispStringHCenterAt("compatible Bluetooth device", 63, 15);
				GUI_Delay(1000);
				for(uint16_t i=0; i<300; i++)
				{
					if(controlActivated||menu_connection_item_id_entered!=MENU_CONNECTION_ITEM_PAIRERROR)
					{
						menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;
						GUI_Clear();
						break;
					}
					GUI_Delay(10);
				}
				menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;
				GUI_Clear();
				break;
			case MENU_CONNECTION_ITEM_DISCONNECTED:
				GUI_Clear();
				GUI_SetFont(&GUI_Font8_1);
				GUI_DispStringHCenterAt("Disconnected", 63, 3);
				GUI_DispStringHCenterAt("Reason:", 63, 15);
				GUI_SetFont(&GUI_Font4x6);
				GUI_DispStringHCenterAt(disconnect_reason, 63, 30);
				GUI_Delay(3000);
				GUI_SetFont(&GUI_Font8_1);
				GUI_Clear();
				if(app_bt_adv_conn_state==APP_BT_ADV_ON_CONN_OFF)
				{
					menu_connection_item_id_entered = MENU_CONNECTION_ITEM_ADVERTISING;
				}
				else if(app_bt_adv_conn_state == APP_BT_ADV_OFF_CONN_ON)
				{
					menu_connection_item_id_entered = MENU_CONNECTION_ITEM_PAIRSUCCESS;
				}
				else
				{
					menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;
				}
				break;
			case MENU_CONNECTION_ITEM_DEFAULT:
				break;
			}
		}
		if(!Capsense_presence&&Capsense_presence_activated)
		{
			printf("powerdown\r\n");
			if(WM_IsWindow(wm_handle))
			{
				WM_DeleteWindow(wm_handle);
				GUI_Clear();
				currentMenuSel = menu_item_id_entered;
			}
			if(show_powerdown_flag)
			{
				GUI_Clear();
				display_show_powerDown();
				show_powerdown_flag = 0;
			}
			while(!Capsense_presence);
			display_startupSequence_static();
			show_powerdown_flag = 1;
		}
	}
}

