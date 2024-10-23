/******************************************************************************
* File Name: gc_bt_hid.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This file contains the bt_hid_task and related functions.
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
#include "gc_bt_hid.h"

#include "gc_bt_management.h"

#include "gameController_boardControl.h"

/*exclude again: */
#include "joystick.h"
#include "gc_serial_flash.h"
#include "display.h"
#include "trigger.h"
#include "MBR3.h"
#include "adc.h"
#include "TLE75008.h"


/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
/*******************************************************************************/
/*Task handle of scan for any button/trigger/joystick updates*/
TaskHandle_t gameControllerScan_task_h;

/* Queue for sending gc events and battery reports to Bluetooth LE Task */
QueueHandle_t hid_rpt_q;

/*declare a structure that holds the report message*/
struct hid_rpt_msg rpt_msg;

uint8_t trigger_L1_pressed = 0;
uint8_t trigger_R1_pressed = 0;

uint8_t batterySOC = 0;


/*******************************************************************************
* Function Definitions
*******************************************************************************/
void app_ble_task(void *pvParameters)
{

	BaseType_t xResult = pdFAIL;
	volatile uint32_t wait = 0;

    uint32_t ulNotifiedValue;
    cyhal_system_delay_ms(100);

    /*Init TLV493D after scheduler has started*/
    joysticks_init();
	trigger_R2L2_init();

    gc_flash_init();

    adc_TMR_joystick_channels_init();

    uint16_t reportCounter = 0;

	while(1){
		/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
		xResult = xQueueReceive(hid_rpt_q, &(rpt_msg), 0x1UL);
		if(wait>10){
			xTaskNotify(gameControllerScan_task_h, 7, 1);
			wait = 0;
		};

		if (xResult != pdPASS)
		{
			wait++;
			continue;
		}

		cyhal_system_delay_ms(1);

		/*TODO: TMR Joystick*/
		/*
		uint16_t tmr_rx=0;
		uint16_t tmr_ry=0;
		uint16_t tmr_lx=0;
		uint16_t tmr_ly=0;
		uint32_t cumul = 0;
		int8_t tmr_rx_8b;
		uint8_t rep = 20;
		for(int p = 0; p<rep; p++)
		{
			tmr_rx = cyhal_adc_read_u16(&adc_obj_tmr_rx);
			tmr_ry = cyhal_adc_read_u16(&adc_obj_tmr_ry);
			tmr_lx = cyhal_adc_read_u16(&adc_obj_tmr_lx);
			tmr_ly = cyhal_adc_read_u16(&adc_obj_tmr_ly);
			//printf("rx: %u, ry: %u, lx: %u, ly: %u\r\n", tmr_rx, tmr_ry, tmr_lx, tmr_ly);
			cumul += tmr_rx;
		}
		*/

		/*Update state of Home RGB-LED*/
		if(led_activate_flag)
		{
			TLE75008_LED_deactivate(led_activate_flag);
			led_activate_flag = 0;
		}

		if(led_toggle_flag)
		{
			TLE75008_LED_toggle(led_toggle_flag);
			led_toggle_flag = 0;
		}


		if(home_led_state_change_flag)
		{
			TLE75008_LED_deactivate(LED_HOME_GREEN);
			TLE75008_LED_deactivate(LED_HOME_RED);
			TLE75008_LED_deactivate(LED_HOME_BLUE);

			if(!Capsense_presence)
			{

			}
			else if(controlActivated)
			{
				TLE75008_LED_toggle_ms(LED_HOME_GREEN, 500);
				home_led_state_change_flag = 0;
			}
			else
			{
				switch(app_bt_adv_conn_state)
				{
				case APP_BT_ADV_OFF_CONN_OFF:
					TLE75008_LED_activate(LED_HOME_RED);
					home_led_state_change_flag = 0;
					break;
				case APP_BT_ADV_ON_CONN_OFF:
					TLE75008_LED_toggle_ms(LED_HOME_BLUE, 500);
					home_led_state_change_flag = 0;
					break;
				case APP_BT_ADV_OFF_CONN_ON:
					TLE75008_LED_activate(LED_HOME_GREEN);
					home_led_state_change_flag = 0;
					break;
				default:
					printf("unkown BLE conn state change\r\n");
				}
			}

		}

		/*recalibration trigger of joystick in display task*/
		if(joystick_calibrate_flag)
		{
			joystick_calibrate(&tlv493d_joystick_left);
			joystick_calibrate(&tlv493d_joystick_right);
			joystick_calibrate_flag = 0;
			printf("joysticks recalibrated\r\n");
		}



		trigger_L1_pressed = get_triggerL1_state();
		trigger_R1_pressed = get_triggerR1_state();

		//printf("L1: %i, R1: %i\r\n", trigger_L1_pressed, trigger_R1_pressed);

		joystick_get_val_polarToTiltFitted(&tlv493d_joystick_right, &zMotion, &rxMotion, &R3_pressed);
		joystick_get_val_polarToTiltFitted(&tlv493d_joystick_left, &xMotion, &yMotion, &L3_pressed);

		uint16_t buttonPress = MBR3_getButtonStates();
		buttonPress |= ((L3_pressed&=0x01)<<10);
		buttonPress |= ((R3_pressed&=0x01)<<11);

		uint16_t buttonPressTrigger = trigger_getButtonStates();
		buttonPress |= (buttonPressTrigger&=0xF0);


		if(ble_conn_id!=0)
		{

			app_hids_gamecontrolller_report[0] = (uint8_t)(buttonPress);
			app_hids_gamecontrolller_report[1] = (uint8_t)(buttonPress>>8);
			app_hids_gamecontrolller_report[2] = xMotion;
			app_hids_gamecontrolller_report[3] = yMotion;
			app_hids_gamecontrolller_report[4] = zMotion;
			app_hids_gamecontrolller_report[5] = rxMotion;


			cyhal_system_delay_ms(11);

			wiced_bt_gatt_status_t gatt_status = wiced_bt_gatt_server_send_notification(ble_conn_id,
																			HDLC_HIDS_GAMECONTROLLLER_REPORT_VALUE,
																			app_hids_gamecontrolller_report_len,
																			app_hids_gamecontrolller_report,
																			 NULL);

			reportCounter++;

			app_bas_battery_level[0] = getSOCBat_Mean();
			/*TODO: send the Battery HID report data */
//			gatt_status = wiced_bt_gatt_server_send_notification(ble_conn_id,
//																 HDLC_BAS_BATTERY_LEVEL_VALUE,
//																 app_bas_battery_level_len,
//																 app_bas_battery_level,
//																 NULL);

			if (gatt_status != WICED_BT_GATT_SUCCESS)
			{

				printf("gatt server notification not successful. Code: %04X; id: %i\r\n", gatt_status, ble_conn_id);
				/*GATT congestion, wait for GATT free event to notify task*/
				xTaskNotifyWait(0, 0, &ulNotifiedValue, pdMS_TO_TICKS(TASK_MAX_WAIT));
				gatt_status = WICED_BT_GATT_SUCCESS;
				gatt_status = wiced_bt_gatt_server_send_notification(ble_conn_id,
																	HDLC_HIDS_GAMECONTROLLLER_REPORT_VALUE,
																	app_hids_gamecontrolller_report_len,
																	app_hids_gamecontrolller_report,
																	 NULL);
				if (gatt_status != WICED_BT_GATT_SUCCESS)
				{
					gatt_status = wiced_bt_gatt_disconnect (ble_conn_id);
					if (gatt_status == WICED_BT_GATT_SUCCESS) printf("GATT disconnected! \r\n");
					gatt_status = wiced_bt_gatt_register(le_app_gatt_event_callback);
					gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
					if (gatt_status == WICED_BT_GATT_SUCCESS) printf("GATT reregistration successfull! \r\n");
					gatt_status = wiced_bt_gatt_validate_conn_id(ble_conn_id);
					if (gatt_status != WICED_BT_GATT_SUCCESS) printf("validation code: %04X \r\n", gatt_status);
					printf("ReportCounter = %i\r\n", reportCounter);
				}
			}

			buttonStates = 0;
		}
		if(app_hids_protocol_mode[0]== 0x01){
			if(app_hids_gamecontrolller_report_client_char_config[0] == GATT_CLIENT_CONFIG_NOTIFICATION){

			}
		}

	}
}
