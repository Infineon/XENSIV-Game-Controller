/******************************************************************************
* File Name: gc_GameController.c
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
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "gc_GameController.h"



/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

struct hid_rpt_msg gamecontroller_msg;



/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
/*TODO: Currently, the game controller scan tasks are implemented in the app_ble_task, where all I2C tasks are located. Use this to implement functions that do not use I2C*/
void app_gameController_task(void *pvParameters)
{
    uint32_t ulNotifiedValue;
	printf("Gamepad task started!\r\n");


	while(1){

		xTaskNotifyWait(0, 0xffffffff, &ulNotifiedValue, 0x10);

		if (pdPASS != xQueueSend(hid_rpt_q, &gamecontroller_msg, TASK_MAX_WAIT))
		{
			//printf("Failed to send msg to HID rpt Queue\r\n");
			continue;
		}

	}
}
