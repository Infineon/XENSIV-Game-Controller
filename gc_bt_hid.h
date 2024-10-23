/******************************************************************************
* File Name: gc_bt_hid.h
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
*
*******************************************************************************/

#ifndef GC_BT_HID_H_
#define GC_BT_HID_H_


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg_bt_settings.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"


/******************************************************************************
 * MACROS
 ******************************************************************************/
/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
/*Differentiate message types gamecontroller_report and battery message*/
#define GAMECONTROLLER_MSG_TYPE	(1U)
#define BATT_MSG_TYPE			(2U)
//#define DISPLAY_MSG_TYPE		(3U)


/******************************************************************************
 * Global variables
 ******************************************************************************/
extern TaskHandle_t gameControllerScan_task_h;

/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
extern QueueHandle_t hid_rpt_q;

/*Game Controller report*/
typedef struct
{
    /* Button state in bitmap */
    uint16_t buttonState;
    /* X motion joystick left 8 bits */
    int8_t xMotion;
    /* Y motion joystick left 8 bits */
    int8_t yMotion;
    /* X motion joystick right 8 bits */
    int8_t zMotion;
    /* Y motion joystick right 8 bits */
    int8_t rxMotion;
} gamecontroller_rpt_t;

/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
/* HID Message type : Keyscan Message or Battery level */
union msg
{
    gamecontroller_rpt_t gc_report;
    uint8_t batt_level;
};

/*TODO: own task for gc scan, separated from ble task; hand over scan by messages and queues*/
struct hid_rpt_msg
{
    uint8_t msg_type;	//GAMECONTROLLER_MSG_TYPE, BATT_MSG_TYPE
    union msg data;
};



/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
void app_ble_task(void *pvParameters);


#endif /* GC_BT_HID_H_ */
