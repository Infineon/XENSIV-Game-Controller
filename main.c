/******************************************************************************
* File Name: main.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This is the main.c source code for the XENSIV Game Controller
*              for ModusToolbox.
*
*
* *****************************************************************************
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
#include <string.h>
#include "stdio.h"

#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal.h"

#include <FreeRTOS.h>

#include <task.h>
#include <queue.h>
#include "semphr.h"

#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "wiced_bt_dev.h"
#include "cybsp_bt_config.h"

#include "cy_serial_flash_qspi.h"
#include "gc_serial_flash.h"

#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "GeneratedSource/cycfg_gap.h"

#include "gameController_boardControl.h"
#include "gc_GameController.h"
#include "joystick.h"
#include "display.h"
#include "MBR3.h"
#include "TLE75008.h"
#include "i2c.h"
#include "TLV493D.h"
#include "TLx493D_inc.h"
#include "trigger.h"
#include "adc.h"

#include "gc_bt_management.h"
#include <gc_bt_utils.h>
#include "gc_bt_hid.h"


/*******************************************************************************
* Macros
********************************************************************************/

/* Sufficient Heap size for Bluetooth activities */
#define BT_HEAP_SIZE                        (0x1000)

/* hid queue size */
#define HID_MSG_Q_SZ                            ((uint8_t)100u)
/* hid queue item size */
#define HID_MSG_Q_ITEM_SZ                       (sizeof(struct hid_rpt_msg))

/* Task names for  Bluetooth LE HID Keyboard tasks */
#define BLE_TASK_NAME                   "BLE Task"
#define GAMEPADSCAN_TASK_NAME           "Keyscan Task"
#define DISPLAY_TASK_NAME				"Display Task"


/* Stack sizes for  Bluetooth LE HID Keyboard tasks */
#define BLE_TASK_STACK_SIZE                 (512u)
#define GAMEPADSCAN_TASK_STACK_SIZE         (512u)
#define DISPLAY_TASK_STACK_SIZE             (1024u)

/* Task Priorities of  Bluetooth LE HID Keyboard Tasks */
#define BLE_TASK_PRIORITY                   (3)
#define GAMEPADSCAN_TASK_PRIORITY           (3)
#define DISPLAY_TASK_PRIORITY               (3)

/* Message type to differentiate Game Controller msg and Battery level msg */
#define GAMECONTROLLER_MSG_TYPE                 (1u)
#define BATT_MSG_TYPE                           (2u)

/* Bitflags for LE secure pairing keys IO capabilities event */
#define PAIRING_CAPS_KEYS_FLAG      \
        (BTM_LE_KEY_PENC | BTM_LE_KEY_PID)

/* Key Size for LE secure pairing key IO capabilities event */
#define PAIRING_CAPS_KEY_SIZE       (16u)



/*******************************************************************************
* Variable Definitions
*******************************************************************************/

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/////
///* Queue for sending gc events and battery reports to Bluetooth LE Task */
//QueueHandle_t hid_rpt_q;

///*Task handle of scan for any button/trigger/joystick updates*/
//TaskHandle_t gameControllerScan_task_h;

/*Task handle of scan for any button/trigger/joystick updates*/
TaskHandle_t display_task_h;

/*app_hid_gamecontroller_report[bufferSize][reportSize]*/
uint8_t app_hid_gamecontroller_report[10][6];


/*******************************************************************************
* Function Prototypes
********************************************************************************/

void app_display_task(void *pvParameters);


/******************************************************************************
 * Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main()
{

    wiced_result_t wiced_result;
    cy_rslt_t cy_result;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package */
    cy_result = cybsp_init();

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);


    printf("\r\n************* Game Controller initialization *************\r\n");

    gc_cybsp_assert(cy_result, "BSP init:");

    gc_initBoard();
    i2c_init();
    spi_init();
    MBR3_init();
    TLE75008_init();
    trigger_R1L1_init();
    adc_Vbat_channel_init();


    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

	app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

	/*Generate unique BT ID for every gaming controller*/
	uint64_t id = 0;
	id = Cy_SysLib_GetUniqueId();
	uint8_t id1 = (uint8_t)(id>>32);
	uint8_t id2 = (uint8_t)(id>>40);
	uint8_t id3 = (uint8_t)(id>>56);
	cy_bt_device_address[3] = id1;
	cy_bt_device_address[4] = id2;
	cy_bt_device_address[5] = id3;

	/* Register call back and configuration with stack */
	wiced_result = wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);
    gc_wiced_assert(wiced_result, "Bluetooth Stack Initialization:");


	/* Create a buffer heap, make it the default heap.  */
	if ( NULL == wiced_bt_create_heap("app", NULL, BT_HEAP_SIZE, NULL, WICED_TRUE))
	{
	   printf("Heap create Failed");
	}

	/* Keyscan Queue to send msgs to Bluetooth LE Task */
	hid_rpt_q =  xQueueCreate(HID_MSG_Q_SZ, HID_MSG_Q_ITEM_SZ);
	if(NULL == hid_rpt_q)
	{
	   printf("HID Report Queue creation failed! \r\n");
	   CY_ASSERT(0);
	}

	if ( pdPASS != xTaskCreate(app_ble_task,
                                  BLE_TASK_NAME,
                                  BLE_TASK_STACK_SIZE,
                                  NULL,
                                  BLE_TASK_PRIORITY,
                                  &ble_task_h))
	{
	   /* Task is not created due to insufficient Heap memory.
		* Use vApplicationMallocFailedHook() callback to trap.
		* And xPortGetFreeHeapSize() to query unallocated heap memory
		*/
	   printf("Bluetooth LE Task creation failed");
	}

	/*Currently, the game controller scan tasks are implemented in the app_ble_task, where all I2C tasks are located. Use this to implement functions that do not use I2C*/
	if( pdPASS != xTaskCreate(app_gameController_task,
							 GAMEPADSCAN_TASK_NAME,
							 GAMEPADSCAN_TASK_STACK_SIZE,
							 NULL,
							 GAMEPADSCAN_TASK_PRIORITY,
							 &gameControllerScan_task_h))
	{
	   printf("GC Task creation failed");
	   CY_ASSERT(0);
	}

	/*Initialize the display task*/
	if( pdPASS != xTaskCreate(app_display_task,
							 DISPLAY_TASK_NAME,
							 DISPLAY_TASK_STACK_SIZE,
							 NULL,
							 DISPLAY_TASK_PRIORITY,
							 &display_task_h))
	{
	   printf("Display Task creation failed");
	   CY_ASSERT(0);
	}

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* Should never get here */
	CY_ASSERT(0) ;
}

/* END OF FILE [] */
