

/******************************************************************************
* File Name: bt_management.h
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

#ifndef GC_BT_MANAGEMENT_H_
#define GC_BT_MANAGEMENT_H_



/*******************************************************************************
* Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal.h"

#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "wiced_bt_dev.h"
#include "cybsp_bt_config.h"

#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "GeneratedSource/cycfg_gap.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "gc_bt_utils.h"
#include "semphr.h"


/******************************************************************************
 * Constants
 ******************************************************************************/

typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;


////
/* Sufficient Heap size for Bluetooth activities */
#define BT_HEAP_SIZE                        (0x1000)

/* Bitflags for LE secure pairing keys IO capabilities event */
#define PAIRING_CAPS_KEYS_FLAG      \
        (BTM_LE_KEY_PENC | BTM_LE_KEY_PID)

/* Key Size for LE secure pairing key IO capabilities event */
#define PAIRING_CAPS_KEY_SIZE       (16u)


typedef void                 (*pfn_free_buffer_t)            (uint8_t *);

/******************************************************************************
 * Extern variables
 ******************************************************************************/
extern uint16_t ble_conn_id;
extern uint8_t ble_conn_adr[6];
extern uint8_t ble_boot_status;

extern uint16_t num_of_congestions;

extern app_bt_adv_conn_mode_t app_bt_adv_conn_state;


extern TaskHandle_t ble_task_h;

enum menu_connection_item_id
{
	MENU_CONNECTION_ITEM_ADVERTISING = 0x00,
	MENU_CONNECTION_ITEM_PAIRSUCCESS  = 0x01,
	MENU_CONNECTION_ITEM_PAIRERROR = 0x02,
	MENU_CONNECTION_ITEM_DISCONNECTED = 0x03,
	MENU_CONNECTION_ITEM_DEFAULT = 0x04,
};

extern volatile uint8_t menu_connection_item_id_entered;

extern char *disconnect_reason;

/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void le_app_init(void);
wiced_bt_gatt_status_t le_app_gatt_event_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
wiced_bt_gatt_status_t le_app_set_value(uint16_t attr_handle, uint8_t *p_val, uint16_t len);
wiced_bt_gatt_status_t le_app_write_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_write_req_t *p_write_req, uint16_t len_req, uint16_t *p_error_handle);
wiced_bt_gatt_status_t le_app_read_handler( uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req, uint16_t len_req, uint16_t *p_error_handle);
wiced_bt_gatt_status_t le_app_connect_handler(wiced_bt_gatt_connection_status_t *p_conn_status);
wiced_bt_gatt_status_t le_app_server_handler (wiced_bt_gatt_attribute_request_t *p_attr_req, uint16_t *p_error_handle);
gatt_db_lookup_table_t  *le_app_find_by_handle(uint16_t handle);
void* app_alloc_buffer(int len);
void app_free_buffer(uint8_t *p_buf);
gatt_db_lookup_table_t  *le_app_find_by_handle(uint16_t handle);
wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested, uint16_t *p_error_handle);

#endif /* GC_BT_MANAGEMENT_H_ */
