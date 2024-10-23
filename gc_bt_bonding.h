/******************************************************************************
* File Name: gc_bt_bonding.h
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

#ifndef GC_BT_BONDING_H_
#define GC_BT_BONDING_H_


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg_bt_settings.h"




/******************************************************************************
 * MACROS
 ******************************************************************************/
/* Max number of bonded devices */
#define  BOND_MAX                            (3u)
#define  NUM_OF_SLOTS                        (2u)
#define  NUM_OF_CCCD                         (3u)
/* LE Key Size : 16 */
#define  KEY_SIZE_MAX                        (0x10)

/* enum for bondinfo structure */
enum
{
    NUM_BONDED,
    LAST_CONNECT_INDEX
};
/******************************************************************************
 * Global variables
 ******************************************************************************/
/* Structure to store info that goes into serial flash -
 * it holds the number of bonded devices, remote keys and local keys
 */
typedef struct
{
    uint8_t slot_data[NUM_OF_SLOTS];
    wiced_bt_device_link_keys_t link_keys[BOND_MAX];
    wiced_bt_device_address_t local_bd_addr[BOND_MAX];
    wiced_bt_device_address_t next_local_bd_addr;
    /* Variable to store CCCD Data
     Position 0 - bas_battery_level_cccd
     Position 1 - mouse_in_report_cccd
     Position 2 - boot_mouse_in_report_cccd
     */
    uint8_t cccd_flags[BOND_MAX];
} tBondInfo;

/* Structure containing bonding info of peer devices */
extern tBondInfo bondinfo;

/* Local Identity Key */
extern wiced_bt_local_identity_keys_t identity_keys;

/* Array Index of the current peer device */
extern uint8_t bondindex;


/****************************************************************************
 * FUNCTION DECLARATIONS
 ***************************************************************************/
/* Peer Device and Bond Info Management APIs */
cy_rslt_t app_bt_bond_restore_data(void);
cy_rslt_t app_bt_bond_update_slot_data(void);
cy_rslt_t app_bt_bond_update_data(void);
uint8_t app_bt_bond_check_info(void);
uint8_t app_bt_bond_get_info_count(void);
cy_rslt_t app_bt_bond_get_new_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_get_local_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_update_new_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_update_local_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_update_index(void);
uint8_t app_bt_bond_get_index(void);
cy_rslt_t app_bt_bond_delete_info(void);
wiced_result_t app_bt_bond_delete_device_info(uint8_t index);
uint8_t app_bt_bond_find_device_in_flash(uint8_t *bd_addr);
wiced_result_t app_bt_bond_add_devices_to_address_resolution_db(void);
wiced_result_t app_bt_bond_check_device_info(uint8_t *bd_addr);

/* Security Keys Management APIs */
cy_rslt_t app_bt_bond_get_device_link_keys(wiced_bt_device_link_keys_t *link_key);
cy_rslt_t app_bt_bond_save_device_link_keys(wiced_bt_device_link_keys_t *link_key);
cy_rslt_t app_bt_bond_save_local_identity_key(wiced_bt_local_identity_keys_t id_key);
cy_rslt_t app_bt_bond_read_local_identity_keys(void);

/* CCCD Management APIs */
cy_rslt_t app_bt_bond_update_cccd(uint16_t cccd, uint8_t index);
void app_bt_bond_modify_cccd_in_nv_storage(uint16_t attr_handle, uint8_t *p_val);
cy_rslt_t app_bt_bond_restore_cccd_using_link_key(wiced_bt_device_link_keys_t *p_link_key);
cy_rslt_t app_bt_bond_restore_cccd(void);

/* Helper APIs */
void app_bt_bond_print_data(void);
void app_bt_bond_print_info_stats(void);



#endif /* GC_BT_BONDING_H_ */

