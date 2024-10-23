/******************************************************************************
* File Name: gc_bt_management.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This file contains the callback functions for the bt/ble stack/gatt database.
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
#include "gc_bt_management.h"
#include "gc_bt_bonding.h"
#include "gc_bt_utils.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"
#include "TLE75008.h"
#include "MBR3.h"

#include "gameController_boardControl.h"


/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
static uint16_t bt_connection_id;

app_bt_adv_conn_mode_t app_bt_adv_conn_state;


uint16_t ble_conn_id;

uint8_t ble_conn_adr[6];

uint8_t ble_boot_status = 0;

/* This variable tracks the number of congestion events during BLE data transfer */
uint16_t num_of_congestions;

/* Flag to check for GATT congestion */
uint8_t is_gatt_congested;

/* Task handle of Bluetooth LE HID keybord Application  */
TaskHandle_t ble_task_h;

uint8_t L3R3_flag;

wiced_bt_device_address_t peer_bd_addr = { 0 };

static uint8_t reset_bond_data = 0;

volatile uint8_t menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DEFAULT;

char *disconnect_reason;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/**************************************************************************************************
* Function Name: app_bt_management_callback
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t       bt_dev_status       = WICED_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    wiced_bt_dev_ble_pairing_info_t *p_pairing_info = NULL;



    switch (event)
    {
        case BTM_ENABLED_EVT:

            /* Retrieve Bond information from NV Storage to Bondinfo structure in RAM */
            /* The​ ​value​ ​of​ ​the​ ​​Client​ ​Characteristic​ ​Configuration​ ​​descriptor​ ​is​ ​persistent​
             * ​for​ ​bonded devices​ ​when​ ​not​ ​in​ ​a​ ​connection
             */
            /* CCCD needs to be restored */
            app_bt_bond_restore_data();

            /* Prints the Number of Bonded devices and free slots */
            app_bt_bond_print_info_stats();

			/* Print the Bond data(BD_ADDR, Link Keys & Identity keys) from EmEEPROM */
			app_bt_bond_print_data();

            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {

                wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                le_app_init();

                ble_boot_status = 1;
            }
            else
            {

                printf( "Bluetooth Disabled \n" );
            }

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                printf("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(0 == bt_connection_id)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                    printf("pair error\r\n");
                    home_led_state_change_flag = 1;
					menu_connection_item_id_entered = MENU_CONNECTION_ITEM_PAIRERROR;
                    ble_conn_id = 0;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                	home_led_state_change_flag = 1;
					menu_connection_item_id_entered = MENU_CONNECTION_ITEM_PAIRSUCCESS;
					if(Capsense_presence!=0)
                    {

                    }
                    ble_conn_id = bt_connection_id;
                }
            }
            else
            {
                /* Advertisement Started */
                printf("Advertisement started\n");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
				menu_connection_item_id_entered = MENU_CONNECTION_ITEM_ADVERTISING;
                home_led_state_change_flag = 1;
            }

            /* Update Advertisement LED to reflect the updated state */
            //adv_led_update();
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Need to compare with BT-SDK remote here for this event */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Set the IO capabilities */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                    BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data =
                    BTM_OOB_NONE;
            /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.auth_req =
                    (BTM_LE_AUTH_REQ_SC | BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM);
            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                    PAIRING_CAPS_KEYS_FLAG;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                    PAIRING_CAPS_KEYS_FLAG;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size =
                    PAIRING_CAPS_KEY_SIZE;

            /* Reset the peer bd addr and use new address got when pairing complete */
            //memset(peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			/* Paired Device Link Keys Request */
			printf("Paired Device Link keys Request Event for device ");
			print_bd_address((uint8_t*)(p_event_data->paired_device_link_keys_request.bd_addr));
			/* Need to search to see if the BD_ADDR we are looking for is in Flash. If not, we return WICED_BT_ERROR and the stack */
			/* will generate keys and will then call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they can be stored */

			bt_dev_status = app_bt_bond_check_device_info(p_event_data->paired_device_link_keys_request.bd_addr);

			if (bt_dev_status == WICED_BT_SUCCESS)
			{
				/* Save the peer address for later use */
				memcpy(peer_bd_addr,
					   p_event_data->paired_device_link_keys_request.bd_addr,
					   sizeof(wiced_bt_device_address_t));
				printf("Peer Device BD ADDR: ");
				print_bd_address(peer_bd_addr);

				/* Fetch the link keys saved previously */
				app_bt_bond_get_device_link_keys(&(p_event_data->paired_device_link_keys_request));
			}
			else
			{
				printf("Device Link Keys not found in the database! \n");
			}
			break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
			/* Save the link keys of paired device to Non volatile storage */
			printf("Paired Device Key Update \r\n");

			wiced_result = app_bt_bond_save_device_link_keys(&(p_event_data->paired_device_link_keys_update));

			if (CY_RSLT_SUCCESS == wiced_result)
			{
				printf("Successfully Bonded to ");
				print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
			}
			else
			{
				printf("Failed to bond! \r\n");
			}

			/* Print all security keys if needed */

			break;

        case BTM_PAIRING_COMPLETE_EVT:
			/* Received Pairing Complete Event */
			p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

			if (SMP_SUCCESS == p_pairing_info->reason) /* Bonding successful */
			{
				/* Save the peer address for later use */
				memcpy(peer_bd_addr,
					   p_event_data->pairing_complete.bd_addr,
					   sizeof(wiced_bt_device_address_t));
				/* Print Bond information stats once a new device is paired.
				 (pairing complete event) */
				printf("Successfully Bonded to: ");
				print_bd_address(p_event_data->pairing_complete.bd_addr);

				/* Update Num of bonded devices in slot data*/
				app_bt_bond_update_slot_data();

				/* Print bond info */
				app_bt_bond_print_info_stats();


			}
			else
			{
				printf("Bonding failed. Reason for failure: ");
				printf(get_pairing_status_name(p_pairing_info->reason));
				printf("\r\n");
				/* Delete host info and update bonded list */
			}

			break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
			/* Retrieve the identity keys from the NV storage */

			if (reset_bond_data)
			{
				app_bt_bond_delete_info();
				app_bt_bond_update_data();
			}

			/*
			 * If the key type is Identity keys; throw WICED_ERROR to cause the
			 * BT stack to generate new keys and then call
			 * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored
			 * */
			printf("Local Identity Key Request\r\n");
			/*Read Local Identity Resolution Keys*/
			wiced_result = app_bt_bond_read_local_identity_keys();
			if (CY_RSLT_SUCCESS == wiced_result)
			{
				memcpy(&(p_event_data->local_identity_keys_request),
					   &(identity_keys),
					   sizeof(wiced_bt_local_identity_keys_t));
				//app_bt_util_print_byte_array(&identity_keys, sizeof(wiced_bt_local_identity_keys_t));
				bt_dev_status = WICED_BT_SUCCESS;
			}
			else
			{
				bt_dev_status = WICED_BT_ERROR;
			}
			break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Save the identity keys to the NV storage */
            printf("Local Identity Key Update\n");
            wiced_result = app_bt_bond_save_local_identity_key(p_event_data->local_identity_keys_update);
            if (CY_RSLT_SUCCESS != wiced_result)
            {
                bt_dev_status = WICED_BT_ERROR;
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
			/* Encryption Status Change */
			printf("Encryption Status event for: ");
			print_bd_address(p_event_data->encryption_status.bd_addr);
			printf("Encryption Status event result: %d \r\n",
				   p_event_data->encryption_status.result);

			/* Check if the bond data of the device that got connected is already present */
			if (!memcmp(peer_bd_addr,
						p_event_data->encryption_status.bd_addr,
						sizeof(wiced_bt_device_address_t)))
			{

				/* Restore the CCCD stored earlier */
				app_bt_bond_restore_cccd();
				/* Enable all IN Report notifications. The HID Host will automatically
				 * enable all CCCDs. On some cases, the HID device is done independently
				 * for example chromecast. So Use it for debug purposes only.
				 */
				//          app_enable_all_cccds();

			}

			break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Connection parameter update status:%d, Connection Interval: %d, Connection Latency: %d, Connection Timeout: %d\n",
                                           p_event_data->ble_connection_param_update.status,
                                           p_event_data->ble_connection_param_update.conn_interval,
                                           p_event_data->ble_connection_param_update.conn_latency,
                                           p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        case BTM_BLE_DATA_LENGTH_UPDATE_EVENT:
            printf("BTM_BLE_DATA_LENGTH_UPDATE_EVENT, \r\n"
                    "Max tx octets is :%d ,\r\n"
                    "Max rx octets is :%d \r\n",
                    p_event_data->ble_data_length_update_event.max_tx_octets,
                    p_event_data->ble_data_length_update_event.max_rx_octets);
            break;

        case BTM_SECURITY_FAILED_EVT:
            /* Handle pairing Failure */
            printf("Pairing Failed\r\n");
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            printf("BTM_BLE_PHY_UPDATE_EVT,\r\n "
                    "PHY Tx value is: %d, \r\n"
                    "PHY Rx value is: %d \r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_DISABLED_EVT:
			/* Bluetooth Controller and Host Stack Disabled */
			printf("BTM_DISABLED_EVT\r\n");
			break;
        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_btm_event_name(event));
            break;
    }

    return wiced_result;
}

/**************************************************************************************************
* Function Name: le_app_init
***************************************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called from the BT
*   management callback once the LE stack enabled event (BTM_ENABLED_EVT) is triggered
*   This function is executed in the BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*  None
*
*************************************************************************************************/
void le_app_init(void)
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    printf("\n***********************************************\n");
    printf("**Please connect to \"XENSIV Gamepad\"\n");
    printf("***********************************************\n\n");

    /* Initialize the PWM used for IAS alert level LED */
    //cy_result = cyhal_pwm_init_adv(&ias_led_pwm, CYBSP_USER_LED1 , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != cy_result  )
    {
        printf("IAS LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }
    /* CYBSP_USER_LED2 is only present on some kits. For those kits,it is used to indicate advertising/connection status */
#ifdef CYBSP_USER_LED2
    /* Initialize the PWM used for Advertising LED */
    cy_result = cyhal_pwm_init_adv(&adv_led_pwm, CYBSP_USER_LED2 , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Advertisement LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }
#endif

    wiced_bt_set_pairable_mode(TRUE, FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(le_app_gatt_event_callback);
    printf("GATT event Handler registration status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != wiced_result)
    {
        printf("failed to start advertisement! \n");
        CY_ASSERT(0);
    }
}

/**************************************************************************************************
* Function Name: le_app_gatt_event_callback
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;

    uint16_t error_handle = 0;
    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    uint16_t id = p_event_data->connection_status.conn_id;

    //printf("GATT CONNECTION ID upon event %s: %i\r\n", get_bt_gatt_event_name(event), id);

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = le_app_connect_handler( &p_event_data->connection_status );
            //ble_conn_id = p_attr_req->conn_id;
            //printf("this is the gatt connection id: %i", ble_conn_id);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = le_app_server_handler(p_attr_req,
                                                &error_handle );
            if(gatt_status != WICED_BT_GATT_SUCCESS)
            {
               wiced_bt_gatt_server_send_error_rsp(p_attr_req->conn_id,
                                                   p_attr_req->opcode,
                                                   error_handle,
                                                   gatt_status);
            }
            break;

        case GATT_OPERATION_CPLT_EVT:
            printf("GATT_OPERATION_CPLT_EVT\r\n");
            break;

        case GATT_CONGESTION_EVT:
            num_of_congestions++;
            is_gatt_congested = (p_event_data->congestion.congested) ? true : false;
            printf("Gatt congestion event!\r\n");

            if (!is_gatt_congested)
            {
                xTaskNotify(ble_task_h, 0, eNoAction);
            }

            break;

        case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;


        default:
            gatt_status = WICED_BT_GATT_ERROR;
            printf("unknown GATT event: %04X\r\n", gatt_status);
               break;
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_set_value
***************************************************************************************************
* Summary:
*   This function handles writing to the attribute handle in the GATT database using the
*   data passed from the BT stack. The value to write is stored in a buffer
*   whose starting address is passed as one of the function parameters
*
* Parameters:
* @param attr_handle  GATT attribute handle
* @param p_val        Pointer to LE GATT write request value
* @param len          length of GATT write request
*
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_set_value(uint16_t attr_handle,
                                                uint8_t *p_val,
                                                uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is written.
                 * In this case, we update the IAS led based on the IAS alert
                 * level characteristic value */

                switch ( attr_handle )
                {
                    case HDLC_IAS_ALERT_LEVEL_VALUE:
                        printf("Alert Level = %d\n", app_ias_alert_level[0]);
                        //ias_led_update();
                        break;

                    /* The application is not going to change its GATT DB,
                     * So this case is not handled */
//                    case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
//                        gatt_status = WICED_BT_GATT_SUCCESS;
//                        break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within generated lookup table.
         * This is a custom logic that depends on the application, and is not used in the
         * current application. If the value for the current handle is successfully written in the
         * below code snippet, then set the result using:
         * res = WICED_BT_GATT_SUCCESS; */
        switch ( attr_handle )
        {
            default:
                /* The write operation was not performed for the indicated handle */
                printf("Write Request to Invalid Handle: 0x%x\n", attr_handle);
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_write_handler
***************************************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  @param conn_id       Connection ID
*  @param opcode        LE GATT request type opcode
*  @param p_write_req   Pointer to LE GATT write request
*  @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_write_req_t *p_write_req,
                                                    uint16_t len_req,
                                                    uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    *p_error_handle = p_write_req->handle;

    /* Attempt to perform the Write Request */
    gatt_status = le_app_set_value(p_write_req->handle,
                                p_write_req->p_val,
                               p_write_req->val_len);

    if( WICED_BT_GATT_SUCCESS != gatt_status )
    {
        printf("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }
    else
    {
        if(GATT_REQ_WRITE == opcode)
        wiced_bt_gatt_server_send_write_rsp(conn_id, opcode, p_write_req->handle);
    }
    return (gatt_status);
}

/**************************************************************************************************
* Function Name: le_app_read_handler
***************************************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* @param conn_id       Connection ID
* @param opcode        LE GATT request type opcode
* @param p_read_req    Pointer to read request containing the handle to read
* @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req,
                                                    uint16_t *p_error_handle)
{

    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    *p_error_handle = p_read_req->handle;

    puAttribute = le_app_find_by_handle(p_read_req->handle);
    if ( NULL == puAttribute )
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;
    if (p_read_req->offset >= puAttribute->cur_len)
    {
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;

    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */;
}

/**************************************************************************************************
* Function Name: le_app_connect_handler
***************************************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_connect_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS ;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            printf("Connected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            for(uint8_t i = 0; i<6; i++)
			{
				ble_conn_adr[i] = 	p_conn_status->bd_addr[i];
			}
            printf("Connection ID '%d' \n", p_conn_status->conn_id );
            ble_conn_id = p_conn_status->conn_id;

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            disconnect_reason = get_bt_gatt_disconn_reason_name(p_conn_status->reason);

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;
            ble_conn_id = 0;

            for(uint8_t i = 0; i<6; i++)
			{
				ble_conn_adr[i] = 	0x00;
			}

            /* Restart the advertisements */
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
			menu_connection_item_id_entered = MENU_CONNECTION_ITEM_DISCONNECTED;

            /* Turn Off the IAS LED on a disconnection */
            //ias_led_update();
        }

        /* Update Advertisement LED to reflect the updated state */
        //adv_led_update();

        gatt_status = WICED_BT_GATT_ERROR;
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_server_handler
***************************************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  p_attr_req     Pointer to LE GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
wiced_bt_gatt_status_t le_app_server_handler (wiced_bt_gatt_attribute_request_t *p_attr_req,
                                                      uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            gatt_status = le_app_read_handler(p_attr_req->conn_id,p_attr_req->opcode,
                                              &p_attr_req->data.read_req,
                                              p_attr_req->len_requested,
                                              p_error_handle);
             break;
        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
            gatt_status = le_app_write_handler(p_attr_req->conn_id,
                                               p_attr_req->opcode,
                                               &p_attr_req->data.write_req,
                                               p_attr_req->len_requested,
                                               p_error_handle );

             break;
        case GATT_REQ_MTU:
            gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;
        case GATT_HANDLE_VALUE_NOTIF:
                    //printf("Notfication send complete\n");
             break;
        case GATT_REQ_READ_BY_TYPE:
            gatt_status = app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                               p_attr_req->opcode,
                                                               &p_attr_req->data.read_by_type,
                                                               p_attr_req->len_requested,
                                                               p_error_handle);
             break;

        default:
                printf("ERROR: Unhandled GATT Connection Request case: %d\n", p_attr_req->opcode);
                gatt_status = WICED_BT_GATT_ERROR;
                break;
    }

    return gatt_status;
}


/*******************************************************************************
 * Function Name: app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 ******************************************************************************/
void app_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
 * Function Name: app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 *
 ******************************************************************************/
void* app_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}
/*******************************************************************************
 * Function Name : le_app_find_by_handle
 * *****************************************************************************
 * Summary : @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 ******************************************************************************/
gatt_db_lookup_table_t  *le_app_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle )
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}
/**
 * Function Name:
 * app_bt_gatt_req_read_by_type_handler
 *
 * Function Description:
 * @brief  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 */
wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested,
                                                                   uint16_t *p_error_handle)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\r\n",len_requested);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        *p_error_handle = attr_handle;
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle )
            break;

        if ( NULL == (puAttribute = le_app_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \r\n",last_handle);
            app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len, len_requested - used_len, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (0 == filled)
            {
                break;
            }
            used_len += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
       printf("attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\r\n",
               p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);
        app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */

    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used_len, p_rsp, (void *)app_free_buffer);
}

