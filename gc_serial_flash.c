/******************************************************************************
 * File Name: gc_serial_flash.c
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/

#include <gc_serial_flash.h>
#include "cycfg_qspi_memslot.h"
#include "mtb_kvstore.h"

#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal.h"

/*******************************************************************************
 *                              Macro Definitions
 ******************************************************************************/


/*******************************************************************************
 *                              GLOBAL DECLARATIONS
 ******************************************************************************/

mtb_kvstore_t kvstore_obj;


/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

//--------------------------------------------------------------------------------------------------
// bd_read_size
//--------------------------------------------------------------------------------------------------
static uint32_t bd_read_size(void* context, uint32_t addr)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(addr);
    return 1;
}


//--------------------------------------------------------------------------------------------------
// bd_program_size
//--------------------------------------------------------------------------------------------------
static uint32_t bd_program_size(void* context, uint32_t addr)
{
    CY_UNUSED_PARAMETER(context);
    return cy_serial_flash_qspi_get_prog_size(addr);
}


//--------------------------------------------------------------------------------------------------
// bd_erase_size
//--------------------------------------------------------------------------------------------------
static uint32_t bd_erase_size(void* context, uint32_t addr)
{
    CY_UNUSED_PARAMETER(context);
    return cy_serial_flash_qspi_get_erase_size(addr);
}


//--------------------------------------------------------------------------------------------------
// bd_read
//--------------------------------------------------------------------------------------------------
static cy_rslt_t bd_read(void* context, uint32_t addr, uint32_t length, uint8_t* buf)
{
    CY_UNUSED_PARAMETER(context);
    return cy_serial_flash_qspi_read(addr, length, buf);
}


//--------------------------------------------------------------------------------------------------
// bd_program
//--------------------------------------------------------------------------------------------------
static cy_rslt_t bd_program(void* context, uint32_t addr, uint32_t length, const uint8_t* buf)
{
    CY_UNUSED_PARAMETER(context);
    return cy_serial_flash_qspi_write(addr, length, buf);
}


//--------------------------------------------------------------------------------------------------
// bd_erase
//--------------------------------------------------------------------------------------------------
static cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length)
{
    CY_UNUSED_PARAMETER(context);
    return cy_serial_flash_qspi_erase(addr, length);
}


static mtb_kvstore_bd_t block_device =
{
    .read         = bd_read,
    .program      = bd_program,
    .erase        = bd_erase,
    .read_size    = bd_read_size,
    .program_size = bd_program_size,
    .erase_size   = bd_erase_size,
    .context      = NULL
};


void gc_flash_init()
{
	const uint32_t qspi_bus_freq_hz = 50000000lu;
	//printf("kv-store/flash initialization...\r\n ");
	cy_rslt_t result = cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0,
												 CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC,
												 NC, NC, NC,
												 CYBSP_QSPI_SCK, CYBSP_QSPI_SS, qspi_bus_freq_hz);
	if (CY_RSLT_SUCCESS != result)
	{
		printf("kv-store/flash initialization: qspi failed!\r\n");
		CY_ASSERT(0);
	}

	uint32_t sector_size = cy_serial_flash_qspi_get_erase_size(0);
	sector_size = (size_t)smifBlockConfig.memConfig[0]->deviceCfg->eraseSize;
	uint32_t length = sector_size * 2;
	uint32_t start_addr = smifMemConfigs[0]->deviceCfg->memSize - sector_size * 2;


	result = mtb_kvstore_init(&kvstore_obj, start_addr, length, &block_device);
	if (CY_RSLT_SUCCESS != result)
	{
		printf("kv-store/flash initialization: failed!\r\n");
		CY_ASSERT(0);
	}
	else
	{
		printf("kv-store/flash initialization: Success!\r\n");
	}
}


cy_rslt_t gc_kvstore_write(const char *key, uint8_t *writeData, uint32_t data_size)
{
	/*get data_size out of *data_size=sizeof(data)*/
	return mtb_kvstore_write(&kvstore_obj, key, writeData, data_size);
}

cy_rslt_t gc_kvstore_read(const char *key, uint8_t *readData, uint32_t *data_size)
{
	return mtb_kvstore_read(&kvstore_obj, key, readData, data_size);
}

cy_rslt_t gc_kvstore_delete(const char *key)
{
	return mtb_kvstore_delete(&kvstore_obj, key);
}
