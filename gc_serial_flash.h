

/******************************************************************************
* File Name: gc_serial_flash.h
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Funciton brief: init and read/write handler for writing key values to storage
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

#ifndef __GC_SERIAL_FLASH_H_
#define __GC_SERIAL_FLASH_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "mtb_kvstore.h"
#include "cycfg_qspi_memslot.h"
#include "cy_serial_flash_qspi.h"


/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/


/*******************************************************************************
 *                          Global Variables
 ******************************************************************************/
extern mtb_kvstore_t kvstore_obj;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void gc_flash_init();
cy_rslt_t gc_kvstore_write(const char *key, uint8_t *writeData, uint32_t data_size);
cy_rslt_t gc_kvstore_read(const char *key, uint8_t *readData, uint32_t *data_size);
cy_rslt_t gc_kvstore_delete(const char *key);

#endif //__GC_SERIAL_FLASH_H_
