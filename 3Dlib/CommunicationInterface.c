/******************************************************************************
* File Name: CommunicationInterface.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This file defines the com interface for the 3D C library.
*
*
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "CommunicationInterface.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "i2c.h"



/*******************************************************************************
* Macros
*******************************************************************************/



/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

bool tlx493d_transfer(TLx493D_t *sensor, uint8_t txBuffer[], uint8_t txLen, uint8_t rxBuffer[], uint8_t rxLen)
{
	cy_rslt_t cy_result;

	//uint8_t iicAddress = 0x35;//sensor->comInterface.comLibParams.iic_params.address;
	uint8_t iicAddress= sensor->comInterface.comLibParams.iic_params.address>>1;
	uint8_t iicAddress_test2 =sensor->regDef->address;

	uint8_t transfer[txLen];
	for(uint8_t i = 0; i<txLen; i++)
	{
		transfer[i]=txBuffer[i];
	}

	if((txBuffer!=NULL)&&(txLen>0))
	{
		cy_result = cyhal_i2c_master_write(&mI2C, iicAddress, transfer, txLen, 100, true);
		if (cy_result != CY_RSLT_SUCCESS)
			{
				printf("Writing to tlx493d at 0x%02x failed!\r\n", iicAddress);
				return false;
			}
		return true;
	}
	else if((rxBuffer!=NULL)&&(rxLen>0))
	{
		cy_result = cyhal_i2c_master_read(&mI2C, iicAddress, rxBuffer, rxLen, 3, true);
		if (cy_result != CY_RSLT_SUCCESS)
		{
			printf("Reading from tlx493d at 0x%02x failed!\r\n", iicAddress);
			return false;
		}
		return true;
	}
	else
	{
		printf("tlx493d transfer failed due to unexpected buffer type\r\n");
		CY_ASSERT(0);
		return false;
	}

}
