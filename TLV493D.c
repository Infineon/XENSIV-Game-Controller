/******************************************************************************
* File Name: tlv493d.c
*
* AUTHOR: Matthias Ernst
*
* Infineon Technologies AG
*
* Description: This is the tlv493d file.
*
* Related Document: See README.md
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

#include "TLV493D.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "i2c.h"

#include "gameController_boardControl.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/



/*******************************************************************************
* Function Definitions
*******************************************************************************/

uint8_t TLV493D_init(TLx493D_t *sensor, cyhal_gpio_t powerPin, uint8_t address)
{
	uint8_t TLV493D_write_data[2];
	uint8_t TLV493D_id = 0;
	uint8_t error_flag = 0;

	printf("TLV493D initialization at 0x%02x: ", address>>1);

	cyhal_gpio_init(powerPin, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_write(powerPin, true);

	/*The setDefaultConfig(sensor) can be set once the 3D library has been published for mtb*/
	/*Until then, we use the function void TLV493D_changeIICAddr(TLx493D_t sensor, uint8_t address)*/
	//TLx493D_A2B6_setDefaultConfig(sensor);
	//TLx493D_A2B6_setIICAddress(sensor, TLx493D_IIC_ADDR_A1_e)

	TLx493D_A2B6_init(sensor);

	cyhal_system_delay_ms(5);

	TLV493D_write_data[0] = TLV493D_CONFIG_REG;
	TLV493D_write_data[1] = 0x18;	//set ADC trigger on read before first MSB, x2 sensitivity
	cyhal_i2c_master_write(&mI2C, 0x35, TLV493D_write_data, 2, 2, true);
	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&mI2C, 0x35, TLV493D_write_data, 2, 2, true))
	{
		printf("Config register failed! ");
		error_flag = 1;
	}


	uint8_t mod1_reg_value = 0x91;	//one byte read protocol, set master controlled mode, addr = 0x35
	switch (address)
	{
		case TLV493D_IIC_ADDR_A0:
			mod1_reg_value = 0x91;
			TLV493D_id = 0;
			break;
		case TLV493D_IIC_ADDR_A1:
			mod1_reg_value = 0x31;
			TLV493D_id = 1;
			break;
		case TLV493D_IIC_ADDR_A2:
			mod1_reg_value = 0x51;
			TLV493D_id = 2;
			break;
		case TLV493D_IIC_ADDR_A3:
			mod1_reg_value = 0xF1;
			TLV493D_id = 3;
			break;
		default:
			printf("Undefined address! ");
	}
	TLV493D_write_data[0] = TLV493D_MOD1_REG;
	TLV493D_write_data[1] = mod1_reg_value;
	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&mI2C, 0x35, TLV493D_write_data, 2, 3, true))
	{
		printf("MOD1 register failed!\r\n");
		error_flag = 1;
	}
	tlx493d_common_setIICAddress(sensor, address);
	TLx493D_A2B6_readRegisters(sensor);
	uint8_t idCheck = sensor->regMap[5];
	if((idCheck==0)||(TLV493D_id!=((idCheck&=0x30)>>4)))
	{
		printf("IIC addr mod failed!\r\n");
		error_flag = 1;
	}
	else
	{
		printf("Success!\r\n");
	}
	return error_flag;
}

uint8_t TLV493D_deinit(TLx493D_t *sensor, cyhal_gpio_t powerPin, uint8_t address)
{
	cyhal_gpio_write(powerPin, false);
	printf("TLV493D De-initialization at 0x%02x ", address>>1);
	if(TLx493D_A2B6_deinit(sensor))
	{
		return 1;
		printf("de-initialized!\r\n");
	}
	else
	{
		printf("error!\r\n");
		return 0;
	}
}

void TLV493D_setShortRangeSensitivity(TLx493D_t *sensor)
{
	if(TLx493D_A2B6_setSensitivity(sensor, TLx493D_EXTRA_SHORT_RANGE_e)!=true)
	{
		printf("XShort range reg failed!\r\n");
	}
//	uint8_t TLV493D_write_data[2];
//	uint8_t address = sensor->comInterface.comLibParams.iic_params.address>>1;
//	uint8_t config2reg;
//	config2reg = sensor->regMap[14];
//	config2reg|=0x01;
//	sensor->regMap[14]|=0x01;
//	TLV493D_write_data[0] = TLV493D_CONFIG2_REG;
//	TLV493D_write_data[1] = config2reg;	//x4 sensitivity
//	cyhal_i2c_master_write(&mI2C, address, TLV493D_write_data, 2, 2, true);
//	if (CY_RSLT_SUCCESS != cyhal_i2c_master_write(&mI2C, 0x35, TLV493D_write_data, 2, 2, true))
//	{
//		printf("Config2 register failed! ");
//	}
}


void TLV493D_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z)
{
	TLx493D_A2B6_getRawMagneticField(sensor, x, y, z);

}
