/*
***********************************************************************************************************************
*
* File-Name: TLE75008.h
* AUTHOR: Matthias Ernst
* Infineon Technologies AG
* Date: 14.10.2020
*
* This file is for Control of TLE75008 Power Driver
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
* */

#ifndef TLE75008_H_
#define TLE75008_H_

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <inttypes.h>

/******************************************************************************
**                      Global Macro Definitions                             **
*******************************************************************************/
/*Addresses of control registers for TLE75008-ESD
8 address bits 			7	6	5	4	3	2	1	0				8 content bits		7	6	5	4	3	2	1	0
				write:			1 0	a	a	a	a	b	b								write:		c c c c c c c c
										a = ADDR0 b = ADDR1						 					c = reg content
				 read:			0 1 a a a a b b 						 	 read:		x x x x x x 1 0
																														x = don't care
																												//(obaaaabb)*/
#define TLE75008_OUT_ADDR									(0x00U)				//(0b000000)			//Power output control register; 0: OFF, 1: ON
#define TLE75008_MAPIN0_ADDR			 				(0x04U)				//(0b000100)			//IN0 mapping, output OUTn is connected to input pin 0
#define TLE75008_MAPIN1_ADDR							(0x05U)				//(0b000101)			//IN1 mapping, output OUTn is connected to input pin 1
#define TLE75008_INST_ADDR								(0x06U)				//(0b000110)			//READ ONLY, Input status Monitor: TER(bit7), reserved(6:2), INn(1:0)
#define TLE75008_DIAG_IOL_ADDR						(0x08U)				//(0b001000)			//Open load diagnostic current control OUTn; 0: disabled, 1: diagnosis enabled
#define TLE75008_DIAG_OSM_ADDR						(0x09U)				//(0b001001)			//READ ONLY, Output status Monitor: OSM.OUTn=1 if Vds<Vds(ol)
//#define TLE75008_DIAG_OSM		read only
#define TLE75008_HWCR_ADDR								(0x0CU)				//(0b001100)			//Hardware configuration, see control register bits
#define TLE75008_HWCR_OCL_ADDR						(0x0DU)				//(0b001101)			//Output clear latch, clear error latch for selected output OUTn

/*control register bits*/
/* =======================================================	OUT.OUTn  	======================================================== */
#define TLE75008_OUT_OUTn_OFF							(0x00U)
#define TLE75008_OUT_OUT0									(0x01U)
#define TLE75008_OUT_OUT1									(0x02U)
#define TLE75008_OUT_OUT2									(0x04U)
#define TLE75008_OUT_OUT3									(0x08U)
#define TLE75008_OUT_OUT4									(0x10U)
#define TLE75008_OUT_OUT5									(0x20U)
#define TLE75008_OUT_OUT6									(0x40U)
#define TLE75008_OUT_OUT7									(0x80U)

/* ======================================================	  MAPIN0.OUTn	  ======================================================= */
#define TLE75008_MAPIN0_OUTn_OFF					(0x00U)
#define TLE75008_MAPIN0_OUT0							(0x01U)
#define TLE75008_MAPIN0_OUT1							(0x02U)
#define TLE75008_MAPIN0_OUT2							(0x04U)
#define TLE75008_MAPIN0_OUT3							(0x08U)
#define TLE75008_MAPIN0_OUT4							(0x10U)
#define TLE75008_MAPIN0_OUT5							(0x20U)
#define TLE75008_MAPIN0_OUT6							(0x40U)
#define TLE75008_MAPIN0_OUT7							(0x80U)

/* ======================================================	  MAPIN1.OUTn	  ======================================================= */
#define TLE75008_MAPIN1_OUTn_OFF					(0x00U)
#define TLE75008_MAPIN1_OUT0							(0x01U)
#define TLE75008_MAPIN1_OUT1							(0x02U)
#define TLE75008_MAPIN1_OUT2							(0x04U)
#define TLE75008_MAPIN1_OUT3							(0x08U)
#define TLE75008_MAPIN1_OUT4							(0x10U)
#define TLE75008_MAPIN1_OUT5							(0x20U)
#define TLE75008_MAPIN1_OUT6							(0x40U)
#define TLE75008_MAPIN1_OUT7							(0x80U)

/* ======================================================	  DIAG_IOL.OUTn  ======================================================= */
#define TLE75008_DIAG_IOL_OUTn_disable		(0x00U)
#define TLE75008_DIAG_IOL_OUT0						(0x01U)
#define TLE75008_DIAG_IOL_OUT1						(0x02U)
#define TLE75008_DIAG_IOL_OUT2						(0x04U)
#define TLE75008_DIAG_IOL_OUT3						(0x08U)
#define TLE75008_DIAG_IOL_OUT4						(0x10U)
#define TLE75008_DIAG_IOL_OUT5						(0x20U)
#define TLE75008_DIAG_IOL_OUT6						(0x40U)
#define TLE75008_DIAG_IOL_OUT7						(0x80U)

/* ======================================================	 			HWCR.x	 	 ======================================================= */
#define TLE75008_HWCR_ACT									(0x80U)				//bit 7; 0: normal; 1: Device enters active mode
#define TLE75008_HWCR_RST									(0x40U)				//bit 6; 0: normal operation; 1: Execute reset command
//#define TLE75008_HWCR_5/4								(0xXXX)				//bit 5, bit 4 reserved
#define TLE75008_HWCR_PAR_CH02						(0x01U)				//bit 0; parallel operation, ***	0 = normal operation					***
#define TLE75008_HWCR_PAR_CH13						(0x02U)				//bit 1; parallel operation, ***	1 = two neighbour channels/		***
#define TLE75008_HWCR_PAR_CH46						(0x04U)				//bit 2; parallel operation, ***	have Over load and Over/ 			***
#define TLE75008_HWCR_PAR_CH57						(0x08U)				//bit 3; parallel operation, ***	over temperature synchronized	***

/* ======================================================	 		HWCR_OCL.x  	======================================================= */
#define TLE75008_HWCR_OCL_ALL							(0xFFU)
#define TLE75008_HWCR_OCL_OUT0						(0x01U)
#define TLE75008_HWCR_OCL_OUT1						(0x02U)
#define TLE75008_HWCR_OCL_OUT2						(0x04U)
#define TLE75008_HWCR_OCL_OUT3						(0x08U)
#define TLE75008_HWCR_OCL_OUT4						(0x10U)
#define TLE75008_HWCR_OCL_OUT5						(0x20U)
#define TLE75008_HWCR_OCL_OUT6						(0x40U)
#define TLE75008_HWCR_OCL_OUT7						(0x80U)

/* PWM Frequency = 10Hz */
#define PWM_FREQUENCY (1000u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (50.0f)

#define RUMBLE_RIGHT2					TLE75008_MAPIN0_OUT0
#define RUMBLE_RIGHT 					RUMBLE_RIGHT2
#define RUMBLE_LEFT						TLE75008_MAPIN0_OUT1
#define RUMBLE_LEFT2					RUMBLE_LEFT

#define LED_BUTTONS_RIGHT				TLE75008_MAPIN0_OUT2
#define LED_BUTTONS_LEFT				TLE75008_MAPIN0_OUT3
#define LED_HOME_BLUE					TLE75008_MAPIN0_OUT4
#define LED_HOME_GREEN					TLE75008_MAPIN0_OUT5
#define LED_HOME_RED					TLE75008_MAPIN0_OUT6

#define LED_OUT_MASK					(0x7C)




/******************************************************************************
**                      Global Variables Definitions                         **
*******************************************************************************/

extern uint8_t rumble_active;

extern uint8_t led_toggle_flag;

extern uint8_t led_activate_flag;

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/



/** \brief initializes TLE75008:
			initialize Tx_Buffer according to length of daisy chain
			set all TLE75008 (lumbar and massage) in active mode -> avoid limphome mode
			set all TLE75008 input mapping to 0 (lumbar and massage) -> default inputs deactivated
			trigger standard diagnosis from all TLE75008s
			IMPORTANT: Start CCU6_T12 after TLE75008_init() or deactivate pinout immediatly. otherwise limp home will be active
 *
 * \param none
 * \return none
 *
 * \brief <b>Example</b><br>
 * \brief This example initializes TLE75008
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *   TLE75008_Init(void);
 *   CCU6_T12_Start();
 *
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup
 */
void TLE75008_init(void);



/** \brief 	writes commands to the Tx Buffer of the corresponding daisy chain, w/o sending it already!
* \param   TLEs from the TLE75008_VALVES_x_y_z list, address from TLE75008 address list,
					 data from the TLE75008 data lists, SPI_CHIPSELECT_MASSAGE or lumbar
					 for further information on mapping of valves please refer to .brd data of SeatComfort ECU
 * \return none
 *
 * \brief <b>Example</b><br>
 * \brief This example activates Valve 6B
 * ~~~~~~~~~~~~~~~{.c}
 * void Example_Function(void)
 * {
 *		TLE75008_writeTxBuffer(TLE75008_VALVES_5_8_M, TLE75008_MAPIN0_ADDR, TLE75008_MAPIN0_OUT2, SPI_CHIPSELECT_MASSAGE);
			SPI_exchange(SPI_CHIPSELECT_MASSAGE);
 * }
 * ~~~~~~~~~~~~~~~
 * \ingroup
 */
void TLE75008_writeRegister(uint8 reg, uint8 data);



/***************to be implemented***************/
uint16 TLE75008_readRegister(uint16 TLE, uint8 address, uint16 chipSelect);
void TLE75008_SetDefault(uint16 SPI_chipSelect);

void TLE75008_initPWM(void);

void TLE75008_startPWM(uint32_t frequency, float dutyCycle);

void TLE75008_deactivatePWM(void);

void spi_init(void);

void rumble_activate(uint8_t rumble, float dutyCycle);

void rumble_deactivate();

void buttonPress_haptic_feedback();

void TLE75008_LED_activate(uint8_t LED);

void TLE75008_LED_deactivate(uint8_t LED);

void TLE75008_LED_toggle(uint8_t LED);

void TLE75008_LED_toggle_ms(uint8_t LED, uint32_t ms);

void TLE75008_LED_toggle_ms_stop(uint8_t LED);

void TLE75008_LED_activate_ms(uint8_t LED, uint32_t ms);

#endif
