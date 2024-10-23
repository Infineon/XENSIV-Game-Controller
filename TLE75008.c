/***********************************************************************************************************************
* File-Name: TLE75008.c
* AUTHOR: Matthias Ernst
* Infineon Technologies AG
* Date: 08.10.2020
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
*/

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "TLE75008.h"
#include "stdlib.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "timers.h"

#include "gameController_boardControl.h"


/*******************************************************************************
**                      Private Variables                                              **
*******************************************************************************/


/*******************************************************************************
**                      Global Variables                                              **
*******************************************************************************/
/* PWM object */
cyhal_pwm_t pwm_rumble;

cyhal_spi_t mSPI;

uint16_t SPI_sendData;
uint32_t SPI_readData;

uint8_t TLE75008_mapin0_reg;
uint8_t TLE75008_out_reg;

TimerHandle_t LEDtimers[10];
uint8_t ledTimerID = 0;

uint8_t rumble_active = 1;

uint8_t led_toggle_flag = 0;

uint8_t led_activate_flag = 0;

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/
void spi_init(void)
{
	printf("SPI master initialization: ");
	cy_rslt_t   result=CY_RSLT_SUCCESS;
	uint32_t    spi_master_frequency = 1000000;

	result = cyhal_spi_init(&mSPI, CYBSP_SPI_MOSI, CYBSP_SPI_MISO, CYBSP_SPI_CLK, CYBSP_SPI_CS, NULL, 16, CYHAL_SPI_MODE_01_MSB, false);
	cyhal_spi_slave_select_config(&mSPI, CYBSP_SPI_CS, CYHAL_SPI_SSEL_ACTIVE_LOW);

	if (result != CY_RSLT_SUCCESS)
		{
			printf("init failed!\r\n");
			CY_ASSERT(0);
		}else{
			printf("Success! \r\n");
		}
	result = cyhal_spi_set_frequency(&mSPI, spi_master_frequency);

}



void TLE75008_init()
{
	cy_rslt_t result;

	printf("SPIDER+ initialization: ");

	/*Set IDLE pin to high */
	cyhal_gpio_init(PIN_SPIDER_IDLE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_write(PIN_SPIDER_IDLE, true);


//	cyhal_gpio_init(CYBSP_SPI_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
//	cyhal_gpio_write(CYBSP_SPI_CS, false);

	SPI_sendData = 0x8C80; 				//activate active mode HWCR.act 0x0C:(7)
	result = cyhal_spi_send(&mSPI, SPI_sendData);
	if (result != CY_RSLT_SUCCESS)
	{
		printf("SPI com failed!");
		CY_ASSERT(0);
	}
	SPI_sendData = 0x8400;				//MAPIN0 all channels off
	result = cyhal_spi_send(&mSPI, SPI_sendData);
	SPI_sendData = 0x8500;				//MAPIN1 all channels off
	cyhal_spi_send(&mSPI, SPI_sendData);
	SPI_sendData = 0x0001;				//standard diagnosis
	result = cyhal_spi_send(&mSPI, SPI_sendData);
	if (result == CY_RSLT_SUCCESS)
	{
		printf("Success!\r\n");
	}
	//TLE75008_writeRegister(TLE75008_MAPIN0_ADDR, RUMBLE_LEFT|RUMBLE_RIGHT2);
	TLE75008_initPWM();
	TLE75008_startPWM(PWM_FREQUENCY, 25);

	TLE75008_mapin0_reg = 0;
	TLE75008_out_reg = 0;
}


void TLE75008_writeRegister(uint8 reg, uint8 data)
{
	uint16_t transfer = (uint16)((reg|(uint16)0x80)<<8);
	transfer = transfer|(uint16)(data);
	cyhal_spi_send(&mSPI, transfer);
}


void TLE75008_initPWM()
{
	cy_rslt_t result;
	result = cyhal_pwm_init(&pwm_rumble, PIN_RUMBLE_PWM, NULL);
	result = cyhal_pwm_set_duty_cycle(&pwm_rumble, 0, PWM_FREQUENCY);
}


void TLE75008_startPWM(uint32_t frequency, float dutyCycle)
{
	cyhal_pwm_set_duty_cycle(&pwm_rumble, dutyCycle, frequency);
	cyhal_pwm_start(&pwm_rumble);
}


void TLE75008_deactivatePWM()
{
	cyhal_pwm_stop(&pwm_rumble);
}

void rumble_activate(uint8_t rumble, float dutyCycle)
{
	if(rumble_active)
	{
		if(rumble==1||rumble==2||rumble==3){
			TLE75008_writeRegister(TLE75008_OUT_ADDR, rumble);
		}else{
			printf("Unknown Rumble number. \r\n");
			return;
		}
	}
}

void rumble_deactivate()
{
	//cyhal_pwm_stop(&pwm_rumble);
	TLE75008_writeRegister(TLE75008_OUT_ADDR, TLE75008_OUT_OUTn_OFF);
	//TLE75008_deactivatePWM();
}

void buttonPress_haptic_feedback()
{
	rumble_activate(RUMBLE_RIGHT2|RUMBLE_LEFT, 70);
	cyhal_system_delay_ms(25);
	rumble_deactivate();
}

void TLE75008_LED_activate(uint8_t LED)
{
	uint8_t led_activate = LED;
	led_activate&=LED_OUT_MASK;		//check that only leds (OUT2 - OUT6) are in argument
	TLE75008_mapin0_reg |= led_activate;		//write OUT register
	cyhal_pwm_start(&pwm_rumble);
	TLE75008_writeRegister(TLE75008_MAPIN0_ADDR, TLE75008_mapin0_reg);
}

void TLE75008_LED_deactivate(uint8_t LED)
{
	TLE75008_LED_toggle_ms_stop(LED);
	int8_t led_activate = LED;
	led_activate ^= 0xFF;
	led_activate &= LED_OUT_MASK;		//check that only leds (OUT2 - OUT6) are in argument
	TLE75008_mapin0_reg &= led_activate;		//write OUT register
	TLE75008_writeRegister(TLE75008_MAPIN0_ADDR, TLE75008_mapin0_reg);
}

void TLE75008_LED_toggle(uint8_t LED)
{
	int8_t led_toggle = LED;
	led_toggle &= LED_OUT_MASK;		//check that only leds (OUT2 - OUT6) are in argument
	TLE75008_mapin0_reg ^= led_toggle;
	TLE75008_writeRegister(TLE75008_MAPIN0_ADDR, TLE75008_mapin0_reg);
}

void ledToggleTimerCallback(TimerHandle_t timerHandle)
{
	uint8_t led = 0;
	ledTimerID = pvTimerGetTimerID(timerHandle);
	switch(ledTimerID)
	{
		case 0:
			led =LED_BUTTONS_RIGHT;
			break;
		case 1:
			led = LED_BUTTONS_LEFT;
			break;
		case 2:
			led = LED_HOME_BLUE;
			break;
		case 3:
			led = LED_HOME_GREEN;
			break;
		case 4:
			led = LED_HOME_RED;
			break;
		default:
			break;
	}
	led_toggle_flag = led;
	//TLE75008_LED_toggle(led);
}

void TLE75008_LED_toggle_ms(uint8_t LED, uint32_t ms)
{
	int8_t led_toggle = LED;
	led_toggle &= LED_OUT_MASK;

	switch(led_toggle)
	{
		case LED_BUTTONS_RIGHT:
			ledTimerID = 0;
			break;
		case LED_BUTTONS_LEFT:
			ledTimerID = 1;
			break;
		case LED_HOME_BLUE:
			ledTimerID = 2;
			break;
		case LED_HOME_GREEN:
			ledTimerID = 3;
			break;
		case LED_HOME_RED:
			ledTimerID = 4;
			break;
		default:
			printf("LED toggle timer error: only choose one led at a time!\r\n");
			break;
	}
	LEDtimers[ledTimerID] = xTimerCreate("LED toggle timer", ms*portTICK_PERIOD_MS, pdTRUE, (void *) ledTimerID, ledToggleTimerCallback);
	if( LEDtimers[ledTimerID] == NULL )
	{
		printf("Led toggle timer not created\r\n");
	}
        else
    {
        printf("Led toggle timer created\r\n");
    }
	xTimerStart(LEDtimers[ledTimerID], 0);

}

void TLE75008_LED_toggle_ms_stop(uint8_t LED)
{
	int8_t led_toggle = LED;
	led_toggle &= LED_OUT_MASK;

	switch(led_toggle)
	{
		case LED_BUTTONS_RIGHT:
			ledTimerID = 0;
			break;
		case LED_BUTTONS_LEFT:
			ledTimerID = 1;
			break;
		case LED_HOME_BLUE:
			ledTimerID = 2;
			break;
		case LED_HOME_GREEN:
			ledTimerID = 3;
			break;
		case LED_HOME_RED:
			ledTimerID = 4;
			break;
		default:
			printf("LED toggle timer error: only choose one led at a time!\r\n");
			break;
	}
	if( LEDtimers[ledTimerID] != NULL )
	{
		xTimerStop(LEDtimers[ledTimerID], 0);
	}

}



void ledActivateTimerCallback(TimerHandle_t timerHandle)
{
	uint8_t led = 0;
	ledTimerID = pvTimerGetTimerID(timerHandle);
	switch(ledTimerID)
	{
		case 5:
			led = LED_BUTTONS_RIGHT;
			break;
		case 6:
			led = LED_BUTTONS_LEFT;
			break;
		case 7:
			led = LED_HOME_BLUE;
			break;
		case 8:
			led = LED_HOME_GREEN;
			break;
		case 9:
			led = LED_HOME_RED;
			break;
		default:
			break;
	}
	led_activate_flag = led;
	//TLE75008_LED_deactivate(led);
}



void TLE75008_LED_activate_ms(uint8_t LED, uint32_t ms)
{
	int8_t led_activate = LED;
	led_activate &= LED_OUT_MASK;

	switch(led_activate)
	{
		case LED_BUTTONS_RIGHT:
			ledTimerID = 5;
			break;
		case LED_BUTTONS_LEFT:
			ledTimerID = 6;
			break;
		case LED_HOME_BLUE:
			ledTimerID = 7;
			break;
		case LED_HOME_GREEN:
			ledTimerID = 8;
			break;
		case LED_HOME_RED:
			ledTimerID = 9;
			break;
		default:
			printf("LED activate timer error: only choose one led at a time!\r\n");
			break;
	}
	LEDtimers[ledTimerID] = xTimerCreate("LED activate timer", ms*portTICK_PERIOD_MS, pdFALSE, (void *) ledTimerID, ledActivateTimerCallback);
	if( LEDtimers[ledTimerID] == NULL )
	{
		printf("Led toggle timer not created\r\n");
	}
		else
	{
		printf("Led toggle timer created\r\n");
	}
	TLE75008_LED_activate(LED);
	xTimerStart(LEDtimers[ledTimerID], 0);
}
