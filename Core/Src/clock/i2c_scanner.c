/*
 * i2c_scanner.c
 *
 *  Created on: Apr 28, 2021
 *      Author: odemki
 */


#include "stdio.h"
//#include "stm32f4xx_hal.h"
#include "main.h"
#include "clock/i2c_scanner.h"

#define TRANSMIT_IN_COMPORT 1

#define DEVICE_FOUND 0

#define OLED_SSD136_I2C_ADDR 0x3C

//extern UART_HandleTypeDef huart1;      			// If nead print information on comport
extern I2C_HandleTypeDef hi2c3;


uint8_t addres_devise=0x00;      //ADRESS_MPU6050=0x68; -> return 0  ,   DRESS_MS5611=0x77;-> return 0
uint8_t addr=0;
uint16_t sizebuf_I2C=1;								// size how many data we receive from devise
uint8_t buff=0;										// data for receive
uint32_t timeout=1000;						        // timeout for receive
uint16_t STATUS=0;								    // Status connect to device (if STATUS==0 - device

struct i2c_devices 				// <<<<<<<<<<<<<<<<<<<<     Fill STATUS devices
{

};

//----------------------------------------------------------------------------------------------------
void I2C_3_scaner(void)
{
	/*Description function
	This function search devise connected to I2C in this case -hi2c1.
	After thet function print in console information about what to connect to I2C.
	*/
	uint8_t number_of_device=0;				// How many device controller is found

	//HAL_Delay(500);

	for(addres_devise=0x00; addres_devise<0xFF; addres_devise++)  // addres_devise<0x7F
	{
		HAL_Delay(1);
		STATUS = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)addres_devise<<1,(uint16_t)addr, (uint16_t) sizebuf_I2C, &buff, (uint16_t) sizebuf_I2C,(uint32_t) timeout);

		if(STATUS == DEVICE_FOUND)																		// if devsice is found
		{
			number_of_device++;

		}
		else
		{

		}
	}

	if(number_of_device==0)  																				// If devices nofound
	{

	}

	osDelay(500);
}
