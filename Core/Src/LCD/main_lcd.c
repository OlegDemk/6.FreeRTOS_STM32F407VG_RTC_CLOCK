/*
 * main_lcd.c
 *
 *  Created on: Jan 5, 2022
 *      Author: odemki
 */

#include "main.h"
#include "LCD/spi_ili9341.h"
#include "LCD/main_lcd.h"
#include <stdbool.h>

extern RNG_HandleTypeDef hrng;

// ---------------------------------------------------------------------------------
////////////////////////////// TEST function
/* 	Compare without DMA and with DMA
 1. Without DMA: 9 seconds
 2. With DMA 1.6 seconds
 Select neaded variant in functions: TFT9341_FillRect and TFT9341_FillScreen.

 */
void speed_test(void)
{
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
	int i = 0;

	// TEST 1
	for(i = 0; i < 10; i++)
	{
		// TEST 1fill all dysplay random colors
		TFT9341_FillScreen(TFT9341_RandColor());
	    //osDelay(200);
	}

	// TEST 2
	for(i = 0; i < 10; i++)
	{
		TFT9341_FillRect(0, 0, TFT9341_WIDTH/2-1, TFT9341_HEIGHT/2-1, TFT9341_RandColor());
		TFT9341_FillRect(TFT9341_WIDTH/2, 0, TFT9341_WIDTH-1, TFT9341_HEIGHT/2-1, TFT9341_RandColor());
		TFT9341_FillRect(0, TFT9341_HEIGHT/2, TFT9341_WIDTH/2-1, TFT9341_HEIGHT-1, TFT9341_RandColor());
		TFT9341_FillRect(TFT9341_WIDTH/2, TFT9341_HEIGHT/2, TFT9341_WIDTH-1, TFT9341_HEIGHT-1, TFT9341_RandColor());
	}

	// TEST 3
	TFT9341_FillScreen(TFT9341_BLACK);
	for(i = 0; i < 100; i++)
	{
		TFT9341_DrawLine(TFT9341_RandColor(),
		HAL_RNG_GetRandomNumber(&hrng)%TFT9341_WIDTH,
		HAL_RNG_GetRandomNumber(&hrng)%TFT9341_HEIGHT,
		HAL_RNG_GetRandomNumber(&hrng)%TFT9341_WIDTH,
		HAL_RNG_GetRandomNumber(&hrng)%TFT9341_HEIGHT);
	}
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);

	i = 0;
}
// ---------------------------------------------------------------------------------
void speed_test_print_string(void)
{
	TFT9341_String(10,25, "ROW 1 qwertyuiop1234567890");
	TFT9341_String(10,40, "ROW 2 qwertyuiop1234567890");
	TFT9341_String(10,55, "ROW 3 qwertyuiop1234567890");
	TFT9341_String(10,70, "ROW 4 qwertyuiop1234567890");
	TFT9341_String(10,85, "ROW 5 qwertyuiop1234567890");
	TFT9341_String(10,100, "ROW 6 qwertyuiop1234567890");
	TFT9341_String(10,115, "ROW 7 qwertyuiop1234567890");
	TFT9341_String(10,130, "ROW 8 qwertyuiop1234567890");
	TFT9341_String(10,145, "ROW 9 qwertyuiop1234567890");
	TFT9341_String(10,160, "ROW 10 qwertyuiop1234567890");
	TFT9341_String(10,175, "ROW 11 qwertyuiop1234567890");
	TFT9341_String(10,190, "ROW 12 qwertyuiop1234567890");
	TFT9341_String(10,205, "ROW 13 qwertyuiop1234567890");

	osDelay(1000);
	TFT9341_FillScreen(TFT9341_BLUE);

	TFT9341_String_DMA(10,25, "ROW 1 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,40, "ROW 2 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,55, "ROW 3 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,70, "ROW 4 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,85, "ROW 5 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,100, "ROW 6 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,115, "ROW 7 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,130, "ROW 8 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,145, "ROW 9 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,160, "ROW 10 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,175, "ROW 11 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,190, "ROW 12 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,205, "ROW 13 qwertyuiop12345 DMA");
	TFT9341_String_DMA(10,220, "ROW 14 qwertyuiop12345 DMA");

	osDelay(1000);
	TFT9341_FillScreen(TFT9341_BLUE);
}
// ---------------------------------------------------------------------------------

void LCD_print_page_1_sensors(void)
{
	static bool print_one_time = true;

	if(print_one_time == true)		// Print only one time
	{
		// Init names sensors
		TFT9341_String(5,25, "1. RTС ");
		TFT9341_String(5,40, "2. AM2302");
		TFT9341_String(5,55, "3. BME280");
		TFT9341_String(5,70, "4. MPU6060 Acc");
		TFT9341_String(5,85, "5. MPU6060 Gyro");
		TFT9341_String(5,100, "6. MPU6060 T");
		TFT9341_String(5,115, "7. L883");
		TFT9341_String(5,130, "8. BMP180");
		TFT9341_String(5,130, "8. APDS9960");
		TFT9341_String(5,130, "9. ADC");
	}

	// Waiting on queue RTC sensors
	// 1. RTC

	// Print data from RTS

	// 2. AM2302

	// Print data from AM2302

	// 3. BME280

	// Print data from BME280



}





