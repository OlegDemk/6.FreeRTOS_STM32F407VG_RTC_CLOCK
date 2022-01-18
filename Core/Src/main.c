/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "usbd_cdc_if.h"
#include "task.h"
#include <stdio.h>
#include <stdbool.h>

// SD
#include "LCD/spi_ili9341.h"
#include "LCD/ILI9341_Touchscreen.h"

// Oled
#include "oled/oled.h"
#include "oled/gfx.h"

#include "clock/i2c_scanner.h"
#include "clock/DS3231.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct 							// Queue for UARD
{
	char Buf[1024];
}QUEUE_t;

typedef struct							// Queue for Touch LCD
{
	char buff[100];
}LCDQUEUE;



volatile unsigned long ulHighFreqebcyTimerTicks;		// This variable using for calculate how many time all tasks was running.
char str_management_memory_str[1000] = {0};
int freemem = 0;


// LCD //////////////////////////////////
extern uint16_t TFT9341_WIDTH;
extern uint16_t TFT9341_HEIGHT;
// LCD DMA
uint8_t dma_spi_fl=0;
uint32_t dma_spi_cnt=1;
////////////////////////////////////////

// Encoder
int32_t currCounter = 0;
int32_t prevCounter = 0;
int klick = 0;





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Show_Resources */
osThreadId_t Show_ResourcesHandle;
uint32_t Show_ResourcesBuffer[ 500 ];
osStaticThreadDef_t Show_ResourcesControlBlock;
const osThreadAttr_t Show_Resources_attributes = {
  .name = "Show_Resources",
  .cb_mem = &Show_ResourcesControlBlock,
  .cb_size = sizeof(Show_ResourcesControlBlock),
  .stack_mem = &Show_ResourcesBuffer[0],
  .stack_size = sizeof(Show_ResourcesBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
uint32_t UART_TaskBuffer[ 500 ];
osStaticThreadDef_t UART_TaskControlBlock;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .cb_mem = &UART_TaskControlBlock,
  .cb_size = sizeof(UART_TaskControlBlock),
  .stack_mem = &UART_TaskBuffer[0],
  .stack_size = sizeof(UART_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
uint32_t LCDBuffer[ 1500 ];
osStaticThreadDef_t LCDControlBlock;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .cb_mem = &LCDControlBlock,
  .cb_size = sizeof(LCDControlBlock),
  .stack_mem = &LCDBuffer[0],
  .stack_size = sizeof(LCDBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LCD_touchscreen */
osThreadId_t LCD_touchscreenHandle;
uint32_t LCD_touchscreenBuffer[ 128 ];
osStaticThreadDef_t LCD_touchscreenControlBlock;
const osThreadAttr_t LCD_touchscreen_attributes = {
  .name = "LCD_touchscreen",
  .cb_mem = &LCD_touchscreenControlBlock,
  .cb_size = sizeof(LCD_touchscreenControlBlock),
  .stack_mem = &LCD_touchscreenBuffer[0],
  .stack_size = sizeof(LCD_touchscreenBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RTC */
osThreadId_t RTCHandle;
uint32_t RTCBuffer[ 256 ];
osStaticThreadDef_t RTCControlBlock;
const osThreadAttr_t RTC_attributes = {
  .name = "RTC",
  .cb_mem = &RTCControlBlock,
  .cb_size = sizeof(RTCControlBlock),
  .stack_mem = &RTCBuffer[0],
  .stack_size = sizeof(RTCBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTQueue */
osMessageQueueId_t UARTQueueHandle;
uint8_t UARTQueueBuffer[ 10 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t UARTQueueControlBlock;
const osMessageQueueAttr_t UARTQueue_attributes = {
  .name = "UARTQueue",
  .cb_mem = &UARTQueueControlBlock,
  .cb_size = sizeof(UARTQueueControlBlock),
  .mq_mem = &UARTQueueBuffer,
  .mq_size = sizeof(UARTQueueBuffer)
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
uint8_t LCDQueueBuffer[ 1 * sizeof( LCDQUEUE ) ];
osStaticMessageQDef_t LCDQueueControlBlock;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue",
  .cb_mem = &LCDQueueControlBlock,
  .cb_size = sizeof(LCDQueueControlBlock),
  .mq_mem = &LCDQueueBuffer,
  .mq_size = sizeof(LCDQueueBuffer)
};
/* USER CODE BEGIN PV */


//--------------------------------------------------------------------------------
// For DMA SPI2 (LCD)
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == &hspi2)
	{
	    dma_spi_cnt--;
	    if(dma_spi_cnt==0)
	    {
	    	HAL_SPI_DMAStop(&hspi2);
	    	dma_spi_cnt=1;
	    	dma_spi_fl=1;
	    }
	}
}
// ---------------------------------------------------------------------------------

void print_time(void)
{
	for(int i = 0; i < 5; i++)
	{
		HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
		osDelay(100);
	}


						  		__HAL_TIM_SET_COUNTER(&htim1, 0);

//						  		if(klick == 0)
//						  		{

						  		// 1. Read time from RTS
						  					char time[20] = {0};
						  					char date[40] = {0};
						  					char time_buf[10] = {0};
						  					char time_buf_2[10] = {0};

						  					uint8_t seconds = 0;
						  					uint8_t minutes = 0;
						  					uint8_t hours = 0;
						  					uint8_t day = 0;
						  					uint8_t date_day = 0;
						  					uint8_t mounth = 0;
						  					uint8_t year = 0;

						  					uint8_t status = 99;

						  					status = ds3231_read(DS3231_REGISTER_SECONDS_DEFAULT, &seconds);
						  					if(HAL_OK != status)
						  					{
						  						int ff = 0;
						  					}
						  					ds3231_read(DS3231_REGISTER_MINUTES_DEFAULT, &minutes);
						  					ds3231_read(DS3231_REGISTER_HOURS_DEFAULT, &hours);

						  					ds3231_read(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &day);
						  					ds3231_read(DS3231_REGISTER_DATE_DEFAULT, &date_day);
						  					ds3231_read(DS3231_REGISTER_MONTH_DEFAULT, &mounth);
						  					ds3231_read(DS3231_REGISTER_YEAR_DEFAULT, &year);

						  					// Convert in string
						  					// Print minutes on OLED
						  					if(hours < 10)
						  					{
						  						memset(time_buf, 0, sizeof(time_buf));
						  						sprintf(time_buf, "%c", '0');
						  						sprintf(time_buf_2, "%d", hours);
						  						strcat(time_buf, time_buf_2);
						  						strcat(time, time_buf);
						  						strcat(time, ":");
						  					}
						  					else
						  					{
						  						sprintf(time_buf, "%d", hours);
						  						strcat(time, time_buf);
						  						strcat(time, ":");
						  						memset(time_buf, 0, sizeof(time_buf));
						  					}

						  		//			sprintf(time_buf, "%d", hours);
						  		//			strcat(time, time_buf);
						  		//			strcat(time, ":");
						  		//			memset(time_buf, 0, sizeof(time_buf));

						  					// Print minutes on OLED
						  					if(minutes < 10)
						  					{
						  						memset(time_buf, 0, sizeof(time_buf));
						  						sprintf(time_buf, "%c", '0');
						  						sprintf(time_buf_2, "%d", minutes);
						  						strcat(time_buf, time_buf_2);
						  						strcat(time, time_buf);
						  						strcat(time, ":");
						  					}
						  					else
						  					{
						  						sprintf(time_buf, "%d", minutes);
						  						strcat(time, time_buf);
						  						strcat(time, ":");
						  						memset(time_buf, 0, sizeof(time_buf));
						  					}

						  					// Print seconds on OLED
						  					if(seconds == 0)
						  					{
						  						clear();
						  						oled_update();
						  					}
						  					if(seconds < 10)
						  					{
						  						memset(time_buf, 0, sizeof(time_buf));
						  						sprintf(time_buf, "%c", '0');
						  						sprintf(time_buf_2, "%d", seconds);
						  						strcat(time_buf, time_buf_2);
						  						strcat(time, time_buf);
						  					}
						  					else
						  					{
						  						sprintf(time_buf, "%d", seconds);
						  						strcat(time, time_buf);
						  						memset(time_buf, 0, sizeof(time_buf));
						  					}


						  					// Print date
						  					sprintf(time_buf, "%d", date_day);
						  					strcat(date, time_buf);
						  					strcat(date, ".");
						  					memset(time_buf, 0, sizeof(time_buf));

						  					sprintf(time_buf, "%d", mounth);
						  					strcat(date, time_buf);
						  					strcat(date, ".");
						  					memset(time_buf, 0, sizeof(time_buf));

						  					sprintf(time_buf, "%d", year);
						  					strcat(date, "20");
						  					strcat(date, time_buf);
						  					memset(time_buf, 0, sizeof(time_buf));

						  					// day
						  					switch (day)
						  					{
						  						case 1:
						  							strcat(date, "  Monday");
						  							break;
						  						case 2:
						  							strcat(date, "  Tuesday");
						  							break;
						  						case 3:
						  							strcat(date, "  Wednesday");
						  							break;
						  						case 4:
						  							strcat(date, "  Thursday");
						  							break;
						  						case 5:
						  							strcat(date, "  Friday");
						  							break;
						  						case 6:
						  							strcat(date, "  Saturday");
						  							break;
						  						case 7:
						  							strcat(date, "  Sunday");
						  							break;
						  					}

						  					graphics_text(40, 0, 3, time);
						  					graphics_text(5, 24, 2, date);
						  					oled_update();


						  				osDelay(1000);
//						  			}

}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void Start_Show_Resources(void *argument);
void Start_UART_Task(void *argument);
void Start_LCD(void *argument);
void Start_LCD_touchscreen(void *argument);
void Start_RTC(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);		//  This TIM3 using for calculate how many time all tasks was running.




  /* IF LCD DOESN'T WORK !
  RIGHT IN RIGHT ORDER SPI2 AND DMA !!!!
  MX_DMA_Init();
  MX_SPI2_Init();
  */
  HAL_DMA_DeInit(&hdma_spi2_tx);
  HAL_SPI_DeInit(&hspi2);
  MX_DMA_Init();
  MX_SPI2_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (10, sizeof(QUEUE_t), &UARTQueue_attributes);

  /* creation of LCDQueue */
  LCDQueueHandle = osMessageQueueNew (1, sizeof(LCDQUEUE), &LCDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Show_Resources */
  Show_ResourcesHandle = osThreadNew(Start_Show_Resources, NULL, &Show_Resources_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(Start_UART_Task, NULL, &UART_Task_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(Start_LCD, NULL, &LCD_attributes);

  /* creation of LCD_touchscreen */
  LCD_touchscreenHandle = osThreadNew(Start_LCD_touchscreen, NULL, &LCD_touchscreen_attributes);

  /* creation of RTC */
  RTCHandle = osThreadNew(Start_RTC, NULL, &RTC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, T_MOSI_Pin|CS_I2C_SPI_Pin|CS_LCD_Pin|RESET_LCD_Pin
                          |DC_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AM2302_Pin|CS_microSD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, T_CLK_Pin|T_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : T_MOSI_Pin CS_I2C_SPI_Pin CS_LCD_Pin */
  GPIO_InitStruct.Pin = T_MOSI_Pin|CS_I2C_SPI_Pin|CS_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_LCD_Pin DC_LCD_Pin */
  GPIO_InitStruct.Pin = RESET_LCD_Pin|DC_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin AM2302_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|AM2302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : encoder_button_Pin */
  GPIO_InitStruct.Pin = encoder_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(encoder_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_microSD_Pin */
  GPIO_InitStruct.Pin = CS_microSD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_microSD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : T_CLK_Pin T_CS_Pin */
  GPIO_InitStruct.Pin = T_CLK_Pin|T_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T_MISO_Pin */
  GPIO_InitStruct.Pin = T_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Show_Resources */
/**
* @brief Function implementing the Show_Resources thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Show_Resources */
void Start_Show_Resources(void *argument)
{
  /* USER CODE BEGIN Start_Show_Resources */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(5000);												// Every 5 second task management will print data

	  char str_end_of_line[3] = {'\r','\n'};
	  char str_sig = '-';
	  char buff[10] = {0};

	  QUEUE_t msg;												// Make a queue
	  memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'
	  strcat(msg.Buf, ">>>>> Free heap memory: ");				// Add string to another (Total heap)

	  freemem = xPortGetFreeHeapSize();							// Function return how many free memory.
	  itoa(freemem, buff, 10);
	  strcat(msg.Buf, buff);
	  strcat(msg.Buf, str_end_of_line);

	  // add a hat
	  strcat(msg.Buf, "| TASK NAME  | STATUS | PRIOR | STACK | NUM |\n\r\0");

	  vTaskList(str_management_memory_str);						// Fill in str_management_memory_str array management task information

	  // Finding the  end of string
	  uint16_t buffer_size = 0;
	  while(msg.Buf[buffer_size] != '\0')
	  {
	  	buffer_size ++;
	  }

	  // Add str_management_memory_str to queue string
	  int i = 0;
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }

	  // add a hat
	  char str_line[] = {"-----------------------\n\r"};
	  char str_head_2[] = {"| TASK NAME | ABS TIME | TASK TIME% |\n\r"};
	  strcat(msg.Buf, str_line);
	  strcat(msg.Buf, str_head_2);

	  memset(str_management_memory_str, 0, sizeof(str_management_memory_str));	// Clean buffer

	  vTaskGetRunTimeStats(str_management_memory_str);							// Function return how much time all functions running.

	  buffer_size = buffer_size + i + (sizeof(str_line)-1) + (sizeof(str_head_2)-1);
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }
	  strcat(msg.Buf, "#########################################\n\r");

	  osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);					// Write data on queue (In will print on StartUART_Task task)
  }
  /* USER CODE END Start_Show_Resources */
}

/* USER CODE BEGIN Header_Start_UART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_Task */
void Start_UART_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_Task */
  /* Infinite loop */
  QUEUE_t msg;
  for(;;)
  {
	// osMessageQueueGet waiting data on a queue (If data are in queue so print it)
	osMessageQueueGet(UARTQueueHandle, &msg, 0, osWaitForever);			// Write for data on queue
	// Counting how many characters will be transmitted
	uint16_t buffer_size = 0;
	while(msg.Buf[buffer_size] != '\0')
	{
		buffer_size ++;
	}
	// Transmit over virtual comport
	CDC_Transmit_FS(msg.Buf, buffer_size);						// Transmit data over virtual comport
    osDelay(1);
  }
  /* USER CODE END Start_UART_Task */
}

/* USER CODE BEGIN Header_Start_LCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD */
void Start_LCD(void *argument)
{
  /* USER CODE BEGIN Start_LCD */
  /* Infinite loop */

	// Init LCD
	TFT9341_ini(240, 320);
	TFT9341_SetRotation(3);
	TFT9341_SetTextColor(TFT9341_WHITE);
	TFT9341_SetBackColor(TFT9341_BLUE);
	TFT9341_FillScreen(TFT9341_BLUE);

	// Init names sensors
	TFT9341_String_DMA(2,30, "TEST ");

	for(;;)
	{




		osDelay(1000);
  }
  /* USER CODE END Start_LCD */
}

/* USER CODE BEGIN Header_Start_LCD_touchscreen */
/**
* @brief Function implementing the LCD_touchscreen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD_touchscreen */
void Start_LCD_touchscreen(void *argument)
{
  /* USER CODE BEGIN Start_LCD_touchscreen */
  /* Infinite loop */
	LCDQUEUE msg;												// Make QUEUE
	memset(msg.buff, 0, sizeof(msg.buff));						// Fill in buff '\0'
	char buffer[50] = {0};

	for(;;)
  	 {
	  memset(msg.buff, 0, sizeof(msg.buff));						// Fill in buff '\0'
	  //СЕНСОР ЕКРАНУ
	  if(TP_Touchpad_Pressed() == TOUCHPAD_PRESSED)
	  {
		  strcat(buffer, "PRESED ");

		  uint16_t x_and_y[2] = {0};
		  uint8_t status_ts = TP_Read_Coordinates(x_and_y);
		  if(status_ts == TOUCHPAD_DATA_OK)
		  {
			  // Convert coordinate from uint16_t format in string format
			  // And save it in main buffer
			  char buff_x_coordinates[6] = {0};
			  char buff_y_coordinates[6] = {0};
			  char buff_coordinates[15] = {0};

			  strcat(buff_x_coordinates, "x: ");
			  itoa(x_and_y[0], buff_x_coordinates, 10);
			  strcat(buff_x_coordinates, " ");

			  strcat(buff_y_coordinates, "y: ");
			  itoa(x_and_y[1], buff_y_coordinates, 10);
			  strcat(buff_y_coordinates, " ");

			  strcat(buff_coordinates, buff_x_coordinates);
			  strcat(buff_coordinates, buff_y_coordinates);
			  strcat(buffer, buff_coordinates);
		  }
	  }
	  else
	  {
		  strcat(buffer, "NO PRESS                  ");
	  }

	  strcat(msg.buff, buffer);
	  osMessageQueuePut(LCDQueueHandle, &msg, 0, osWaitForever);  	// Write data on queue (In will print on StartUART_Task task)
	  memset(buffer, 0, sizeof(buffer));

	  osDelay(200);
    //osDelay(1);
  }
  /* USER CODE END Start_LCD_touchscreen */
}

/* USER CODE BEGIN Header_Start_RTC */
/**
* @brief Function implementing the RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RTC */
void Start_RTC(void *argument)
{
  /* USER CODE BEGIN Start_RTC */
  /* Infinite loop */

	osDelay(1000);

	// For resd time
	char time[20] = {0};
	char date[40] = {0};
	char time_buf[10] = {0};
	char time_buf_2[10] = {0};

	uint8_t seconds = 0;
	uint8_t minutes = 0;
	uint8_t hours = 0;
	uint8_t day = 0;
	uint8_t date_day = 0;
	uint8_t mounth = 0;
	uint8_t year = 0;
	uint8_t status = 9;
	//

	oled_init();
	oled_update();
	ds3231_I2C_init();

	// Encoder
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	int32_t prevCounter = 0;
	//

	for(;;)
	{
		switch (klick)
		{
			case 0:

//				for(int i = 0; i < 5; i++)
//				{
//					HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
//					osDelay(100);
//					HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
//					osDelay(100);
//				}


				while(klick == 0)
				{
					memset(time, 0, sizeof(time));
					memset(date, 0, sizeof(date));
					memset(time_buf, 0, sizeof(time_buf));
					memset(time_buf_2, 0, sizeof(time_buf_2));

					// 1. Read time from RTS
					// Red status (Detect DS3231)
					uint8_t buff= 0;
					status = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)DS3231_I2C_ADDRESS<<1,(uint16_t)0, (uint16_t) 1, &buff, (uint16_t) 1,(uint32_t) 1000);
					if(status != HAL_OK )				// If DS3231 doesen'e detect
					{
						clear();
						while(status != HAL_OK)
						{
							strcat(time_buf, "RTC ERROR");
							graphics_text(8, 5, 3, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));
							osDelay(300);

							invert_rectangle(0, 0, 128, 32);
							oled_update();
							osDelay(300);

							clear();
							oled_update();

							status = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)DS3231_I2C_ADDRESS<<1,(uint16_t)0, (uint16_t) 1, &buff, (uint16_t) 1,(uint32_t) 1000);
						}
						clear();
					}

					else	// If all DS3231 detected, read time and date
					{
						ds3231_read(DS3231_REGISTER_SECONDS_DEFAULT, &seconds);
						ds3231_read(DS3231_REGISTER_MINUTES_DEFAULT, &minutes);
						ds3231_read(DS3231_REGISTER_HOURS_DEFAULT, &hours);

						ds3231_read(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &day);
						ds3231_read(DS3231_REGISTER_DATE_DEFAULT, &date_day);
						ds3231_read(DS3231_REGISTER_MONTH_DEFAULT, &mounth);
						ds3231_read(DS3231_REGISTER_YEAR_DEFAULT, &year);

						// Convert in string
						// Print minutes on OLED
						if(hours < 10)
						{
							memset(time_buf, 0, sizeof(time_buf));
							sprintf(time_buf, "%c", '0');
							sprintf(time_buf_2, "%d", hours);
							strcat(time_buf, time_buf_2);
							strcat(time, time_buf);
							strcat(time, ":");
						}
						else
						{
							sprintf(time_buf, "%d", hours);
							strcat(time, time_buf);
							strcat(time, ":");
							memset(time_buf, 0, sizeof(time_buf));
						}

						// Print minutes on OLED
						if(minutes < 10)
						{
							memset(time_buf, 0, sizeof(time_buf));
							sprintf(time_buf, "%c", '0');
							sprintf(time_buf_2, "%d", minutes);
							strcat(time_buf, time_buf_2);
							strcat(time, time_buf);
							strcat(time, ":");
						}
						else
						{
							sprintf(time_buf, "%d", minutes);
							strcat(time, time_buf);
							strcat(time, ":");
							memset(time_buf, 0, sizeof(time_buf));
						}

						// Print seconds on OLED
						if(seconds == 0)
						{
							clear();
							oled_update();
						}
						if(seconds < 10)
						{
							memset(time_buf, 0, sizeof(time_buf));
							sprintf(time_buf, "%c", '0');
							sprintf(time_buf_2, "%d", seconds);
							strcat(time_buf, time_buf_2);
							strcat(time, time_buf);
						}
						else
						{
							sprintf(time_buf, "%d", seconds);
							strcat(time, time_buf);
							memset(time_buf, 0, sizeof(time_buf));
						}

						// Print date
						sprintf(time_buf, "%d", date_day);
						strcat(date, time_buf);
						strcat(date, ".");
						memset(time_buf, 0, sizeof(time_buf));

						sprintf(time_buf, "%d", mounth);
						strcat(date, time_buf);
						strcat(date, ".");
						memset(time_buf, 0, sizeof(time_buf));

						sprintf(time_buf, "%d", year);
						strcat(date, "20");
						strcat(date, time_buf);
						memset(time_buf, 0, sizeof(time_buf));

						// day
						switch (day)
						{
							case 1:
								strcat(date, "  Monday");
								break;
							case 2:
								strcat(date, "  Tuesday");
								break;
							case 3:
								strcat(date, "  Wednesday");
								break;
							case 4:
								strcat(date, "  Thursday");
								break;
							case 5:
								strcat(date, "  Friday");
								break;
							case 6:
								strcat(date, "  Saturday");
								break;
							case 7:
								strcat(date, "  Sunday");
								break;
						}

						graphics_text(40, 0, 3, time);
						graphics_text(5, 24, 2, date);
						oled_update();

						osDelay(1000);
					}

				}
		  		break;


			case 1:
				// Set yer
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);

				graphics_text(0, 0, 1, "   SET:");
				graphics_text(0, 8, 1, "YEAR   ");
				oled_update();

				__HAL_TIM_SET_COUNTER(&htim1, 0);								// Start count encoder from 0

				while(klick == 1)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						prevCounter = currCounter;
						if(currCounter > 100)									// Encoder count from 0 to 100
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);
						}
						if(currCounter < 0)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);
						}
//						prevCounter = currCounter;

						graphics_text(0, 16, 1, "           ");
						oled_update();

						sprintf(time_buf, "%d", currCounter);
						graphics_text(0, 16, 1, time_buf);
						oled_update();
						memset(time_buf, 0, sizeof(time_buf));
						osDelay(10);
					}
				}
				while(klick == 2)
				{
					// write data
					ds3231_set(DS3231_REGISTER_YEAR_DEFAULT, &prevCounter);
					//klick = 3;
					//break;
				}

				break;

			case 2:
				// set mounth
				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 1);								// Start count encoder from 0

				while(klick == 2)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 12)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 1);					// Encoder count from 0 to 10
						}

						prevCounter = currCounter;

					}
				}

				break;

			case 3:
				// Set date
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 1);								// Start count encoder from 1

				while(klick == 3)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 31)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);					// Encoder count from 1 to 32
						}

						prevCounter = currCounter;

					}
				}
				break;

			case 4:
				// Set day of week
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 1);

				while(klick == 4)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 7)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 1);
						}

						prevCounter = currCounter;

					}
				}
				break;

			case 5:
				// Set hour
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 0);

				while(klick == 5)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 23)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);
						}

						prevCounter = currCounter;
					}
				}
				break;

			case 6:
				// Set minutes
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 0);

				while(klick == 6)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 59)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);
						}

						prevCounter = currCounter;
					}
				}
				break;

			case 7:
				// Set minutes
				HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

				__HAL_TIM_SET_COUNTER(&htim1, 0);

				while(klick == 7)
				{
					currCounter = __HAL_TIM_GET_COUNTER(&htim1);
					currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

					if(currCounter != prevCounter)
					{
						if(currCounter > 59)
						{
							__HAL_TIM_SET_COUNTER(&htim1, 0);
						}

						prevCounter = currCounter;
					}
				}
				break;
		}















//		/////////////////////////////////////////////////////////////////////
//		// 1. Set time
//		bool set_time = false;
//		bool exit_from_set_time = false;
//
//		uint8_t clik = 0;
//		char klik_buf[3] = {0};
//		if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)	// If button pressed
//		{
//			clik = 1;
//				do{
//					int currCounter = 0;
//
//					switch (clik)
//					{
//						case 1:
//							// set the yers
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "YEAR   ");
//							oled_update();
//
//							osDelay(500);
//
//							//__HAL_TIM_SET_COUNTER(&htim1, 10);
//
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter < 10)
//									{
//										prevCounter = 10;
//										//_HAL_TIM_SET_COUNTER(&htim1, 10);
//									}
//									if(prevCounter > 99)
//									{
//										prevCounter = 99;
//										//__HAL_TIM_SET_COUNTER(&htim1, 99);
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//																		// write data
//									ds3231_set(DS3231_REGISTER_YEAR_DEFAULT, &prevCounter);
//									clik = 2;
//									break;
//								}
//							}
//
//						break;
//
//						case 2:
//							// set mounth
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "MONTH ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 1);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter <= 0)
//									{
//										prevCounter = 1;
//									}
//									if(prevCounter > 12)
//									{
//										prevCounter = 12;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//									// write data
//									ds3231_set(DS3231_REGISTER_MONTH_DEFAULT, &prevCounter);
//									clik = 3;
//									break;
//								}
//							}
//						break;
//
//						case 3:
//							// set date
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "DATE    ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 1);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter <= 0)
//									{
//										prevCounter = 1;
//									}
//									if(prevCounter > 31)
//									{
//										prevCounter = 1;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//									// write data
//									ds3231_set(DS3231_REGISTER_DATE_DEFAULT, &prevCounter);
//									clik = 4;
//									break;
//								}
//							}
//						break;
//
//						case 4:
//							// set day of week
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "DAY      ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 1);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter <= 0)
//									{
//										prevCounter = 1;
//									}
//									if(prevCounter > 7)
//									{
//										prevCounter = 1;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//																// write data
//									ds3231_set(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &prevCounter);
//									clik = 5;
//									break;
//								}
//							}
//
//						break;
//
//						case 5:
//							// set hours
//							// set date
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "HOUR    ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 0);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter < 0)
//									{
//										prevCounter = 0;
//									}
//									if(prevCounter > 24)
//									{
//										prevCounter = 0;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//									// write data
//									ds3231_set(DS3231_REGISTER_HOURS_DEFAULT, &prevCounter);
//									clik = 6;
//									break;
//								}
//							}
//						break;
//
//						case 6:
//							// set minutes
//							// set hours
//							// set date
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "MINUTE ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 0);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter < 0)
//									{
//										prevCounter = 0;
//									}
//									if(prevCounter > 59)
//									{
//										prevCounter = 0;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//									// write data
//									ds3231_set(DS3231_REGISTER_MINUTES_DEFAULT, &prevCounter);
//									clik = 7;
//									break;
//								}
//							}
//
//						break;
//
//						case 7:
//							// set seconds
//							currCounter = 0;
//							prevCounter = 0;
//
//							graphics_text(0, 0, 1, "   SET:");
//							graphics_text(0, 8, 1, "SECOND   ");
//							oled_update();
//
//							__HAL_TIM_SET_COUNTER(&htim1, 1);
//
//							osDelay(500);
//							while(1)
//							{
//								currCounter = __HAL_TIM_GET_COUNTER(&htim1);
//								currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
//								if(currCounter != prevCounter)
//								{
//									prevCounter = currCounter;
//									if(prevCounter < 0)
//									{
//										prevCounter = 0;
//									}
//									if(prevCounter > 59)
//									{
//										prevCounter = 0;
//									}
//
//									graphics_text(0, 16, 1, "         ");
// 									oled_update();
//
//									sprintf(klik_buf, "%d", prevCounter);
//									graphics_text(0, 16, 1, klik_buf);
//									oled_update();
//									memset(klik_buf, 0, sizeof(klik_buf));
//
//								}
//								if(HAL_GPIO_ReadPin(GPIOE, encoder_button_Pin) == 0)
//								{
//									// write data
//									ds3231_set(DS3231_REGISTER_SECONDS_DEFAULT, &prevCounter);
//									clik = 8;
//
//
//									osDelay(500);
//									graphics_text(0, 0, 1, "                  ");
//									graphics_text(0, 8, 1, "               ");
//									graphics_text(0, 16, 1, "               ");
//									oled_update();
//
//									for(uint8_t k =0; k< 5; k++)
//									{
//										graphics_text(0, 0, 1, " END");
//										oled_update();
//										osDelay(300);
//										graphics_text(0, 0, 1, "         ");
//										oled_update();
//										osDelay(100);
//									}
//
//
//
//									break;
//								}
//							}
//
//							break;
//
//					}
//
//					osDelay(300);
//
//			}while(clik <= 7);
//
//
//		}
//		else		// Print current time
//		{
//			// 1. Read time from RTS
//			char time[20] = {0};
//			char date[40] = {0};
//			char time_buf[10] = {0};
//			char time_buf_2[10] = {0};
//
//			uint8_t seconds = 0;
//			uint8_t minutes = 0;
//			uint8_t hours = 0;
//			uint8_t day = 0;
//			uint8_t date_day = 0;
//			uint8_t mounth = 0;
//			uint8_t year = 0;
//
//			uint8_t status = 99;
//
//			status = ds3231_read(DS3231_REGISTER_SECONDS_DEFAULT, &seconds);
//			if(HAL_OK != status)
//			{
//				int ff = 0;
//			}
//			ds3231_read(DS3231_REGISTER_MINUTES_DEFAULT, &minutes);
//			ds3231_read(DS3231_REGISTER_HOURS_DEFAULT, &hours);
//
//			ds3231_read(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &day);
//			ds3231_read(DS3231_REGISTER_DATE_DEFAULT, &date_day);
//			ds3231_read(DS3231_REGISTER_MONTH_DEFAULT, &mounth);
//			ds3231_read(DS3231_REGISTER_YEAR_DEFAULT, &year);
//
//			// Convert in string
//			// Print minutes on OLED
//			if(hours < 10)
//			{
//				memset(time_buf, 0, sizeof(time_buf));
//				sprintf(time_buf, "%c", '0');
//				sprintf(time_buf_2, "%d", hours);
//				strcat(time_buf, time_buf_2);
//				strcat(time, time_buf);
//				strcat(time, ":");
//			}
//			else
//			{
//				sprintf(time_buf, "%d", hours);
//				strcat(time, time_buf);
//				strcat(time, ":");
//				memset(time_buf, 0, sizeof(time_buf));
//			}
//
////			sprintf(time_buf, "%d", hours);
////			strcat(time, time_buf);
////			strcat(time, ":");
////			memset(time_buf, 0, sizeof(time_buf));
//
//			// Print minutes on OLED
//			if(minutes < 10)
//			{
//				memset(time_buf, 0, sizeof(time_buf));
//				sprintf(time_buf, "%c", '0');
//				sprintf(time_buf_2, "%d", minutes);
//				strcat(time_buf, time_buf_2);
//				strcat(time, time_buf);
//				strcat(time, ":");
//			}
//			else
//			{
//				sprintf(time_buf, "%d", minutes);
//				strcat(time, time_buf);
//				strcat(time, ":");
//				memset(time_buf, 0, sizeof(time_buf));
//			}
//
//			// Print seconds on OLED
//			if(seconds == 0)
//			{
//				clear();
//				oled_update();
//			}
//			if(seconds < 10)
//			{
//				memset(time_buf, 0, sizeof(time_buf));
//				sprintf(time_buf, "%c", '0');
//				sprintf(time_buf_2, "%d", seconds);
//				strcat(time_buf, time_buf_2);
//				strcat(time, time_buf);
//			}
//			else
//			{
//				sprintf(time_buf, "%d", seconds);
//				strcat(time, time_buf);
//				memset(time_buf, 0, sizeof(time_buf));
//			}
//
//
//			// Print date
//			sprintf(time_buf, "%d", date_day);
//			strcat(date, time_buf);
//			strcat(date, ".");
//			memset(time_buf, 0, sizeof(time_buf));
//
//			sprintf(time_buf, "%d", mounth);
//			strcat(date, time_buf);
//			strcat(date, ".");
//			memset(time_buf, 0, sizeof(time_buf));
//
//			sprintf(time_buf, "%d", year);
//			strcat(date, "20");
//			strcat(date, time_buf);
//			memset(time_buf, 0, sizeof(time_buf));
//
//			// day
//			switch (day)
//			{
//				case 1:
//					strcat(date, "  Monday");
//					break;
//				case 2:
//					strcat(date, "  Tuesday");
//					break;
//				case 3:
//					strcat(date, "  Wednesday");
//					break;
//				case 4:
//					strcat(date, "  Thursday");
//					break;
//				case 5:
//					strcat(date, "  Friday");
//					break;
//				case 6:
//					strcat(date, "  Saturday");
//					break;
//				case 7:
//					strcat(date, "  Sunday");
//					break;
//			}
//
//			graphics_text(40, 0, 3, time);
//			graphics_text(5, 24, 2, date);
//			oled_update();
//		}
//
//		osDelay(1000);
	}
  /* USER CODE END Start_RTC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */





	// Handler for count how many time works any tasks
	if(htim->Instance == TIM3)
	{
		ulHighFreqebcyTimerTicks++;					// Update time tasks counter
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
