/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "printf.h"
#include <stdio.h>
#include "dma.h"
#include "semphr.h"
#include <string.h>
#include "RingBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE_TXBUF_UART1 4
#define SIZE_SAMPLE_UART1 4

#define SIZE_TXBUF_UART2 4
#define SIZE_SAMPLE_UART2 4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t Buff_TxUart1[SIZE_TXBUF_UART1];
static uint8_t Buff_TxUart2[SIZE_TXBUF_UART2];

static uint8_t firstRunUart1 = 1;
static uint8_t firstRunUart2 = 0;

static uint8_t tempUART1[SIZE_SAMPLE_UART1];
static uint8_t tempUART2[SIZE_SAMPLE_UART2];

static RingBuffer_t Buff_RxUart1;
static RingBuffer_t Buff_RxUart2;



/* USER CODE END Variables */
/* Definitions for HBTask */
osThreadId_t HBTaskHandle;
const osThreadAttr_t HBTask_attributes = {
  .name = "HBTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PrintToPCTask */
osThreadId_t PrintToPCTaskHandle;
const osThreadAttr_t PrintToPCTask_attributes = {
  .name = "PrintToPCTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ToNucleoTask */
osThreadId_t ToNucleoTaskHandle;
const osThreadAttr_t ToNucleoTask_attributes = {
  .name = "ToNucleoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SemaphoreUART2 */
osSemaphoreId_t SemaphoreUART2Handle;
const osSemaphoreAttr_t SemaphoreUART2_attributes = {
  .name = "SemaphoreUART2"
};
/* Definitions for SemaphoreUART1 */
osSemaphoreId_t SemaphoreUART1Handle;
const osSemaphoreAttr_t SemaphoreUART1_attributes = {
  .name = "SemaphoreUART1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHearthBeatTask(void *argument);
void StartPrintToPCTask(void *argument);
void StartToNucleoTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreUART2 */
  SemaphoreUART2Handle = osSemaphoreNew(1, 1, &SemaphoreUART2_attributes);

  /* creation of SemaphoreUART1 */
  SemaphoreUART1Handle = osSemaphoreNew(1, 1, &SemaphoreUART1_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */


  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HBTask */
  HBTaskHandle = osThreadNew(StartHearthBeatTask, NULL, &HBTask_attributes);

  /* creation of PrintToPCTask */
  PrintToPCTaskHandle = osThreadNew(StartPrintToPCTask, NULL, &PrintToPCTask_attributes);

  /* creation of ToNucleoTask */
  ToNucleoTaskHandle = osThreadNew(StartToNucleoTask, NULL, &ToNucleoTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHearthBeatTask */
/**
 * @brief  Function implementing the HBTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHearthBeatTask */
void StartHearthBeatTask(void *argument)
{
  /* USER CODE BEGIN StartHearthBeatTask */

	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		osDelay(200);
	}
  /* USER CODE END StartHearthBeatTask */
}

/* USER CODE BEGIN Header_StartPrintToPCTask */
/**
 * @brief Function implementing the PrintToPCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPrintToPCTask */
void StartPrintToPCTask(void *argument)
{
  /* USER CODE BEGIN StartPrintToPCTask */
	HAL_UART_Receive_DMA(&huart2,
			tempUART2, SIZE_SAMPLE_UART2);
	uint8_t errorerUart2;
	printf("Hello PC, it's Discovery board here!!\n\r");
	xSemaphoreTake(SemaphoreUART2Handle, portMAX_DELAY);

	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(SemaphoreUART2Handle, portMAX_DELAY);
		errorerUart2 = RingBuffReadSome(&Buff_RxUart2, Buff_TxUart2, SIZE_TXBUF_UART2);
		if(errorerUart2 != 0)
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

		printf("Echo: %c%c%c%c\n\r", Buff_TxUart2[0],
				Buff_TxUart2[1],
				Buff_TxUart2[2],
				Buff_TxUart2[3]);
	}
  /* USER CODE END StartPrintToPCTask */
}

/* USER CODE BEGIN Header_StartToNucleoTask */
/**
 * @brief Function implementing the ToNucleoTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartToNucleoTask */
void StartToNucleoTask(void *argument)
{
  /* USER CODE BEGIN StartToNucleoTask */
	HAL_UART_Receive_DMA(&huart4, tempUART1, SIZE_SAMPLE_UART1);
	xSemaphoreTake(SemaphoreUART1Handle, portMAX_DELAY);
	uint8_t errorerUart1;
	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(SemaphoreUART1Handle, portMAX_DELAY);
		errorerUart1 = RingBuffReadSome(&Buff_RxUart1, Buff_TxUart1, SIZE_TXBUF_UART1);
		if(errorerUart1 != 0)
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		printf("NUCLEO: %c%c%c%c\n\r", Buff_TxUart1[0],
				Buff_TxUart1[1],
				Buff_TxUart1[2],
				Buff_TxUart1[3]);
	}
  /* USER CODE END StartToNucleoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
	HAL_UART_Transmit(&huart2, (uint8_t *) &character, 1, 1000);

	// send char to console etc.
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t errorInISR;
	/************************************************************************************/
	if(huart == &huart4)
	{
		HAL_UART_Receive_DMA(&huart4, tempUART1, SIZE_SAMPLE_UART1);

		errorInISR = RingBuffWrite(&Buff_RxUart1, tempUART1, SIZE_SAMPLE_UART1);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		//if(errorInISR != 0)
			//HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		xSemaphoreGiveFromISR(SemaphoreUART1Handle, pdTRUE);
	}
	/**************************************************************************************/
	if(huart == &huart2)
		{
			HAL_UART_Receive_DMA(&huart2, tempUART2, SIZE_SAMPLE_UART2);
			errorInISR = RingBuffWrite(&Buff_RxUart2, tempUART2, SIZE_SAMPLE_UART2);
			if(errorInISR != 0)
				HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			xSemaphoreGiveFromISR(SemaphoreUART2Handle, pdTRUE);

		}
	/**************************************************************************************/
}
/* USER CODE END Application */

