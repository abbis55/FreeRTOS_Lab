/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t varBlink1 = 0;
uint8_t varBlink2 = 0;
uint8_t varBlink3 = 0;

SemaphoreHandle_t xButtonSemaphore = NULL;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Blink1Task */
osThreadId_t Blink1TaskHandle;
const osThreadAttr_t Blink1Task_attributes = {
  .name = "Blink1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Blink2Task */
osThreadId_t Blink2TaskHandle;
const osThreadAttr_t Blink2Task_attributes = {
  .name = "Blink2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TriggTask */
osThreadId_t TriggTaskHandle;
const osThreadAttr_t TriggTask_attributes = {
  .name = "TriggTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UserbuttonTask */
osThreadId_t UserbuttonTaskHandle;
const osThreadAttr_t UserbuttonTask_attributes = {
  .name = "UserbuttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void wait_cycles(uint32_t n);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Blink1(void *argument);
void Blink2(void *argument);
void Trigg(void *argument);
void Userbutton(void *argument);
void LedTaskEntry(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	xButtonSemaphore = xSemaphoreCreateBinary();
	configASSERT(xButtonSemaphore);
	/* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Blink1Task */
  Blink1TaskHandle = osThreadNew(Blink1, NULL, &Blink1Task_attributes);

  /* creation of Blink2Task */
  //Blink2TaskHandle = osThreadNew(Blink2, NULL, &Blink2Task_attributes);

  /* creation of TriggTask */
  //TriggTaskHandle = osThreadNew(Trigg, NULL, &TriggTask_attributes);

  /* creation of UserbuttonTask */
  UserbuttonTaskHandle = osThreadNew(Userbutton, NULL, &UserbuttonTask_attributes);

  /* creation of LedTask */
  //LedTaskHandle = osThreadNew(LedTaskEntry, NULL, &LedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Blink1 */
/**
* @brief Function implementing the Blink1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Blink1 */
void Blink1(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100); // 100 ms blink

  for(;;)
  {
    // Om Userbutton gav semafor sedan sist (knappen nedtryckt),
    // skippa blink (håll LED av) denna period
    if (xSemaphoreTake(xButtonSemaphore, 0) == pdTRUE) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LED off
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    // Annars: toggla LED normalt var 100 ms
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

  /* USER CODE END Blink1 */


/* USER CODE BEGIN Header_Blink2 */
/**
* @brief Function implementing the Blink2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Blink2 */
void Blink2(void *argument)
{
  /* USER CODE BEGIN Blink2 */

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 20 ms period

    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        varBlink2 = 1;
        wait_cycles(400000);   // mer jobb
        varBlink2 = 0;

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
  /* USER CODE END Blink2 */
}

/* USER CODE BEGIN Header_Trigg */
/**
* @brief Function implementing the TriggTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Trigg */
void Trigg(void *argument)
{
  /* USER CODE BEGIN Trigg */
  /* Infinite loop */
	TickType_t xLastWakeTime;
	  const TickType_t xPeriod = pdMS_TO_TICKS(200); // ms to ticks

	  // Initialise the xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount();

	  /* Infinite loop */
	  for(;;)
	  {
	    vTaskDelayUntil(&xLastWakeTime, xPeriod);
	    wait_cycles(10); // här ska du lägga breakpoint sen
	  }
  /* USER CODE END Trigg */
}

/* USER CODE BEGIN Header_Userbutton */
/**
* @brief Function implementing the UserbuttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Userbutton */
void Userbutton(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(20); // kolla knappen var 20 ms

  for(;;)
  {
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
      // Knappen nedtryckt → signalera
      xSemaphoreGive(xButtonSemaphore);

      // (valfritt) markera i SWV om du vill se blå puls i Task 2:
      // varBlink3 = 1;
      // wait_cycles(250000); // ~3 ms jobb (om du vill simulera arbete)
      // varBlink3 = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


/* USER CODE BEGIN Header_LedTaskEntry */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTaskEntry */
/*
void LedTaskEntry(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(100); // 100 ms blink

  for(;;)
  {
    // Om semaforen gavs nyligen (knappen nedtryckt) → hoppa över toggle denna period
    if (xSemaphoreTake(xButtonSemaphore, 0) == pdTRUE) {
    	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LED av
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue; // tillbaka till början av loopen
    }

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
*/

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void wait_cycles(uint32_t n) {
  uint32_t l = n/3; // cycles per loop is 3
  asm volatile( "0:" "SUBS %[count], 1;" "BNE 0b;" : [count] "+r" (l) );
}
/* USER CODE END Application */

