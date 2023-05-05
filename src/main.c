/***************************************************************************//**
 * @file
 * @brief FreeRTOS Blink Demo for Energy Micro EFM32GG_STK3700 Starter Kit
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

#include "em_chip.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "sleep.h"

// Own includes
#include "my_i2c.h"
// -------

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

#define ADDR_MAGNET 0x3D
#define ADDR_GA 0xD7

#define LOOP 1

/* Structure with parameters for LedBlink */
typedef struct {
	QueueHandle_t accXQueue;
} TaskParams_1_2_t;

typedef struct {
	int hasHit;
} TaskParams_2_3_t;

/***************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 ******************************************************************************/
static void WriteOutputTask(void *pParameters)
{
	TaskParams_1_2_t *params = (TaskParams_1_2_t *)pParameters;
	int16_t receivedData;
	for(;;)
	{
		if (xQueueReceive(params->accXQueue, &receivedData, portMAX_DELAY) == pdPASS) {
			// Failed to receive data from the queue
			printf("Data XL X: %i\n", receivedData);
		}
	}
}
static void ReadI2CTask(void *pParameters)
{
	TaskParams_1_2_t *params = (TaskParams_1_2_t *)pParameters;
	I2C_WriteRegister(0x20, 1 << 6, ADDR_GA); // Activar acc
	I2C_WriteRegister(0x10, 1 << 6, ADDR_GA); // Activar gyro
	uint8_t data_H = 0, data_L = 0;
	int16_t data;

    if(LOOP)
    {
	  for (;; ) {
		  // Test acc
		  I2C_ReadRegister(0x28, &data_L, ADDR_GA);
		  I2C_ReadRegister(0x29, &data_H, ADDR_GA);
		  data = data_H << 8 | data_L;
		  xQueueSend(params->accXQueue, &data, 0);
		  //printf("Data XL X: %i\t", data);

		  // Test gyro
		  I2C_ReadRegister(0x18, &data_L, ADDR_GA);
		  I2C_ReadRegister(0x19, &data_H, ADDR_GA);
		  data = data_H << 8 | data_L;

		  // Test magnet
		  I2C_ReadRegister(0x28, &data_L, ADDR_MAGNET);
		  I2C_ReadRegister(0x29, &data_H, ADDR_MAGNET);
		  data = data_H << 8 | data_L;
    }
  }
}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize LED driver */
  BSP_LedsInit();

  /* Initialize SLEEP driver, no calbacks are used */
  //SLEEP_Init(NULL, NULL);
#if (configSLEEP_MODE < 3)
  /* do not let to sleep deeper than define */
  SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
#endif


  int ret = i2c_initSemaphore();

  printf("InitSemaphore: %d \n", ret);

  BSP_I2C_Init();

  I2C_Test(ADDR_GA, ADDR_MAGNET);

  // ------- Main Code -------
  /* Parameters value for taks*/
  QueueHandle_t accXQueue = xQueueCreate(10, sizeof(int16_t));
  static TaskParams_1_2_t parametersToTask1;
  parametersToTask1.accXQueue = accXQueue;



  /*Create two task for blinking leds*/
  xTaskCreate(ReadI2CTask, (const char *) "ReadI2CTask", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  xTaskCreate(WriteOutputTask, (const char *) "WriteOutputTask", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);


  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}
