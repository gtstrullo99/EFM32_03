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

/* Structure with parameters for LedBlink */
typedef struct {

} TaskParams_t;

/***************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 ******************************************************************************/
static void TestTask(void *pParameters)
{
	uint8_t data_H = 0, data_L = 0;
	int16_t data;

  for (;; ) {
	  // Test here
	  I2C_ReadRegister(0x28, data_L, ADDR_GA);
	  I2C_ReadRegister(0x29, data_H, ADDR_GA);
	  data = data_H << 8 | data_L;

	  printf("Data XL X: %i\n", data);

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
  static TaskParams_t parametersToTask1 = {  };



  /*Create two task for blinking leds*/
  xTaskCreate(TestTask, (const char *) "TestTask", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}
