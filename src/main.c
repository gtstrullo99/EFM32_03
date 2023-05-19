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

// Own includes
#include "my_i2c.h"
// -------

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

#define ADDR_MAGNET 0x3D
#define ADDR_GA 0xD7

#define COLLISION_THRESHOLD 50e6
#define COLLISION_DELAY pdMS_TO_TICKS(500)


typedef struct {
	QueueHandle_t accQueue;
} TaskParams_1_t;

typedef struct {
	QueueHandle_t accQueue;
	QueueHandle_t moduleQueue;
} TaskParams_2_t;

typedef struct {
	QueueHandle_t moduleQueue;
	QueueHandle_t buttonQueue;
} TaskParams_3_t;

typedef struct {
	QueueHandle_t buttonQueue;
} TaskParams_4_t;

typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
} queueVector;

static void ReadButtonTask(void *pParameters)
{
	TaskParams_4_t *params = (TaskParams_4_t *)pParameters;
	uint8_t res;
	GPIO_PinModeSet(gpioPortB, 9, gpioModeInputPull, 1); // Assuming pull-up resistor

	for(;;)
	{
		res = !GPIO_PinInGet(gpioPortB, 9);
		xQueueSend(params->buttonQueue, &res, 0);
	}
}

static void AirbagControlTask(void *pParameters)
{
	TaskParams_3_t *params = (TaskParams_3_t *)pParameters;
	int32_t receivedData;
	uint8_t buttonPressed;
	TickType_t startTime;
	int isOn = 0;
	int isColliding = 0;
	int lastButttonPressed = 0;
	for(;;)
	{
		if (xQueueReceive(params->moduleQueue, &receivedData, portMAX_DELAY) == pdPASS) {
			if(receivedData > COLLISION_THRESHOLD && !isOn && !isColliding)
			{
				//printf("Colisio: %li\n", receivedData);
				printf("Colisio: Airbags activats! \n");

				isOn = 1;
				BSP_LedSet(0);

				startTime = xTaskGetTickCount();
				isColliding = 1;
			}
			else if(receivedData > COLLISION_THRESHOLD && isOn && !isColliding)
			{
				printf("Colisio sense airbags: PERILL! \n");
				startTime = xTaskGetTickCount();
				isColliding = 1;
			}
		}


		if (xQueueReceive(params->buttonQueue, &buttonPressed, portMAX_DELAY) == pdPASS) {
			if (lastButttonPressed != buttonPressed)
			{
				if(buttonPressed && isOn)
				{
					printf("Airbags recargats. \n");
					isOn = 0;
					BSP_LedClear(0);
				}
				else if(buttonPressed && !isOn)
				{
					printf("Error: Airbags ja recargats. \n");
				}
				lastButttonPressed = buttonPressed;
			}
		}

		if(isColliding && xTaskGetTickCount() > startTime + COLLISION_DELAY)
		{
			isColliding = 0;
		}
	}
}

static void CalculateModule(void *pParameters)
{
	TaskParams_2_t *params = (TaskParams_2_t *)pParameters;
	queueVector receivedData;
	int32_t module_result;

	for(;;)
	{
		if (xQueueReceive(params->accQueue, &receivedData, portMAX_DELAY) == pdPASS) {
			module_result = (receivedData.X*receivedData.X) + (receivedData.Y*receivedData.Y) + (receivedData.Z*receivedData.Z);
			xQueueSend(params->moduleQueue, &module_result, 0);
		}

	}
}

static void ReadI2CTask(void *pParameters)
{
	TaskParams_1_t *params = (TaskParams_1_t *)pParameters;
	I2C_WriteRegister(0x20, 1 << 6 | 1 << 3, ADDR_GA); // Activar acc
	I2C_WriteRegister(0x10, 1 << 6, ADDR_GA); // Activar gyro
	uint8_t data_H = 0, data_L = 0;
	int16_t data;

	queueVector qVector;

	for (;; ) {
	  // AccX
	  I2C_ReadRegister(0x28, &data_L, ADDR_GA);
	  I2C_ReadRegister(0x29, &data_H, ADDR_GA);
	  data = data_H << 8 | data_L;
	  qVector.X = data;

	  // AccY
	  I2C_ReadRegister(0x2A, &data_L, ADDR_GA);
	  I2C_ReadRegister(0x2B, &data_H, ADDR_GA);
	  data = data_H << 8 | data_L;
	  qVector.Y = data;

	  // AccZ
	  I2C_ReadRegister(0x2C, &data_L, ADDR_GA);
	  I2C_ReadRegister(0x2D, &data_H, ADDR_GA);
	  data = data_H << 8 | data_L;
	  qVector.Z = data;

	  xQueueSend(params->accQueue, &qVector, 0);

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
  /* Setting state of leds*/
  BSP_LedSet(0);
  BSP_LedSet(1);
  BSP_LedClear(0);
  BSP_LedClear(1);

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
  QueueHandle_t accQueue = xQueueCreate(10, sizeof(queueVector));
  QueueHandle_t moduleQueue = xQueueCreate(10, sizeof(int32_t));
  QueueHandle_t buttonQueue = xQueueCreate(10, sizeof(uint8_t));

  static TaskParams_1_t parametersToTask1;
  static TaskParams_2_t parametersToTask2;
  static TaskParams_3_t parametersToTask3;
  static TaskParams_4_t parametersToTask4;

  parametersToTask1.accQueue = accQueue;
  parametersToTask2.accQueue = accQueue;
  parametersToTask2.moduleQueue = moduleQueue;
  parametersToTask3.moduleQueue = moduleQueue;
  parametersToTask3.buttonQueue = buttonQueue;
  parametersToTask4.buttonQueue = buttonQueue;

  /*Create two task for blinking leds*/
  xTaskCreate(ReadI2CTask, (const char *) "ReadI2CTask", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  xTaskCreate(CalculateModule, (const char *) "CalculateModule", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);
  xTaskCreate(AirbagControlTask, (const char *) "AirbagControlTask", STACK_SIZE_FOR_TASK, &parametersToTask3, TASK_PRIORITY, NULL);
  xTaskCreate(ReadButtonTask, (const char *) "ReadButtonTask", STACK_SIZE_FOR_TASK, &parametersToTask4, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}
