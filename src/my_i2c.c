#include "my_i2c.h"

#include "FreeRTOS.h"
#include "semphr.h"


#include <stdio.h>
#include <stdbool.h>
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"

SemaphoreHandle_t xSemaphore;

int _write(int file, const char *ptr, int len) {
    int x;
    for (x = 0; x < len; x++) {
       ITM_SendChar (*ptr++);
    }
    return (len);
}

int i2c_initSemaphore()
{
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	return xSemaphore != NULL;
}

void BSP_I2C_Init() {

	xSemaphoreTake(xSemaphore, portMAX_DELAY);

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	CMU_ClockEnable(cmuClock_I2C1, true);
	GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
	I2C1->ROUTE = I2C_ROUTE_SDAPEN |
	I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC0;
	I2C_Init(I2C1, &i2cInit);

	xSemaphoreGive(xSemaphore);
}

/**
 * @brief Write register using default I2C bus
 * @param reg register to write
 * @param data data to write
 * @return true on success
 */
bool I2C_WriteRegister(uint8_t reg, uint8_t data, uint8_t device_addr) {

	xSemaphoreTake(xSemaphore, portMAX_DELAY);

	I2C_TransferReturn_TypeDef I2C_Status;
	bool ret_value = false;

	I2C_TransferSeq_TypeDef seq;
	uint8_t dataW[2];

	seq.addr = device_addr;
	seq.flags = I2C_FLAG_WRITE;

	/* Register to write: 0x67 ( INT_FLAT )*/
	dataW[0] = reg;
	dataW[1] = data;

	seq.buf[0].data = dataW;
	seq.buf[0].len = 2;
	I2C_Status = I2C_TransferInit(I2C1, &seq);

	while (I2C_Status == i2cTransferInProgress) {
		I2C_Status = I2C_Transfer(I2C1);
	}

	if (I2C_Status != i2cTransferDone) {
		ret_value = false;
	} else {
		ret_value = true;
	}

	xSemaphoreGive(xSemaphore);
	return ret_value;
}

/**
 * @brief Read register from I2C device
 * @param reg Register to read
 * @param val Value read
 * @return true on success
 */
bool I2C_ReadRegister(uint8_t reg, uint8_t *val, uint8_t device_addr) {

	xSemaphoreTake(xSemaphore, portMAX_DELAY);

	I2C_TransferReturn_TypeDef I2C_Status;
	I2C_TransferSeq_TypeDef seq;
	uint8_t data[2];

	seq.addr = device_addr;
	seq.flags = I2C_FLAG_WRITE_READ;

	seq.buf[0].data = &reg;
	seq.buf[0].len = 1;
	seq.buf[1].data = data;
	seq.buf[1].len = 1;

	I2C_Status = I2C_TransferInit(I2C1, &seq);

	while (I2C_Status == i2cTransferInProgress) {
		I2C_Status = I2C_Transfer(I2C1);
	}

	if (I2C_Status != i2cTransferDone) {
		xSemaphoreGive(xSemaphore);
		return false;
	}

	*val = data[0];

	xSemaphoreGive(xSemaphore);
	return true;
}

bool I2C_Test(uint8_t addr_ga, uint8_t addr_m) {
	uint8_t data_gyro, data_magnet;

	I2C_ReadRegister(0x0F, &data_gyro, addr_ga);
	I2C_ReadRegister(0x0F, &data_magnet, addr_m);

	printf("I2C gyro: %02X \n Valor esperat: %02X \n", data_gyro, 0b01101000);

	printf("I2C Magnet: %02X \n Valor esperat: %02X \n", data_magnet, 0b00111101);

	return data_gyro == 0b01101000 && data_magnet == 0b00111101;
}

