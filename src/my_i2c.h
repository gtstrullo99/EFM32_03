#ifndef SRC_MY_I2C_H_
#define SRC_MY_I2C_H_

#include <stdio.h>
#include <stdbool.h>

int i2c_initSemaphore();
void BSP_I2C_Init();
bool I2C_WriteRegister(uint8_t reg, uint8_t data, uint8_t device_addr);
bool I2C_ReadRegister(uint8_t reg, uint8_t *val, uint8_t device_addr);
bool I2C_Test(uint8_t addr_ga, uint8_t addr_m);

#endif /* SRC_MY_I2C_H_ */
