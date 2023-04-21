#ifndef SRC_MY_I2C_H_
#define SRC_MY_I2C_H_

#include <stdio.h>
#include <stdbool.h>

int i2c_initSemaphore();
void BSP_I2C_Init(uint8_t addr);
bool I2C_WriteRegister(uint8_t reg, uint8_t data);
bool I2C_ReadRegister(uint8_t reg, uint8_t *val);
bool I2C_Test();

#endif /* SRC_MY_I2C_H_ */
