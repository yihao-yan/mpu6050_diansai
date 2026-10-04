#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"

/* 7-bit address 0x68 shifted left -> 8-bit 0xD0 for HAL I2C */
#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_I2C_TIMEOUT 100U   /* ms; keep small so a stuck bus cannot hang the MCU */

extern uint8_t mpu_whoami_id;      /* raw WHO_AM_I value read at init, for diagnostics */

uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadRawData(int16_t *ax, int16_t *ay, int16_t *az,
                            int16_t *gx, int16_t *gy, int16_t *gz);

#endif
