#include "mpu6050.h"
#include "i2c.h"

uint8_t mpu_whoami_id = 0;

/* Write one register */
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &data, 1,
                           MPU6050_I2C_TIMEOUT);
}

/* Read 'len' consecutive registers into buf */
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len,
                          MPU6050_I2C_TIMEOUT);
}

uint8_t MPU6050_Init(void)
{
  uint8_t id = 0;
  uint8_t val;

  /* Wake up: clear SLEEP bit */
  if (MPU6050_WriteReg(0x6B, 0x00) != HAL_OK)
  {
    return 1;
  }

  /* Check WHO_AM_I */
  if (MPU6050_ReadReg(0x75, &id, 1) != HAL_OK)
  {
    return 2;
  }
  mpu_whoami_id = id;

  /* 0x68 = MPU6050.
   * 0x98/0x12/0x70/0x71/0x73 are register-compatible chips often found on
   * "MPU6050" modules (ICM-20689, ICM-20602, MPU6500/9250/9255): accel/gyro/
   * INT register map is identical for the basic reads we use. */
  if (id != 0x68 && id != 0x98 && id != 0x12 &&
      id != 0x70 && id != 0x71 && id != 0x73)
  {
    return 3;
  }

  /* Sample rate = 1kHz / (1 + 9) = 100Hz */
  MPU6050_WriteReg(0x19, 9);
  /* DLPF = 3 */
  MPU6050_WriteReg(0x1A, 0x03);
  /* Accel full scale +-2g */
  MPU6050_WriteReg(0x1C, 0x00);
  /* Gyro full scale +-250 deg/s */
  MPU6050_WriteReg(0x1B, 0x00);

  /* INT_PIN_CFG: active-high, push-pull (default) */
  val = 0x00;
  MPU6050_WriteReg(0x37, val);
  /* INT_ENABLE: enable data-ready interrupt */
  val = 0x01;
  MPU6050_WriteReg(0x38, val);

  return 0;
}

/* Read 3-axis accel + gyro raw data */
uint8_t MPU6050_ReadRawData(int16_t *ax, int16_t *ay, int16_t *az,
                            int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[6];

  /* Accel: 0x3B..0x40, big-endian high/low bytes */
  if (MPU6050_ReadReg(0x3B, buf, 6) != HAL_OK)
  {
    return 1;
  }
  *ax = (int16_t)((buf[0] << 8) | buf[1]);
  *ay = (int16_t)((buf[2] << 8) | buf[3]);
  *az = (int16_t)((buf[4] << 8) | buf[5]);

  /* Gyro: 0x43..0x48 */
  if (MPU6050_ReadReg(0x43, buf, 6) != HAL_OK)
  {
    return 1;
  }
  *gx = (int16_t)((buf[0] << 8) | buf[1]);
  *gy = (int16_t)((buf[2] << 8) | buf[3]);
  *gz = (int16_t)((buf[4] << 8) | buf[5]);

  return 0;
}
