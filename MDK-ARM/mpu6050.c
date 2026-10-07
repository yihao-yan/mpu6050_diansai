#include "mpu6050.h"   // 本模块头文件：地址、宏、函数声明都在这里
#include "i2c.h"       // CubeMX生成，提供hi2c1，I2C硬件操作必须

uint8_t mpu_whoami_id = 0;   // 保存读到的芯片ID，给main函数串口打印，排查硬件

/**
 * @brief  向MPU6050【单个寄存器】写1字节，用来配置传感器
 * @param  reg: MPU内部寄存器地址(如0x6B)
 * @param  data: 要写入的8位配置数值
 * @retval HAL库状态：HAL_OK=成功；其他=I2C通信出错
 */
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    /* HAL_I2C_Mem_Write(I2C句柄,器件I2C地址,寄存器号,寄存器字节宽度,待写数据,写几个字节,超时ms) */
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,      // MPU的I2C设备地址 (0x68 <<1)
                             reg,               // 要操作的MPU内部寄存器
                             I2C_MEMADD_SIZE_8BIT, // MPU寄存器是1字节(8bit)
                             &data,             // 要发送的数据的内存地址
                             1,                 // 只写1个字节
                             MPU6050_I2C_TIMEOUT); // I2C超时时间，超时直接返回，不死等
}

/**
 * @brief  从MPU某寄存器开始，【连续读len个字节】存入buf数组
 * @note   MPU硬件支持寄存器地址自动累加，读一次就能连续读出多组数据，效率高
 * @param  reg: 读取的起始寄存器编号
 * @param  buf: uint8_t数组，读到的字节全部存在这里
 * @param  len: 需要读取多少字节
 * @retval HAL_OK成功；其他代表I2C读失败
 */
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* HAL_I2C_Mem_Read：I2C外设读取寄存器 */
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,     // 读到的数据存到这个数组
                            len,     // 读多少字节
                            MPU6050_I2C_TIMEOUT);
}

/**
 * @brief  MPU6050初始化函数
 * @note   MPU上电默认休眠，需要唤醒；配置采样率、滤波、量程；开启数据就绪中断
 * @retval 0:初始化成功
 *         1:写唤醒寄存器0x6B失败(I2C写不通)
 *         2:读ID寄存器0x75失败(I2C读不通)
 *         3:读到的芯片ID不在兼容白名单，不是可用的传感器
 */
uint8_t MPU6050_Init(void)
{
  uint8_t id = 0;
  uint8_t val;

  /* 寄存器0x6B(PWR_MGMT_1)：唤醒芯片。MPU上电默认睡眠，不采集数据，必须写0唤醒 */
  if (MPU6050_WriteReg(0x6B, 0x00) != HAL_OK)
  {
    return 1;   // 写失败，直接返回错误码1
  }

  /* 寄存器0x75 WHO_AM_I：芯片ID寄存器，用来确认I2C通信正常、芯片真实存在 */
  if (MPU6050_ReadReg(0x75, &id, 1) != HAL_OK)
  {
    return 2;   // 读ID失败，返回错误码2
  }
  mpu_whoami_id = id;  // 把读到的ID保存给全局变量，main可以打印看是什么芯片

  /* ID白名单：市面上很多仿MPU6050模块，芯片不是原版MPU6050，但是寄存器完全兼容，可以直接用
   * 0x68  = MPU6050原版
   * 0x98  = ICM‑20689（最常见的克隆）
   * 0x12  = ICM‑20602
   * 0x70/0x71/0x73 = MPU6500 / MPU9250
   * 如果读到ID不在列表，判定硬件异常，返回错误码3
   */
  if (id != 0x68 && id != 0x98 && id != 0x12 &&
      id != 0x70 && id != 0x71 && id != 0x73)
  {
    return 3;
  }

  /* 寄存器0x19 SMPLRT_DIV 采样率分频器
   * MPU内部基础输出1000Hz；采样率 = 1000Hz / (1 + 分频值)
   * 写9 → 1000/(1+9)=100Hz，每10ms输出一帧传感器数据 */
  MPU6050_WriteReg(0x19, 9);

  /* 寄存器0x1A CONFIG，DLPF数字低通滤波设置
   * DLPF=3，滤除高频震动噪声，让采集的数据更平滑 */
  MPU6050_WriteReg(0x1A, 0x03);

  /* 寄存器0x1C ACCEL_CONFIG 加速度计量程配置
   * 0x00代表±2g；灵敏度：16384 LSB/g，也就是数字16384代表1个g重力 */
  MPU6050_WriteReg(0x1C, 0x00);

  /* 寄存器0x1B GYRO_CONFIG 陀螺仪量程配置
   * 0x00代表 ±250°/s；灵敏度131 LSB/(°/s)，数字131代表每秒转动1度 */
  MPU6050_WriteReg(0x1B, 0x00);

  /* 寄存器0x37 INT_PIN_CFG：INT中断引脚配置
   * val=0x00：高电平有效、推挽输出，不锁存；MPU采完数据INT脚输出高电平 */
  val = 0x00;
  MPU6050_WriteReg(0x37, val);

  /* 寄存器0x38 INT_ENABLE：中断使能寄存器
   * val=0x01，打开DATA_RDY(数据就绪中断)：每采集完一帧新数据，INT引脚电平跳变，通知STM32 */
  val = 0x01;
  MPU6050_WriteReg(0x38, val);

  return 0;  // 全部配置完成，返回成功
}

/**
 * @brief 【阻塞式读取】加速度+陀螺仪原始数据
 * @note  会卡住CPU等待I2C完成！！只在初始化阶段调用一次，**主循环DMA流程不用这个函数**
 *        用途：初始化时读一次，把MPU的INT中断引脚电平拉低，解决上电中断误触发bug
 * @param  ax ay az: 传出加速度原始16位数值
 * @param  gx gy gz: 传出陀螺仪原始16位数值
 * @retval 0成功；1 I2C读失败
 */
uint8_t MPU6050_ReadRawData(int16_t *ax, int16_t *ay, int16_t *az,
                            int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[6];   // 临时字节缓存，存放I2C读到的原始字节

  /* 加速度寄存器起始地址0x3B，连续读6字节：AX_H,AX_L,AY_H,AY_L,AZ_H,AZ_L */
  if (MPU6050_ReadReg(0x3B, buf, 6) != HAL_OK)
  {
    return 1;
  }
  /* MPU数据是大端模式：高字节在前，低字节在后
   * buf[0] = X加速度高8位；buf[1] = X加速度低8位
   * (buf[0] <<8) | buf[1]：把高低8位拼接成完整16位有符号数字 */
  *ax = (int16_t)((buf[0] << 8) | buf[1]);
  *ay = (int16_t)((buf[2] << 8) | buf[3]);
  *az = (int16_t)((buf[4] << 8) | buf[5]);

  /* 陀螺仪寄存器起始地址0x43，连续读6字节 GX_H GX_L GY_H GY_L GZ_H GZ_L */
  if (MPU6050_ReadReg(0x43, buf, 6) != HAL_OK)
  {
    return 1;
  }
  *gx = (int16_t)((buf[0] << 8) | buf[1]);
  *gy = (int16_t)((buf[2] << 8) | buf[3]);
  *gz = (int16_t)((buf[4] << 8) | buf[5]);

  return 0;
}

/**
 * @brief 启动I2C的DMA读取，从0x3B开始一次性读14字节
 * @note  【只下达启动命令，函数立刻返回，CPU不等数据！】
 *        14字节内容：加速度6字节 + 温度2字节 + 陀螺仪6字节
 *        DMA硬件在后台搬运数据，搬运完成后触发DMA中断回调通知main函数
 * @param  buf: 目标数组，DMA硬件会把读到的14字节全部放到这个数组
 * @retval HAL_OK代表DMA任务启动成功；其他=启动失败
 */
uint8_t MPU6050_StartDMARead(uint8_t *buf)
{
    /* HAL_I2C_Mem_Read_DMA：I2C的DMA模式读取函数 */
    return HAL_I2C_Mem_Read_DMA(&hi2c1,
                                MPU6050_ADDR,
                                0x3B,                 // 读取起始寄存器地址
                                I2C_MEMADD_SIZE_8BIT,
                                buf,                  // DMA搬运的目标内存数组
                                MPU6050_RAW_BUF_LEN); // 需要读的总字节数14
}

/**
 * @brief 解析DMA读到的14字节原始缓冲区
 * @note 纯内存运算！！不操作I2C硬件。DMA已经把字节放到buf数组，本函数只做高低字节拼接
 * @param  buf：DMA收到的14字节原始缓冲区
 * @param  ax,ay,az：输出加速度16位原始数
 * @param  gx,gy,gz：输出陀螺仪16位原始数
 */
void MPU6050_ParseRawData(const uint8_t *buf, int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz)
{
    /* buf[0]~buf[5]：加速度6字节，拼接为ax ay az */
    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);

    /* buf[6] buf[7] 是芯片温度，本工程不用，直接跳过 */

    /* buf[8]~buf[13]：陀螺仪6字节，拼接gx gy gz */
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}
