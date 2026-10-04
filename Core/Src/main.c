/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_READY_CNT 8U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t ax, ay, az;
int16_t gx, gy, gz;
float a_gx, a_gy, a_gz;
float g_dx, g_dy, g_dz;

uint8_t calibrate_state = 0;
uint32_t calibrate_tick_start = 0;
uint8_t calibrate_flag = 0;
volatile uint8_t key_int_flag = 0;   /* 按键中断标记：ISR 写，主循环读 */

volatile uint8_t ready_cnt = 0;      /* MPU 数据就绪中断计数：ISR 写，主循环读写 */
uint8_t mpu_init_ret = 0;            /* MPU6050 初始化返回值，用于诊断 */

float offset_gx = 0.0f;
float offset_gy = 0.0f;
float offset_gz = 0.0f;

float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

uint32_t last_sample_tick = 0;       /* 上一次采样时刻，用于计算真实 dt */

float acc_roll  = 0.0f;
float acc_pitch = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化期间先关掉 EXTI，避免误触发 */
  ready_cnt = 0;
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

  mpu_init_ret = MPU6050_Init();

  if (mpu_init_ret == 0U)
  {
    printf("MPU6050 init OK (id=0x%02X)\r\n", mpu_whoami_id);

    /* 关键修复：
     * MPU6050 使能数据就绪中断后，INT 引脚在“有数据未读”时保持高电平。
     * 若在 INT 已为高电平时才开上升沿 EXTI，会错过这个上升沿，导致永远
     * 进不了中断 -> ready_cnt 恒为 0 -> 不读数、PB15 不亮。
     * 所以先读一次数据把 INT 拉低，再开上升沿中断。 */
    MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    printf("Data-ready INT enabled\r\n");
  }
  else
  {
    /* I2C 或传感器初始化失败：PB15 常亮提示，串口打印错误码 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    printf("MPU6050 init FAIL err=%u id=0x%02X\r\n",
           (unsigned)mpu_init_ret, mpu_whoami_id);
  }

  last_sample_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (ready_cnt > 0U)
    {
      uint32_t now_tick;
      float dt;

      /* I2C 读取放在主循环，禁止放中断回调（HAL I2C 不能在 EXTI 回调里跑） */
      MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz);
      ready_cnt--;

      /* 用真实采样间隔做积分，而不是写死 0.01s */
      now_tick = HAL_GetTick();
      dt = (float)(now_tick - last_sample_tick) / 1000.0f;
      last_sample_tick = now_tick;
      if (dt <= 0.0f || dt > 0.5f)
      {
        dt = 0.01f; /* 首帧或间隔异常时按 100Hz 兜底 */
      }

      /* 原始值换算为物理量（±2g / ±250°/s） */
      a_gx = ax / 16384.0f;
      a_gy = ay / 16384.0f;
      a_gz = az / 16384.0f;
      g_dx = gx / 131.0f;
      g_dy = gy / 131.0f;
      g_dz = gz / 131.0f;

      /* 按键中断标记处理 */
      if (key_int_flag == 1)
      {
        key_int_flag = 0;
        calibrate_flag = 1;
      }

      /* ========= 陀螺仪零偏校准状态机 ========= */
      if (calibrate_flag == 1)
      {
        calibrate_flag = 0;
        calibrate_state = 1;
        calibrate_tick_start = HAL_GetTick();
        yaw = 0.0f;
        printf("Gyro Cal Start! Keep board still, wait 3s\r\n");
      }

      if (calibrate_state == 1)
      {
        if (HAL_GetTick() - calibrate_tick_start >= 3000U)
        {
          calibrate_state = 2;
          calibrate_tick_start = HAL_GetTick();
          offset_gx = 0.0f;
          offset_gy = 0.0f;
          offset_gz = 0.0f;
          printf("Sampling 500 samples, no shaking, 5s\r\n");
        }
      }
      else if (calibrate_state == 2)
      {
        uint32_t past = HAL_GetTick() - calibrate_tick_start;
        if (past < 5000U)
        {
          offset_gx += g_dx;
          offset_gy += g_dy;
          offset_gz += g_dz;
        }
        else
        {
          offset_gx /= 500.0f;
          offset_gy /= 500.0f;
          offset_gz /= 500.0f;
          printf("offset_gx=%.2f offset_gy=%.2f offset_gz=%.2f\r\n",
                 offset_gx, offset_gy, offset_gz);
          calibrate_state = 0;
        }
      }

      /* ========== 互补滤波姿态解算 ========== */
      if (calibrate_state == 0)
      {
        float gx_cal = g_dx - offset_gx;
        float gy_cal = g_dy - offset_gy;
        float gz_cal = g_dz - offset_gz;

        /* 加速度计姿态角（互补滤波参考） */
        acc_roll  = atan2f(a_gy, a_gz) * 57.29578f;
        acc_pitch = atan2f(-a_gx, sqrtf(a_gy * a_gy + a_gz * a_gz)) * 57.29578f;

        /* 陀螺仪积分：gx->roll, gy->pitch, gz->yaw */
        roll  = roll  + gx_cal * dt;
        pitch = pitch + gy_cal * dt;
        yaw   = yaw   + gz_cal * dt;

        /* 互补滤波：0.98 陀螺仪 + 0.02 加速度计 */
        roll  = 0.98f * roll  + 0.02f * acc_roll;
        pitch = 0.98f * pitch + 0.02f * acc_pitch;

        /* VOFA+ FireWater 协议输出 roll,pitch,yaw（单位：度） */
        printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // PB5 MPU INT
  if(GPIO_Pin == GPIO_PIN_5)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); //翻转PB15
    if(ready_cnt < MAX_READY_CNT)
    {
      ready_cnt++;
    }
  }
  //PA0按键
  if(GPIO_Pin == GPIO_PIN_0)
  {
    key_int_flag = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
