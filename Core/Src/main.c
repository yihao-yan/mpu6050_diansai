/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序
  *   整体流程：MPU6050 每 10ms 数据就绪 -> INT(PB5) 产生上升沿
  *             -> EXTI 中断里仅做计数(ready_cnt++)，不碰 I2C
  *             -> 主循环检测到就绪且 I2C 空闲 -> 启动一次 DMA 突发读(14字节)
  *             -> DMA 硬件搬完数据 -> 完成回调置 dma_done
  *             -> 主循环解析数据、零偏校准、互补滤波、解算姿态角
  *             -> USART1 按 FireWater 协议发往 VOFA+ 显示
  *   硬件：I2C1 PB6=SCL / PB7=SDA；INT=PB5；KEY_UP=PA0；指示灯=PB15
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
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
#define MAX_READY_CNT 8U   /* 数据就绪计数上限：防止干扰脉冲把计数冲爆导致读数混乱 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 原始寄存器数据（有符号16位）与物理量 ---- */
int16_t ax, ay, az;             /* 加速度三轴原始值 */
int16_t gx, gy, gz;             /* 陀螺仪三轴原始值 */
float a_gx, a_gy, a_gz;         /* 加速度物理量，单位 g */
float g_dx, g_dy, g_dz;         /* 角速度物理量，单位 °/s */

/* ---- 零偏校准状态机相关 ---- */
uint8_t calibrate_state = 0;    /* 校准状态：0=正常运行 1=等待3s 2=采样5s */
uint32_t calibrate_tick_start = 0; /* 校准各阶段起始时刻(HAL_GetTick) */
uint8_t calibrate_flag = 0;     /* 校准触发标志：按键置1，主循环检测后进入状态机 */
volatile uint8_t key_int_flag = 0;   /* 按键中断标记：ISR写、主循环读，必须volatile */

/* ---- 数据就绪中断相关 ---- */
volatile uint8_t ready_cnt = 0;      /* 就绪计数：ISR写、主循环读写，必须volatile */
uint8_t mpu_init_ret = 0;            /* MPU6050_Init()返回值，用于串口诊断 */

/* ---- 陀螺仪零偏（静止时的平均角速度）---- */
float offset_gx = 0.0f;
float offset_gy = 0.0f;
float offset_gz = 0.0f;

/* ---- 最终姿态角，单位 度 ---- */
float roll  = 0.0f;            /* 横滚 */
float pitch = 0.0f;            /* 俯仰 */
float yaw   = 0.0f;            /* 偏航 */

uint32_t last_sample_tick = 0;       /* 上一帧采样时刻，用来算真实积分间隔 dt */

/* ---- I2C + DMA 相关 ---- */
uint8_t mpu_dma_buf[MPU6050_RAW_BUF_LEN];  /* DMA接收缓冲：加速度6+温度2+陀螺6=14字节 */
volatile uint8_t dma_busy = 0;             /* DMA传输进行中标志：为1时不再启动新传输 */
volatile uint8_t dma_done = 0;             /* DMA完成标志：回调置1，主循环据此解析数据 */

/* ---- 加速度计算出的姿态角（互补滤波的参考）---- */
float acc_roll  = 0.0f;
float acc_pitch = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 重定向 printf 到 USART1：VOFA 与调试信息都靠它发送 */
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
  /* 注意顺序：DMA 必须在 I2C 之前初始化，否则 I2C 关联的 DMA 句柄未就绪 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化传感器前先关掉 EXTI，避免上电瞬间误触发进入中断 */
  ready_cnt = 0;
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

  mpu_init_ret = MPU6050_Init();   /* 配置量程/采样率/滤波，并使能数据就绪中断 */

  if (mpu_init_ret == 0U)
  {
    printf("MPU6050 init OK (id=0x%02X)\r\n", mpu_whoami_id);

    /* 关键修复：
     * MPU6050 使能数据就绪中断后，INT 引脚在“有数据未读”时保持高电平。
     * 若在 INT 已为高电平时才开上升沿 EXTI，会错过这个上升沿，导致永远
     * 进不了中断 -> ready_cnt 恒为 0 -> 不读数、PB15 不亮。
     * 所以先读一次数据把 INT 拉低，再开上升沿中断。 */
    MPU6050_ReadRawData(&ax, &ay, &az, &gx, &gy, &gz);  /* 阻塞读一次，清掉挂起数据 */

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);                /* 清 EXTI 挂起标志 */
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);              /* 清 NVIC 挂起中断 */
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);                    /* 此时 INT 为低，再使能 */
    printf("Data-ready INT enabled\r\n");
  }
  else
  {
    /* I2C 或传感器初始化失败：PB15 常亮提示，串口打印错误码与总线错误码 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    printf("MPU6050 init FAIL err=%u id=0x%02X ecode=0x%lX\r\n",
           (unsigned)mpu_init_ret, mpu_whoami_id, (unsigned long)hi2c1.ErrorCode);
  }

  last_sample_tick = HAL_GetTick();   /* 记录起始时刻，供首帧计算 dt */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ===== 分支1：DMA 已把一帧数据搬完，进行解析、校准、解算、发送 ===== */
    if (dma_done)
    {
      uint32_t now_tick;
      float dt;

      dma_done = 0;   /* 先清零完成标志，等待下一帧 */

      /* 解析 DMA 读回的 14 字节（跳过中间温度2字节）为原始加速度/陀螺 */
      MPU6050_ParseRawData(mpu_dma_buf, &ax, &ay, &az, &gx, &gy, &gz);

      /* 用真实采样间隔做积分，而不是写死 0.01s */
      now_tick = HAL_GetTick();
      dt = (float)(now_tick - last_sample_tick) / 1000.0f;
      last_sample_tick = now_tick;
      if (dt <= 0.0f || dt > 0.5f)
      {
        dt = 0.01f; /* 首帧或间隔异常时按 100Hz 兜底 */
      }

      /* 原始值换算为物理量（±2g / ±250°/s）：
       * 加速度 16384 LSB/g；陀螺仪 131 LSB/(°/s) */
      a_gx = ax / 16384.0f;
      a_gy = ay / 16384.0f;
      a_gz = az / 16384.0f;
      g_dx = gx / 131.0f;
      g_dy = gy / 131.0f;
      g_dz = gz / 131.0f;

      /* 按键中断标记处理：中断里只置位，真正响应放到这里 */
      if (key_int_flag == 1)
      {
        key_int_flag = 0;
        calibrate_flag = 1;   /* 通知下面的状态机启动校准 */
      }

      /* ========= 陀螺仪零偏校准状态机 ========= */
      /* 触发：进入等待阶段，同时 yaw 归零 */
      if (calibrate_flag == 1)
      {
        calibrate_flag = 0;
        calibrate_state = 1;
        calibrate_tick_start = HAL_GetTick();
        yaw = 0.0f;
        printf("Gyro Cal Start! Keep board still, wait 3s\r\n");
      }

      /* 阶段1：等待3秒，给用户放稳板子的时间 */
      if (calibrate_state == 1)
      {
        if (HAL_GetTick() - calibrate_tick_start >= 3000U)
        {
          /* 3秒到 -> 进入采样阶段，清零累加器准备采集 */
          calibrate_state = 2;
          calibrate_tick_start = HAL_GetTick();
          offset_gx = 0.0f;
          offset_gy = 0.0f;
          offset_gz = 0.0f;
          printf("Sampling 500 samples, no shaking, 5s\r\n");
        }
      }
      /* 阶段2：5秒内累加各帧角速度，结束后除以500得到平均零偏 */
      else if (calibrate_state == 2)
      {
        uint32_t past = HAL_GetTick() - calibrate_tick_start;
        if (past < 5000U)
        {
          /* 采样中：累加静止时的角速度（理想应为0，实际即零偏）*/
          offset_gx += g_dx;
          offset_gy += g_dy;
          offset_gz += g_dz;
        }
        else
        {
          /* 5秒到：取平均得到零偏，回到正常运行状态 */
          offset_gx /= 500.0f;
          offset_gy /= 500.0f;
          offset_gz /= 500.0f;
          printf("offset_gx=%.2f offset_gy=%.2f offset_gz=%.2f\r\n",
                 offset_gx, offset_gy, offset_gz);
          calibrate_state = 0;
        }
      }

      /* ========== 互补滤波姿态解算（仅正常运行时） ========== */
      if (calibrate_state == 0)
      {
        /* 先减去零偏，得到校准后的角速度 */
        float gx_cal = g_dx - offset_gx;
        float gy_cal = g_dy - offset_gy;
        float gz_cal = g_dz - offset_gz;

        /* 加速度计姿态角（互补滤波参考，长期稳定、不漂移）：
         * 用重力方向反推横滚/俯仰，无 yaw（水平面无法用重力区分）*/
        acc_roll  = atan2f(a_gy, a_gz) * 57.29578f;
        acc_pitch = atan2f(-a_gx, sqrtf(a_gy * a_gy + a_gz * a_gz)) * 57.29578f;

        /* 陀螺仪积分（短期响应快、平滑，但会漂移）：gx->roll, gy->pitch, gz->yaw */
        roll  = roll  + gx_cal * dt;
        pitch = pitch + gy_cal * dt;
        yaw   = yaw   + gz_cal * dt;

        /* 互补滤波：98% 陀螺仪(快、平滑) + 2% 加速度计(稳、不漂)，
         * 兼顾动态响应与长期稳定；yaw 无加速度参考故不做修正 */
        roll  = 0.98f * roll  + 0.02f * acc_roll;
        pitch = 0.98f * pitch + 0.02f * acc_pitch;

        /* VOFA+ FireWater 协议输出 roll,pitch,yaw（单位：度），末尾 \n 作为一帧结束 */
        printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
      }
    }
    /* ===== 分支2：有数据就绪且 I2C 空闲，启动一次 DMA 突发读 ===== */
    else if (ready_cnt > 0U && dma_busy == 0U)
    {
      /* 数据就绪且 I2C 空闲：启动 DMA 后 CPU 不等待，数据由硬件搬运 */
      ready_cnt--;
      dma_busy = 1;                       /* 标记占用，防止重复启动 */
      if (MPU6050_StartDMARead(mpu_dma_buf) != HAL_OK)
      {
        dma_busy = 0;   /* 启动失败，下轮就绪会重新尝试 */
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

  /** Configure the internal regulator output voltage
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
/* EXTI 统一回调：EXTI0(PA0按键) 与 EXTI9_5(PB5的MPU就绪) 都会进入这里。
 * 铁律：中断里只做标志/计数，绝不能放 I2C 读写、printf 等慢速阻塞操作。*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // PB5：MPU 数据就绪
  if(GPIO_Pin == GPIO_PIN_5)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); //翻转PB15：正常时以100Hz快闪，指示中断正常
    if(ready_cnt < MAX_READY_CNT)
    {
      ready_cnt++;                          //就绪计数+1，主循环据此启动DMA
    }
  }
  // PA0：KEY_UP 按键，置位后由主循环处理（真正启动校准）
  if(GPIO_Pin == GPIO_PIN_0)
  {
    key_int_flag = 1;
  }
}

/* I2C DMA 接收完成回调：14字节全部搬完后由 HAL 调用。
 * 只清状态、置标志，数据解析放到主循环做。*/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    dma_busy = 0;    /* 传输结束，释放总线 */
    dma_done = 1;    /* 通知主循环：有一帧数据可解析 */
  }
}

/* I2C DMA 接收错误回调：总线异常时调用。
 * 仅释放总线，丢弃本帧，等下一次数据就绪重新启动。*/
void HAL_I2C_MemRxErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    dma_busy = 0;   /* 出错放弃本次，下轮数据就绪会重新启动 */
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
