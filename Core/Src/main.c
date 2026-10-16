/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 基于 STM32F103C8T6 + MPU6050 的姿态解算例程
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * 硬件连接（STM32F103C8T6 最小系统板 / "Blue Pill"  <->  MPU6050 模块）
  * ---------------------------------------------------------------------------
  *   PB6  (I2C1_SCL)  ->  SCL
  *   PB7  (I2C1_SDA)  ->  SDA
  *   3V3              ->  VCC
  *   GND              ->  GND
  *   PA9  (USART1_TX) ->  USB-TTL 模块的 RX   （调试串口，115200 8N1）
  *   PA10 (USART1_RX) ->  USB-TTL 模块的 TX
  *   PC13             ->  板载 LED（低电平点亮），用于状态指示
  *
  * 软件流程
  * ---------------------------------------------------------------------------
  *   1) SystemClock_Config()：HSE 8MHz -> PLL×9 -> SYSCLK 72MHz
  *   2) MX_I2C1_Init()      ：I2C1 400kHz Fast Mode，接 MPU6050
  *   3) MX_TIM2_Init()      ：TIM2 自由运行，1MHz 计数（1 计数 = 1us）
  *   4) MX_USART1_UART_Init()：115200 调试串口
  *   5) MPU6050_Init()      ：唤醒、配置量程与 DLPF、校验 WHO_AM_I
  *   6) MPU6050_CalibrateGyro()：静止标定陀螺零偏
  *   7) 主循环以 100Hz 读取传感器并调用 Attitude_Update() 做姿态解算，
  *      再以 10Hz 通过串口输出欧拉角，供上位机绘图。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
#include "attitude.h"
#include "dbg_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @brief 姿态解算与传感器采样频率（Hz） */
#define APP_SAMPLE_FREQ_HZ       100U
/** @brief 采样周期（微秒） */
#define APP_SAMPLE_PERIOD_US     (1000000U / APP_SAMPLE_FREQ_HZ)
/** @brief 串口输出分频：100Hz / 10 = 10Hz */
#define APP_PRINT_DIVIDER        10U
/** @brief 陀螺仪零偏标定采样点数 */
#define APP_GYRO_CALIB_SAMPLES   500U
/** @brief 姿态解算算法选择：
  *        ATTITUDE_FILTER_MAHONY / ATTITUDE_FILTER_COMPLEMENTARY / ATTITUDE_FILTER_KALMAN */
#define APP_FILTER_DEFAULT       ATTITUDE_FILTER_MAHONY
/** @brief 传感器离线时的重新初始化间隔（毫秒） */
#define APP_REINIT_INTERVAL_MS   2000U
/** @brief LED 心跳周期（毫秒）：在线 500ms，离线告警 100ms */
#define APP_LED_PERIOD_ONLINE_MS 500U
#define APP_LED_PERIOD_OFFLINE_MS 100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static MPU6050_Handle_t s_mpu;              /*!< MPU6050 驱动句柄          */
static Attitude_t       s_attitude;         /*!< 姿态解算句柄              */
static uint8_t          s_mpu_online = 0U;  /*!< 传感器是否在线            */
static uint16_t         s_last_tick_us = 0U;/*!< 上一次采样时刻（TIM2 计数）*/
static uint32_t         s_sample_idx = 0U;  /*!< 采样序号，用于输出分频    */
static uint32_t         s_led_last_ms = 0U; /*!< LED 上次翻转时刻          */
static uint32_t         s_retry_last_ms = 0U;/*!< 上次重试初始化时刻       */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static MPU6050_Status_t APP_InitSensor(void);
static void APP_PrintHeader(void);
static void APP_PrintAttitude(const MPU6050_Data_t *data);
static void APP_Step(float dt);
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
  uint16_t now_us;
  uint16_t delta_us;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 启动 TIM2 自由运行计数器：预分频 72-1 后计数频率为 1MHz（1 计数 = 1us）
     计数器周期为 65535，约 65.5ms 回绕一次。这里利用 16 位无符号减法
     自动处理回绕的特性，直接得到两次采样之间的精确微秒差。 */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 绑定调试串口，此后可直接使用 printf */
  DBG_Init(&huart1);

  (void)printf("\r\n");
  (void)printf("=====================================================\r\n");
  (void)printf(" STM32F103C8T6 + MPU6050 Attitude Estimation (HAL)\r\n");
  (void)printf("=====================================================\r\n");
  (void)printf("SYSCLK   : %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  (void)printf("I2C1     : PB6(SCL) / PB7(SDA) @ 400kHz\r\n");
  (void)printf("USART1   : PA9(TX)  / PA10(RX) @ 115200 8N1\r\n");
  (void)printf("Sampling : %u Hz, output %u Hz\r\n",
               (unsigned)APP_SAMPLE_FREQ_HZ,
               (unsigned)(APP_SAMPLE_FREQ_HZ / APP_PRINT_DIVIDER));
  (void)printf("Filter   : %s\r\n", Attitude_FilterName(APP_FILTER_DEFAULT));
  (void)printf("-----------------------------------------------------\r\n");

  if (APP_InitSensor() == MPU6050_OK)
  {
    APP_PrintHeader();
  }

  /* 记录起始时刻，进入主循环 */
  s_last_tick_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---------- 100Hz 定步长采样节拍（微秒级） ---------- */
    now_us   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    delta_us = (uint16_t)(now_us - s_last_tick_us);   /* 16 位无符号减法，自动处理回绕 */

    if (delta_us >= (uint16_t)APP_SAMPLE_PERIOD_US)
    {
      s_last_tick_us = now_us;
      APP_Step((float)delta_us * 1.0e-6f);            /* 用实测间隔作为积分步长 */
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @note  外部 8MHz 晶振 -> PLL×9 -> 72MHz，APB1 36MHz，APB2 72MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  * @note  仅作为微秒计时基准，不开启中断；APB1 定时器时钟 72MHz，
  *        预分频 71 后为 1MHz，向上计数到 65535 自动回绕。
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level : PC13 板载 LED 低电平点亮，先置高熄灭 */
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  初始化 MPU6050、标定零偏并初始化姿态解算模块
  * @retval MPU6050 驱动状态码
  */
static MPU6050_Status_t APP_InitSensor(void)
{
  MPU6050_Status_t status;
  char b0[12];
  char b1[12];
  char b2[12];

  status = MPU6050_Init(&s_mpu, &hi2c1);
  if (status != MPU6050_OK)
  {
    s_mpu_online = 0U;
    (void)printf("[FAIL] MPU6050 init failed: %s\r\n", MPU6050_StatusStr(status));
    (void)printf("       check wiring: SCL->PB6, SDA->PB7, VCC->3V3, GND->GND\r\n");
    return status;
  }

  s_mpu_online = 1U;

  (void)printf("[ OK ] MPU6050 found at address 0x%02X, WHO_AM_I = 0x%02X%s\r\n",
               (unsigned)(s_mpu.dev_addr >> 1U),
               (unsigned)s_mpu.who_am_i,
               (MPU6050_WhoAmIIsStandard(&s_mpu) != 0U) ? "" : " (compatible clone chip)");
  (void)printf("[ OK ] gyro range +/-2000dps, accel range +/-2g, DLPF 44Hz\r\n");

  (void)printf("[ .. ] calibrating gyroscope, keep the sensor still ...\r\n");
  if (MPU6050_CalibrateGyro(&s_mpu, APP_GYRO_CALIB_SAMPLES) != MPU6050_OK)
  {
    (void)printf("[WARN] gyro calibration failed, bias kept at zero\r\n");
  }
  else
  {
    DBG_FloatToStr(s_mpu.gyro_bias_dps[0], 3, b0, sizeof(b0));
    DBG_FloatToStr(s_mpu.gyro_bias_dps[1], 3, b1, sizeof(b1));
    DBG_FloatToStr(s_mpu.gyro_bias_dps[2], 3, b2, sizeof(b2));
    (void)printf("[ OK ] gyro bias = (%s, %s, %s) dps\r\n", b0, b1, b2);
  }

  /* 初始化姿态解算模块（内部会把四元数复位为单位阵） */
  Attitude_Init(&s_attitude, APP_FILTER_DEFAULT, (float)APP_SAMPLE_FREQ_HZ);

  s_sample_idx = 0U;
  return MPU6050_OK;
}

/**
  * @brief  打印 CSV 表头，方便上位机直接保存为数据文件
  * @note   数据行格式：
  *         $ATT,t_ms,roll,pitch,yaw,gx,gy,gz,ax,ay,az,temp
  */
static void APP_PrintHeader(void)
{
  (void)printf("# $ATT,t_ms,roll_deg,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,temp_c\r\n");
}

/**
  * @brief  通过串口输出一帧姿态数据（10Hz 调用）
  * @param  data : 当前传感器数据，用于同时输出原始物理量
  */
static void APP_PrintAttitude(const MPU6050_Data_t *data)
{
  char roll[12];
  char pitch[12];
  char yaw[12];
  char gx[12];
  char gy[12];
  char gz[12];
  char ax[12];
  char ay[12];
  char az[12];
  char temp[12];
  float r;
  float p;
  float y;

  Attitude_GetEuler(&s_attitude, &r, &p, &y);

  DBG_FloatToStr(r, 2, roll, sizeof(roll));
  DBG_FloatToStr(p, 2, pitch, sizeof(pitch));
  DBG_FloatToStr(y, 2, yaw, sizeof(yaw));

  DBG_FloatToStr(data->gyro_x_dps, 2, gx, sizeof(gx));
  DBG_FloatToStr(data->gyro_y_dps, 2, gy, sizeof(gy));
  DBG_FloatToStr(data->gyro_z_dps, 2, gz, sizeof(gz));

  DBG_FloatToStr(data->accel_x_g, 3, ax, sizeof(ax));
  DBG_FloatToStr(data->accel_y_g, 3, ay, sizeof(ay));
  DBG_FloatToStr(data->accel_z_g, 3, az, sizeof(az));

  DBG_FloatToStr(data->temperature_c, 2, temp, sizeof(temp));

  (void)printf("$ATT,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
               (unsigned long)HAL_GetTick(),
               roll, pitch, yaw,
               gx, gy, gz,
               ax, ay, az,
               temp);
}

/**
  * @brief  单个采样周期内的应用逻辑（100Hz 调用）
  * @param  dt : 距上次调用的实测时间间隔（秒）
  */
static void APP_Step(float dt)
{
  MPU6050_Data_t data;
  uint32_t now_ms = HAL_GetTick();
  uint32_t led_period;

  /* ---------- LED 心跳：在线慢闪，离线快闪 ---------- */
  led_period = (s_mpu_online != 0U) ? APP_LED_PERIOD_ONLINE_MS : APP_LED_PERIOD_OFFLINE_MS;
  if ((now_ms - s_led_last_ms) >= led_period)
  {
    s_led_last_ms = now_ms;
    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
  }

  /* ---------- 传感器离线时周期性重试 ---------- */
  if (s_mpu_online == 0U)
  {
    if ((now_ms - s_retry_last_ms) >= APP_REINIT_INTERVAL_MS)
    {
      s_retry_last_ms = now_ms;
      if (APP_InitSensor() == MPU6050_OK)
      {
        APP_PrintHeader();
      }
    }
    return;
  }

  /* ---------- 读取传感器并做姿态解算 ---------- */
  if (MPU6050_ReadData(&s_mpu, &data) == MPU6050_OK)
  {
    const float gyro[3]  = { data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps };
    const float accel[3] = { data.accel_x_g,  data.accel_y_g,  data.accel_z_g  };

    Attitude_Update(&s_attitude, gyro, accel, dt);

    s_sample_idx++;
    if ((s_sample_idx % APP_PRINT_DIVIDER) == 0U)
    {
      APP_PrintAttitude(&data);
    }
  }
  else
  {
    /* 连续读失败说明总线或器件异常，标记离线，等待重新初始化 */
    s_mpu_online = 0U;
    (void)printf("[FAIL] MPU6050 read error, sensor marked offline\r\n");
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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
