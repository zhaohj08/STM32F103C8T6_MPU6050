/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴惯性传感器驱动实现
  ******************************************************************************
  * 数据流：I2C1(400kHz) -> HAL_I2C_Mem_Read -> 14 字节原始数据
  *         -> 换算为 g / °/s / ℃ -> 交给姿态解算模块
  ******************************************************************************
  */

#include "mpu6050.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/** @brief 单次 I2C 事务超时（毫秒） */
#define MPU6050_I2C_TIMEOUT_MS      50U

/** @brief 软复位 / 时钟切换后的等待时间 */
#define MPU6050_RESET_DELAY_MS      100U
#define MPU6050_CLKSWITCH_DELAY_MS  50U

/** @brief 零偏标定前丢弃的样本数（等待内部滤波器稳定） */
#define MPU6050_CALIB_DISCARD       50U

/* Private function prototypes -----------------------------------------------*/
static MPU6050_Status_t MPU6050_WriteRegInternal(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t value);
static MPU6050_Status_t MPU6050_ReadRegsInternal(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *buf, uint16_t len);
static void MPU6050_UpdateSensitivity(MPU6050_Handle_t *hmpu);

/* Exported functions --------------------------------------------------------*/

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c)
{
  uint8_t who = 0U;
  uint8_t value = 0U;

  if ((hmpu == NULL) || (hi2c == NULL))
  {
    return MPU6050_ERR_PARAM;
  }

  (void)memset(hmpu, 0, sizeof(MPU6050_Handle_t));
  hmpu->hi2c        = hi2c;
  hmpu->dev_addr    = MPU6050_I2C_ADDR_LOW;
  hmpu->gyro_range  = MPU6050_GYRO_RANGE_2000DPS;
  hmpu->accel_range = MPU6050_ACCEL_RANGE_2G;
  MPU6050_UpdateSensitivity(hmpu);

  /* -------- 第 1 步：探测从机地址（AD0 接地 0x68 / 接高 0x69） -------- */
  if (MPU6050_ReadRegsInternal(hmpu, MPU6050_REG_WHO_AM_I, &who, 1U) != MPU6050_OK)
  {
    hmpu->dev_addr = MPU6050_I2C_ADDR_HIGH;
    if (MPU6050_ReadRegsInternal(hmpu, MPU6050_REG_WHO_AM_I, &who, 1U) != MPU6050_OK)
    {
      hmpu->dev_addr = MPU6050_I2C_ADDR_LOW;
      return MPU6050_ERR_NOT_FOUND;
    }
  }
  hmpu->who_am_i = who;

  /* -------- 第 2 步：软复位 -------- */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  HAL_Delay(MPU6050_RESET_DELAY_MS);

  /* -------- 第 3 步：唤醒并选择 PLL(X 轴陀螺) 作为时钟源 --------
   *  复位后芯片默认处于 Sleep 状态，必须清除 SLEEP 位；
   *  使用 PLL 时钟可以获得比内部 RC 更稳定的采样节拍。
   */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_PWR_MGMT_1,
                               (uint8_t)(MPU6050_CLKSEL_PLL_XGYRO | MPU6050_CLKSEL_STOP_WHEN_UNSTABLE)) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  HAL_Delay(MPU6050_CLKSWITCH_DELAY_MS);

  /* 复位后回读校验：SLEEP 位为 0 才认为芯片真正进入工作状态 */
  if (MPU6050_ReadRegsInternal(hmpu, MPU6050_REG_PWR_MGMT_1, &value, 1U) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  if ((value & MPU6050_PWR1_SLEEP) != 0U)
  {
    return MPU6050_ERR_NOT_FOUND;
  }

  /* -------- 第 4 步：使能全部 6 个轴（PWR_MGMT_2 = 0） -------- */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_PWR_MGMT_2, 0x00U) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* -------- 第 5 步：关闭 FIFO 与 I2C 主机模式，直接访问寄存器 -------- */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_USER_CTRL, 0x00U) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* -------- 第 6 步：复位信号通路（清空采样寄存器） -------- */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_SIGNAL_PATH_RST,
                               (uint8_t)(MPU6050_SIGPATH_GYRO_RESET |
                                         MPU6050_SIGPATH_ACCEL_RESET |
                                         MPU6050_SIGPATH_TEMP_RESET)) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  HAL_Delay(10U);

  /* -------- 第 7 步：中断引脚配置（推挽 / 高有效 / 50us 脉冲 / 读寄存器清标志） --------
   *  本工程采用轮询方式读取，保留 INT 配置是为了后续扩展 EXTI 中断采集。
   */
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_INT_PIN_CFG, MPU6050_INT_RD_CLEAR) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_INT_ENABLE, 0x00U) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* -------- 第 8 步：量程与滤波带宽 -------- */
  if (MPU6050_SetGyroRange(hmpu, MPU6050_GYRO_RANGE_2000DPS) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  if (MPU6050_SetAccelRange(hmpu, MPU6050_ACCEL_RANGE_2G) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }
  /* DLPF = 44Hz，采样率 100Hz */
  if (MPU6050_SetDLPF(hmpu, MPU6050_DLPF_BW_44HZ, 100U) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  hmpu->is_calibrated = 0U;
  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu, MPU6050_GyroRange_t range)
{
  uint8_t reg_val;

  if (hmpu == NULL)
  {
    return MPU6050_ERR_PARAM;
  }
  if ((uint8_t)range > (uint8_t)MPU6050_GYRO_RANGE_2000DPS)
  {
    return MPU6050_ERR_PARAM;
  }

  reg_val = (uint8_t)(((uint8_t)range) << 3);

  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_GYRO_CONFIG, reg_val) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  hmpu->gyro_range = range;
  MPU6050_UpdateSensitivity(hmpu);
  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu, MPU6050_AccelRange_t range)
{
  uint8_t reg_val;

  if (hmpu == NULL)
  {
    return MPU6050_ERR_PARAM;
  }
  if ((uint8_t)range > (uint8_t)MPU6050_ACCEL_RANGE_16G)
  {
    return MPU6050_ERR_PARAM;
  }

  reg_val = (uint8_t)(((uint8_t)range) << 3);

  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_ACCEL_CONFIG, reg_val) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  hmpu->accel_range = range;
  MPU6050_UpdateSensitivity(hmpu);
  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t *hmpu, uint8_t dlpf, uint16_t sample_rate_hz)
{
  uint16_t gyro_output_rate;
  uint16_t divider;

  if ((hmpu == NULL) || (sample_rate_hz == 0U))
  {
    return MPU6050_ERR_PARAM;
  }
  if (dlpf > MPU6050_DLPF_BW_5HZ)
  {
    return MPU6050_ERR_PARAM;
  }

  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_CONFIG, (uint8_t)(dlpf & 0x07U)) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* DLPF_CFG = 0 时陀螺仪输出率为 8kHz，其余档位均被降采样到 1kHz */
  gyro_output_rate = (dlpf == MPU6050_DLPF_BW_260HZ) ? 8000U : 1000U;

  divider = (uint16_t)(gyro_output_rate / sample_rate_hz);
  if (divider > 0U)
  {
    divider -= 1U;
  }
  if (divider > 255U)
  {
    divider = 255U;
  }

  if (MPU6050_WriteRegInternal(hmpu, MPU6050_REG_SMPLRT_DIV, (uint8_t)divider) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Handle_t *hmpu, MPU6050_Raw_t *raw)
{
  uint8_t buf[MPU6050_BURST_LEN];

  if ((hmpu == NULL) || (raw == NULL))
  {
    return MPU6050_ERR_PARAM;
  }

  /* 一次性突发读取 14 字节，保证同一采样时刻的数据来自同一帧 */
  if (MPU6050_ReadRegsInternal(hmpu, MPU6050_REG_BURST_START, buf, MPU6050_BURST_LEN) != MPU6050_OK)
  {
    return MPU6050_ERR_I2C;
  }

  raw->accel_x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
  raw->accel_y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
  raw->accel_z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
  raw->temp    = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
  raw->gyro_x  = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
  raw->gyro_y  = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
  raw->gyro_z  = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadData(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data)
{
  MPU6050_Raw_t raw;
  MPU6050_Status_t status;

  if ((hmpu == NULL) || (data == NULL))
  {
    return MPU6050_ERR_PARAM;
  }

  status = MPU6050_ReadRaw(hmpu, &raw);
  if (status != MPU6050_OK)
  {
    return status;
  }

  data->accel_x_g = (float)raw.accel_x / hmpu->accel_lsb_per_g;
  data->accel_y_g = (float)raw.accel_y / hmpu->accel_lsb_per_g;
  data->accel_z_g = (float)raw.accel_z / hmpu->accel_lsb_per_g;

  data->gyro_x_dps = (float)raw.gyro_x / hmpu->gyro_lsb_per_dps - hmpu->gyro_bias_dps[0];
  data->gyro_y_dps = (float)raw.gyro_y / hmpu->gyro_lsb_per_dps - hmpu->gyro_bias_dps[1];
  data->gyro_z_dps = (float)raw.gyro_z / hmpu->gyro_lsb_per_dps - hmpu->gyro_bias_dps[2];

  /* 数据手册给出的温度换算公式 */
  data->temperature_c = ((float)raw.temp / 340.0f) + 36.53f;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *hmpu, uint16_t samples)
{
  MPU6050_Raw_t raw;
  int32_t sum_x = 0;
  int32_t sum_y = 0;
  int32_t sum_z = 0;
  uint16_t i;
  uint16_t valid = 0U;

  if (hmpu == NULL)
  {
    return MPU6050_ERR_PARAM;
  }
  if (samples == 0U)
  {
    return MPU6050_ERR_PARAM;
  }

  /* 丢弃前若干个样本，等内部 DLPF 与采样节拍稳定 */
  for (i = 0U; i < MPU6050_CALIB_DISCARD; i++)
  {
    (void)MPU6050_ReadRaw(hmpu, &raw);
    HAL_Delay(2U);
  }

  for (i = 0U; i < samples; i++)
  {
    if (MPU6050_ReadRaw(hmpu, &raw) != MPU6050_OK)
    {
      return MPU6050_ERR_I2C;
    }
    sum_x += (int32_t)raw.gyro_x;
    sum_y += (int32_t)raw.gyro_y;
    sum_z += (int32_t)raw.gyro_z;
    valid++;
    HAL_Delay(2U);
  }

  if (valid == 0U)
  {
    return MPU6050_ERR_I2C;
  }

  hmpu->gyro_bias_dps[0] = ((float)sum_x / (float)valid) / hmpu->gyro_lsb_per_dps;
  hmpu->gyro_bias_dps[1] = ((float)sum_y / (float)valid) / hmpu->gyro_lsb_per_dps;
  hmpu->gyro_bias_dps[2] = ((float)sum_z / (float)valid) / hmpu->gyro_lsb_per_dps;
  hmpu->is_calibrated = 1U;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadReg(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *value)
{
  if ((hmpu == NULL) || (value == NULL))
  {
    return MPU6050_ERR_PARAM;
  }
  return MPU6050_ReadRegsInternal(hmpu, reg, value, 1U);
}

MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t value)
{
  if (hmpu == NULL)
  {
    return MPU6050_ERR_PARAM;
  }
  return MPU6050_WriteRegInternal(hmpu, reg, value);
}

MPU6050_Status_t MPU6050_ReadRegs(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if ((hmpu == NULL) || (buf == NULL) || (len == 0U))
  {
    return MPU6050_ERR_PARAM;
  }
  return MPU6050_ReadRegsInternal(hmpu, reg, buf, len);
}

const char *MPU6050_StatusStr(MPU6050_Status_t status)
{
  switch (status)
  {
    case MPU6050_OK:            return "OK";
    case MPU6050_ERR_I2C:       return "I2C communication error";
    case MPU6050_ERR_NOT_FOUND: return "device not found";
    case MPU6050_ERR_PARAM:     return "invalid parameter";
    default:                    return "unknown error";
  }
}

/* Private functions ---------------------------------------------------------*/

static MPU6050_Status_t MPU6050_WriteRegInternal(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t value)
{
  if (HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                        &value, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }
  return MPU6050_OK;
}

static MPU6050_Status_t MPU6050_ReadRegsInternal(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                       buf, len, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }
  return MPU6050_OK;
}

static void MPU6050_UpdateSensitivity(MPU6050_Handle_t *hmpu)
{
  /* 陀螺仪灵敏度 LSB/(°/s)：250 / 500 / 1000 / 2000 dps */
  static const float gyro_sens[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

  /* 加速度计灵敏度 LSB/g：2 / 4 / 8 / 16 g */
  static const float accel_sens[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };

  hmpu->gyro_lsb_per_dps = gyro_sens[(uint8_t)hmpu->gyro_range];
  hmpu->accel_lsb_per_g  = accel_sens[(uint8_t)hmpu->accel_range];
}

uint8_t MPU6050_WhoAmIIsStandard(const MPU6050_Handle_t *hmpu)
{
  if (hmpu == NULL)
  {
    return 0U;
  }
  return (hmpu->who_am_i == MPU6050_WHO_AM_I_VALUE) ? 1U : 0U;
}
