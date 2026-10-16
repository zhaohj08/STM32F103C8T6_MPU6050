/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴惯性传感器驱动（基于 STM32 HAL 的 I2C 阻塞收发）
  ******************************************************************************
  * 特性：
  *   - 自动探测 I2C 从机地址（0x68 / 0x69）
  *   - WHO_AM_I 校验，兼容国产 MPU6050 兼容芯片
  *   - 可配置陀螺仪 / 加速度计量程与数字低通滤波器带宽
  *   - 14 字节突发读取（加速度 + 温度 + 角速度）
  *   - 静止状态零偏标定（消除陀螺仪零漂）
  ******************************************************************************
  */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "mpu6050_reg.h"

/* ============================ 类型定义 ============================ */

/** @brief 驱动返回值 */
typedef enum
{
  MPU6050_OK            = 0,  /*!< 操作成功                         */
  MPU6050_ERR_I2C       = 1,  /*!< I2C 通信失败（无应答 / 总线错误）*/
  MPU6050_ERR_NOT_FOUND = 2,  /*!< 未找到器件                       */
  MPU6050_ERR_PARAM     = 3   /*!< 参数非法                         */
} MPU6050_Status_t;

/** @brief 陀螺仪量程 */
typedef enum
{
  MPU6050_GYRO_RANGE_250DPS  = 0,
  MPU6050_GYRO_RANGE_500DPS  = 1,
  MPU6050_GYRO_RANGE_1000DPS = 2,
  MPU6050_GYRO_RANGE_2000DPS = 3
} MPU6050_GyroRange_t;

/** @brief 加速度计量程 */
typedef enum
{
  MPU6050_ACCEL_RANGE_2G  = 0,
  MPU6050_ACCEL_RANGE_4G  = 1,
  MPU6050_ACCEL_RANGE_8G  = 2,
  MPU6050_ACCEL_RANGE_16G = 3
} MPU6050_AccelRange_t;

/** @brief 原始数据（LSB 单位，与寄存器一一对应） */
typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MPU6050_Raw_t;

/** @brief 物理量数据（已换算为工程单位） */
typedef struct
{
  float accel_x_g;    /*!< 加速度 X，单位 g           */
  float accel_y_g;    /*!< 加速度 Y，单位 g           */
  float accel_z_g;    /*!< 加速度 Z，单位 g           */
  float gyro_x_dps;   /*!< 角速度 X，单位 °/s         */
  float gyro_y_dps;   /*!< 角速度 Y，单位 °/s         */
  float gyro_z_dps;   /*!< 角速度 Z，单位 °/s         */
  float temperature_c;/*!< 芯片温度，单位 ℃          */
} MPU6050_Data_t;

/** @brief 驱动句柄 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;              /*!< 关联的 I2C 句柄                */
  uint16_t           dev_addr;          /*!< 8 位格式的从机地址             */
  uint8_t            who_am_i;          /*!< WHO_AM_I 回读值                */
  MPU6050_GyroRange_t  gyro_range;      /*!< 当前陀螺仪量程                 */
  MPU6050_AccelRange_t accel_range;     /*!< 当前加速度计量程               */
  float              gyro_lsb_per_dps;  /*!< 陀螺仪灵敏度 LSB/(°/s)         */
  float              accel_lsb_per_g;   /*!< 加速度计灵敏度 LSB/g           */
  float              gyro_bias_dps[3];  /*!< 陀螺仪零偏（° /s），标定得到   */
  uint8_t            is_calibrated;     /*!< 零偏是否已标定                 */
} MPU6050_Handle_t;

/* ============================ 接口函数 ============================ */

/**
  * @brief  初始化 MPU6050
  * @param  hmpu : 驱动句柄
  * @param  hi2c : 已初始化的 I2C 句柄（本工程为 &hi2c1）
  * @retval MPU6050_OK 表示成功，其它为错误码
  */
MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, I2C_HandleTypeDef *hi2c);

/**
  * @brief  设置陀螺仪量程
  * @param  hmpu  : 驱动句柄
  * @param  range : 量程枚举
  */
MPU6050_Status_t MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu, MPU6050_GyroRange_t range);

/**
  * @brief  设置加速度计量程
  * @param  hmpu  : 驱动句柄
  * @param  range : 量程枚举
  */
MPU6050_Status_t MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu, MPU6050_AccelRange_t range);

/**
  * @brief  设置数字低通滤波器带宽（同时决定陀螺仪输出率）
  * @param  hmpu   : 驱动句柄
  * @param  dlpf   : 带宽宏 MPU6050_DLPF_BW_xxx
  * @param  sample_rate_hz : 期望采样率（Hz），用于计算 SMPLRT_DIV
  */
MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t *hmpu, uint8_t dlpf, uint16_t sample_rate_hz);

/**
  * @brief  读取 14 字节原始数据（加速度 + 温度 + 角速度）
  * @param  hmpu : 驱动句柄
  * @param  raw  : 原始数据结构指针
  */
MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Handle_t *hmpu, MPU6050_Raw_t *raw);

/**
  * @brief  读取并换算为工程单位（g / °/s / ℃），自动扣除陀螺仪零偏
  * @param  hmpu : 驱动句柄
  * @param  data : 输出数据结构指针
  */
MPU6050_Status_t MPU6050_ReadData(MPU6050_Handle_t *hmpu, MPU6050_Data_t *data);

/**
  * @brief  陀螺仪零偏标定（标定期间必须保持传感器静止）
  * @param  hmpu    : 驱动句柄
  * @param  samples : 采样点数，建议 200 ~ 1000
  * @note   标定结果保存在句柄内，MPU6050_ReadData() 会自动扣除。
  */
MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *hmpu, uint16_t samples);

/**
  * @brief  读取单个寄存器
  */
MPU6050_Status_t MPU6050_ReadReg(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *value);

/**
  * @brief  写入单个寄存器
  */
MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t value);

/**
  * @brief  读取寄存器块
  */
MPU6050_Status_t MPU6050_ReadRegs(MPU6050_Handle_t *hmpu, uint8_t reg, uint8_t *buf, uint16_t len);

/**
  * @brief  将错误码翻译为可读字符串（用于串口日志）
  */
const char *MPU6050_StatusStr(MPU6050_Status_t status);

/**
  * @brief  判断 WHO_AM_I 是否为标准值 0x68
  * @retval 1 表示标准 InvenSense 器件，0 表示可能是国产兼容芯片（功能一致，仅 ID 不同）
  */
uint8_t MPU6050_WhoAmIIsStandard(const MPU6050_Handle_t *hmpu);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
