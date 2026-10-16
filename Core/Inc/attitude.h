/**
  ******************************************************************************
  * @file    attitude.h
  * @brief   姿态解算：Mahony 四元数 AHRS / 互补滤波 / 一阶卡尔曼滤波
  ******************************************************************************
  * 坐标系约定（与 MPU6050 芯片丝印方向一致，右手系）：
  *   - 传感器水平静止放置时：Z 轴朝上，重力矢量为 (0, 0, +1g)
  *   - Roll  (横滚角 φ)：绕 X 轴旋转，水平时约为 0°，范围 (-180°, +180°]
  *   - Pitch (俯仰角 θ)：绕 Y 轴旋转，水平时约为 0°，范围 [-90°, +90°]
  *   - Yaw   (航向角 ψ)：绕 Z 轴旋转，无磁力计时只能靠陀螺积分，长时间会漂移
  ******************************************************************************
  * 三种算法说明：
  *   1) ATTITUDE_FILTER_MAHONY
  *      Mahony 提出的非线性互补滤波，用加速度计校正四元数的俯仰/横滚，
  *      通过 PI 控制器补偿陀螺仪漂移，输出完整四元数与欧拉角。
  *      优点：全程无奇异点、动态响应好，是目前工程上最常用的方案。
  *   2) ATTITUDE_FILTER_COMPLEMENTARY
  *      一阶互补滤波：高通陀螺仪积分 + 低通加速度计解算角。
  *      优点：计算量极小，参数直观（一个时间常数），适合入门理解原理。
  *   3) ATTITUDE_FILTER_KALMAN
  *      对 Roll / Pitch 分别做一维卡尔曼滤波，状态为 [角度, 陀螺零偏]。
  *      优点：理论上最优估计，可在线估计陀螺零偏；Yaw 仍为陀螺积分。
  ******************************************************************************
  */

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 姿态解算算法选择 */
typedef enum
{
  ATTITUDE_FILTER_MAHONY        = 0,  /*!< Mahony 四元数 AHRS（推荐）   */
  ATTITUDE_FILTER_COMPLEMENTARY = 1,  /*!< 一阶互补滤波                 */
  ATTITUDE_FILTER_KALMAN        = 2   /*!< 一维卡尔曼滤波（Roll/Pitch） */
} AttitudeFilter_t;

/** @brief 一维卡尔曼滤波器（状态：角度 + 陀螺零偏） */
typedef struct
{
  float q_angle;      /*!< 角度过程噪声协方差      */
  float q_bias;       /*!< 零偏过程噪声协方差      */
  float r_measure;    /*!< 测量噪声协方差          */
  float angle;        /*!< 估计角度（°）           */
  float bias;         /*!< 估计陀螺零偏（°/s）     */
  float p[2][2];      /*!< 误差协方差矩阵          */
} Attitude_Kalman1D_t;

/** @brief 姿态解算句柄 */
typedef struct
{
  AttitudeFilter_t filter;              /*!< 当前使用的算法             */

  /* --- Mahony 四元数 AHRS --- */
  float q[4];                           /*!< 四元数 (w, x, y, z)        */
  float integral_fb[3];                 /*!< 误差积分反馈               */
  float two_kp;                         /*!< 比例增益 ×2（默认 1.0）    */
  float two_ki;                         /*!< 积分增益 ×2（默认 0.0）    */

  /* --- 互补滤波 --- */
  float comp_angle[2];                  /*!< 互补滤波累积角 (roll, pitch) */
  float comp_tau;                       /*!< 时间常数（秒），默认 0.5s  */

  /* --- 航向角积分器（互补滤波 / 卡尔曼共用） --- */
  float yaw_deg;                        /*!< 陀螺积分航向角（°），无磁力计只能靠积分 */

  /* --- 卡尔曼滤波 --- */
  Attitude_Kalman1D_t k_roll;
  Attitude_Kalman1D_t k_pitch;

  /* --- 公共 --- */
  float sample_freq_hz;                 /*!< 标称采样率，仅用于初始化参数 */
  float euler_deg[3];                   /*!< 输出欧拉角 (roll, pitch, yaw) */
  uint32_t update_cnt;                  /*!< 已解算次数                 */
} Attitude_t;

/* ============================ 接口函数 ============================ */

/**
  * @brief  初始化姿态解算模块
  * @param  att          : 句柄
  * @param  filter       : 算法选择
  * @param  sample_freq_hz : 标称采样率（Hz）
  */
void Attitude_Init(Attitude_t *att, AttitudeFilter_t filter, float sample_freq_hz);

/**
  * @brief  复位姿态（四元数归为单位阵，欧拉角清零）
  */
void Attitude_Reset(Attitude_t *att);

/**
  * @brief  姿态更新，需以固定（或可变）周期调用
  * @param  att   : 句柄
  * @param  gyro  : 三轴角速度，单位 °/s，顺序 {x, y, z}
  * @param  accel : 三轴加速度，单位 g，顺序 {x, y, z}
  * @param  dt    : 距上次调用的时间间隔，单位秒
  * @note   加速度全零或模长接近 0 时自动跳过加速度校正（避免除零与误校正）
  */
void Attitude_Update(Attitude_t *att, const float gyro[3], const float accel[3], float dt);

/**
  * @brief  获取欧拉角
  * @param  att   : 句柄
  * @param  roll  : 输出横滚角（°），可为 NULL
  * @param  pitch : 输出俯仰角（°），可为 NULL
  * @param  yaw   : 输出航向角（°），可为 NULL
  */
void Attitude_GetEuler(const Attitude_t *att, float *roll, float *pitch, float *yaw);

/**
  * @brief  获取四元数
  * @param  att : 句柄
  * @param  q   : 输出缓冲区，长度 4，顺序 (w, x, y, z)
  */
void Attitude_GetQuaternion(const Attitude_t *att, float q[4]);

/**
  * @brief  设置 Mahony 增益
  * @param  att : 句柄
  * @param  kp  : 比例增益，经验值 0.5 ~ 5.0（越大收敛越快、越容易引入噪声）
  * @param  ki  : 积分增益，经验值 0.0 ~ 0.05
  */
void Attitude_SetMahonyGains(Attitude_t *att, float kp, float ki);

/**
  * @brief  设置互补滤波时间常数
  * @param  att : 句柄
  * @param  tau : 时间常数（秒），越大越信任陀螺、越小越信任加速度计
  */
void Attitude_SetComplementaryTau(Attitude_t *att, float tau);

/**
  * @brief  设置卡尔曼滤波噪声参数
  * @param  att : 句柄
  * @param  q_angle   : 角度过程噪声，默认 0.001
  * @param  q_bias    : 零偏过程噪声，默认 0.003
  * @param  r_measure : 测量噪声，默认 0.03
  */
void Attitude_SetKalmanParams(Attitude_t *att, float q_angle, float q_bias, float r_measure);

/**
  * @brief  切换姿态解算算法（会先复位）
  */
void Attitude_SetFilter(Attitude_t *att, AttitudeFilter_t filter);

/**
  * @brief  算法名称字符串，用于串口打印
  */
const char *Attitude_FilterName(AttitudeFilter_t filter);

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_H */
