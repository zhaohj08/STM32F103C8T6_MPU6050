/**
  ******************************************************************************
  * @file    attitude.c
  * @brief   姿态解算算法实现
  ******************************************************************************
  * 模块内部统一使用右手坐标系，输入为：
  *   gyro  : °/s（陀螺仪，已经过零偏补偿）
  *   accel : g  （加速度计）
  *   dt    : s  （两次调用的时间间隔）
  * 输出为欧拉角（°）：roll / pitch / yaw
  ******************************************************************************
  */

#include "attitude.h"
#include <math.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define RAD_TO_DEG      57.29577951308232f
#define DEG_TO_RAD      0.017453292519943295f

/** @brief 加速度模长平方低于该阈值时认为测量无效（自由落体或数据异常） */
#define ACCEL_SQ_EPS    1.0e-6f

/** @brief dt 上限，防止长时间未调用导致积分发散 */
#define DT_MAX_SEC      0.1f

/* Private function prototypes -----------------------------------------------*/
static void    Mahony_Update(Attitude_t *att, float gx, float gy, float gz,
                            float ax, float ay, float az, float dt);
static void    Complementary_Update(Attitude_t *att, float gx, float gy, float gz,
                                    float ax, float ay, float az, float dt);
static void    Kalman_Update(Attitude_t *att, float gx, float gy, float gz,
                             float ax, float ay, float az, float dt);
static void    Quaternion_ToEuler(const float q[4], float euler[3]);
static void    Kalman1D_Init(Attitude_Kalman1D_t *k, float q_angle, float q_bias, float r_measure);
static float   Kalman1D_Update(Attitude_Kalman1D_t *k, float new_angle, float new_rate, float dt);
static float   Wrap180(float angle);

/* Exported functions --------------------------------------------------------*/

void Attitude_Init(Attitude_t *att, AttitudeFilter_t filter, float sample_freq_hz)
{
  if (att == NULL)
  {
    return;
  }

  (void)memset(att, 0, sizeof(Attitude_t));

  att->two_kp = 1.0f;   /* 2 * Kp，Kp = 0.5 */
  att->two_ki = 0.0f;   /* 2 * Ki，Ki = 0.0，纯比例反馈已足够稳定 */
  att->comp_tau = 0.5f;
  att->sample_freq_hz = (sample_freq_hz > 1.0f) ? sample_freq_hz : 100.0f;

  /* 卡尔曼滤波默认参数（经验值，适用于 100Hz 采样、MPU6050 级别噪声） */
  Kalman1D_Init(&att->k_roll,  0.001f, 0.003f, 0.030f);
  Kalman1D_Init(&att->k_pitch, 0.001f, 0.003f, 0.030f);

  att->filter = filter;
  Attitude_Reset(att);
}

void Attitude_Reset(Attitude_t *att)
{
  if (att == NULL)
  {
    return;
  }

  /* 四元数复位为单位四元数（对应传感器水平放置、朝向正北） */
  att->q[0] = 1.0f;
  att->q[1] = 0.0f;
  att->q[2] = 0.0f;
  att->q[3] = 0.0f;

  att->integral_fb[0] = 0.0f;
  att->integral_fb[1] = 0.0f;
  att->integral_fb[2] = 0.0f;

  att->comp_angle[0] = 0.0f;
  att->comp_angle[1] = 0.0f;
  att->yaw_deg       = 0.0f;

  Kalman1D_Init(&att->k_roll,  att->k_roll.q_angle,  att->k_roll.q_bias,  att->k_roll.r_measure);
  Kalman1D_Init(&att->k_pitch, att->k_pitch.q_angle, att->k_pitch.q_bias, att->k_pitch.r_measure);

  att->euler_deg[0] = 0.0f;
  att->euler_deg[1] = 0.0f;
  att->euler_deg[2] = 0.0f;
  att->update_cnt   = 0U;
}

void Attitude_Update(Attitude_t *att, const float gyro[3], const float accel[3], float dt)
{
  if ((att == NULL) || (gyro == NULL) || (accel == NULL))
  {
    return;
  }
  if (!(dt > 0.0f))
  {
    return;
  }
  if (dt > DT_MAX_SEC)
  {
    dt = DT_MAX_SEC;
  }

  switch (att->filter)
  {
    case ATTITUDE_FILTER_COMPLEMENTARY:
      Complementary_Update(att, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
      break;

    case ATTITUDE_FILTER_KALMAN:
      Kalman_Update(att, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
      break;

    case ATTITUDE_FILTER_MAHONY:
    default:
      /* Mahony 算法内部要求角速度单位为 rad/s */
      Mahony_Update(att,
                    gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD,
                    accel[0], accel[1], accel[2], dt);
      Quaternion_ToEuler(att->q, att->euler_deg);
      break;
  }

  att->update_cnt++;
}

void Attitude_GetEuler(const Attitude_t *att, float *roll, float *pitch, float *yaw)
{
  if (att == NULL)
  {
    return;
  }
  if (roll != NULL)
  {
    *roll = att->euler_deg[0];
  }
  if (pitch != NULL)
  {
    *pitch = att->euler_deg[1];
  }
  if (yaw != NULL)
  {
    *yaw = att->euler_deg[2];
  }
}

void Attitude_GetQuaternion(const Attitude_t *att, float q[4])
{
  if ((att == NULL) || (q == NULL))
  {
    return;
  }
  q[0] = att->q[0];
  q[1] = att->q[1];
  q[2] = att->q[2];
  q[3] = att->q[3];
}

void Attitude_SetMahonyGains(Attitude_t *att, float kp, float ki)
{
  if (att == NULL)
  {
    return;
  }
  att->two_kp = 2.0f * kp;
  att->two_ki = 2.0f * ki;
}

void Attitude_SetComplementaryTau(Attitude_t *att, float tau)
{
  if (att == NULL)
  {
    return;
  }
  if (tau > 0.0f)
  {
    att->comp_tau = tau;
  }
}

void Attitude_SetKalmanParams(Attitude_t *att, float q_angle, float q_bias, float r_measure)
{
  if (att == NULL)
  {
    return;
  }
  Kalman1D_Init(&att->k_roll,  q_angle, q_bias, r_measure);
  Kalman1D_Init(&att->k_pitch, q_angle, q_bias, r_measure);
}

void Attitude_SetFilter(Attitude_t *att, AttitudeFilter_t filter)
{
  if (att == NULL)
  {
    return;
  }
  att->filter = filter;
  Attitude_Reset(att);
}

const char *Attitude_FilterName(AttitudeFilter_t filter)
{
  switch (filter)
  {
    case ATTITUDE_FILTER_MAHONY:        return "Mahony quaternion AHRS";
    case ATTITUDE_FILTER_COMPLEMENTARY: return "Complementary filter";
    case ATTITUDE_FILTER_KALMAN:        return "1D Kalman filter";
    default:                            return "unknown";
  }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Mahony 非线性互补滤波（IMU 六轴版本）
  * @note   参考：R. Mahony, T. Hamel, J.-M. Pflimlin,
  *         "Nonlinear Complementary Filters on the Special Orthogonal Group", 2008.
  */
static void Mahony_Update(Attitude_t *att, float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
  float recip_norm;
  float half_vx, half_vy, half_vz;   /* 由四元数推算出的重力方向 */
  float half_ex, half_ey, half_ez;   /* 测量重力与估计重力的误差 */
  float q0 = att->q[0];
  float q1 = att->q[1];
  float q2 = att->q[2];
  float q3 = att->q[3];
  float qa, qb, qc;
  float norm = (ax * ax) + (ay * ay) + (az * az);

  if (norm > ACCEL_SQ_EPS)
  {
    /* 归一化加速度向量 */
    recip_norm = 1.0f / sqrtf(norm);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    /* 当前四元数对应的理论重力方向（单位向量） */
    half_vx = q1 * q3 - q0 * q2;
    half_vy = q0 * q1 + q2 * q3;
    half_vz = (q0 * q0) - 0.5f + (q3 * q3);

    /* 误差 = 实测重力 × 估计重力（向量叉积） */
    half_ex = (ay * half_vz) - (az * half_vy);
    half_ey = (az * half_vx) - (ax * half_vz);
    half_ez = (ax * half_vy) - (ay * half_vx);

    /* 积分反馈：用于消除陀螺仪常值零偏 */
    if (att->two_ki > 0.0f)
    {
      att->integral_fb[0] += att->two_ki * half_ex * dt;
      att->integral_fb[1] += att->two_ki * half_ey * dt;
      att->integral_fb[2] += att->two_ki * half_ez * dt;

      gx += att->integral_fb[0];
      gy += att->integral_fb[1];
      gz += att->integral_fb[2];
    }

    /* 比例反馈：把估计姿态"拉"向重力实测方向 */
    gx += att->two_kp * half_ex;
    gy += att->two_kp * half_ey;
    gz += att->two_kp * half_ez;
  }

  /* 一阶龙格-库塔法积分四元数微分方程 */
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  qa = q0;
  qb = q1;
  qc = q2;

  q0 += (-qb * gx) - (qc * gy) - (q3 * gz);
  q1 += ( qa * gx) + (qc * gz) - (q3 * gy);
  q2 += ( qa * gy) - (qb * gz) + (q3 * gx);
  q3 += ( qa * gz) + (qb * gy) - (qc * gx);

  /* 归一化，抑制数值积分带来的模长漂移 */
  recip_norm = 1.0f / sqrtf((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));

  att->q[0] = q0 * recip_norm;
  att->q[1] = q1 * recip_norm;
  att->q[2] = q2 * recip_norm;
  att->q[3] = q3 * recip_norm;
}

/**
  * @brief  一阶互补滤波
  *         角度 = α · (角度 + 陀螺积分)  +  (1-α) · 加速度计解算角
  *         其中 α = τ / (τ + dt)，τ 越大越信任陀螺仪。
  */
static void Complementary_Update(Attitude_t *att, float gx, float gy, float gz,
                                 float ax, float ay, float az, float dt)
{
  float alpha = att->comp_tau / (att->comp_tau + dt);
  float roll_gyro  = att->comp_angle[0] + (gx * dt);
  float pitch_gyro = att->comp_angle[1] + (gy * dt);
  float norm = (ax * ax) + (ay * ay) + (az * az);

  if (norm > ACCEL_SQ_EPS)
  {
    /* 由重力分量直接解算出的姿态角（静态准确，动态噪声大） */
    float roll_acc  = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;

    att->comp_angle[0] = (alpha * roll_gyro) + ((1.0f - alpha) * roll_acc);
    att->comp_angle[1] = (alpha * pitch_gyro) + ((1.0f - alpha) * pitch_acc);
  }
  else
  {
    /* 加速度测量不可信时退化为纯陀螺积分 */
    att->comp_angle[0] = roll_gyro;
    att->comp_angle[1] = pitch_gyro;
  }

  /* 航向角没有绝对参考，只能靠陀螺积分（会缓慢漂移） */
  att->yaw_deg = Wrap180(att->yaw_deg + (gz * dt));

  att->euler_deg[0] = Wrap180(att->comp_angle[0]);
  att->euler_deg[1] = att->comp_angle[1];
  att->euler_deg[2] = att->yaw_deg;
}

/**
  * @brief  对 Roll / Pitch 各做一次一维卡尔曼滤波
  *         状态向量 x = [角度, 陀螺零偏]^T
  */
static void Kalman_Update(Attitude_t *att, float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
  float norm = (ax * ax) + (ay * ay) + (az * az);

  if (norm > ACCEL_SQ_EPS)
  {
    float roll_meas  = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_meas = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;

    att->k_roll.angle  = Kalman1D_Update(&att->k_roll,  roll_meas,  gx, dt);
    att->k_pitch.angle = Kalman1D_Update(&att->k_pitch, pitch_meas, gy, dt);
  }
  else
  {
    /* 无有效观测时只做状态预测 */
    att->k_roll.angle  += dt * (gx - att->k_roll.bias);
    att->k_pitch.angle += dt * (gy - att->k_pitch.bias);
  }

  att->yaw_deg = Wrap180(att->yaw_deg + (gz * dt));

  att->euler_deg[0] = Wrap180(att->k_roll.angle);
  att->euler_deg[1] = att->k_pitch.angle;
  att->euler_deg[2] = att->yaw_deg;
}

/**
  * @brief  四元数转欧拉角（ZYX 顺序，单位：度）
  */
static void Quaternion_ToEuler(const float q[4], float euler[3])
{
  float q0 = q[0];
  float q1 = q[1];
  float q2 = q[2];
  float q3 = q[3];
  float sin_pitch;

  euler[0] = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                    1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * RAD_TO_DEG;

  sin_pitch = 2.0f * ((q0 * q2) - (q3 * q1));
  if (sin_pitch >  1.0f) { sin_pitch =  1.0f; }
  if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }
  euler[1] = asinf(sin_pitch) * RAD_TO_DEG;

  euler[2] = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                    1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * RAD_TO_DEG;
}

/**
  * @brief  一维卡尔曼滤波器初始化
  */
static void Kalman1D_Init(Attitude_Kalman1D_t *k, float q_angle, float q_bias, float r_measure)
{
  k->q_angle   = q_angle;
  k->q_bias    = q_bias;
  k->r_measure = r_measure;
  k->angle     = 0.0f;
  k->bias      = 0.0f;
  k->p[0][0]   = 0.0f;
  k->p[0][1]   = 0.0f;
  k->p[1][0]   = 0.0f;
  k->p[1][1]   = 0.0f;
}

/**
  * @brief  一维卡尔曼滤波一次完整迭代（预测 + 更新）
  * @param  new_angle : 加速度计解算出的角度观测值（°）
  * @param  new_rate  : 陀螺仪角速度（°/s）
  * @param  dt        : 采样周期（s）
  * @return 滤波后的角度（°）
  */
static float Kalman1D_Update(Attitude_Kalman1D_t *k, float new_angle, float new_rate, float dt)
{
  float s;
  float y;
  float k0;
  float k1;
  float p00_temp;
  float p01_temp;

  /* ---------- 1. 预测 ---------- */
  k->angle += dt * (new_rate - k->bias);

  k->p[0][0] += dt * ((dt * k->p[1][1]) - k->p[0][1] - k->p[1][0] + k->q_angle);
  k->p[0][1] -= dt * k->p[1][1];
  k->p[1][0] -= dt * k->p[1][1];
  k->p[1][1] += k->q_bias * dt;

  /* ---------- 2. 更新 ---------- */
  s = k->p[0][0] + k->r_measure;
  if (s == 0.0f)
  {
    s = 1.0e-6f;
  }

  k0 = k->p[0][0] / s;
  k1 = k->p[1][0] / s;

  y = new_angle - k->angle;
  k->angle += k0 * y;
  k->bias  += k1 * y;

  p00_temp = k->p[0][0];
  p01_temp = k->p[0][1];

  k->p[0][0] -= k0 * p00_temp;
  k->p[0][1] -= k0 * p01_temp;
  k->p[1][0] -= k1 * p00_temp;
  k->p[1][1] -= k1 * p01_temp;

  return k->angle;
}

/**
  * @brief  把角度归一化到 (-180, 180]
  */
static float Wrap180(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  while (angle <= -180.0f)
  {
    angle += 360.0f;
  }
  return angle;
}
