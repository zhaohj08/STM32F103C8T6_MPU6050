/**
  ******************************************************************************
  * @file    mpu6050_reg.h
  * @brief   MPU6050 寄存器地址与位定义（数据手册 Register Map 摘录）
  ******************************************************************************
  * 说明：
  *   - 仅列出本工程用到的寄存器，完整寄存器表请参考 InvenSense
  *     《MPU-6000 and MPU-6050 Register Map and Descriptions》。
  *   - 地址均为 8 位寄存器地址（不含读写位，HAL 会自行处理）。
  ******************************************************************************
  */

#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 器件地址 ========================= */
/** I2C 7 位从机地址：AD0 接地为 0x68，AD0 接 VCC 为 0x69 */
#define MPU6050_I2C_ADDR_LOW        (0x68U << 1)
#define MPU6050_I2C_ADDR_HIGH       (0x69U << 1)

/* ========================= 寄存器地址 ========================= */
#define MPU6050_REG_SELF_TEST_X     0x0DU
#define MPU6050_REG_SELF_TEST_Y     0x0EU
#define MPU6050_REG_SELF_TEST_Z     0x0FU
#define MPU6050_REG_SELF_TEST_A     0x10U

/* 采样率分频：采样率 = 陀螺仪输出率 / (1 + SMPLRT_DIV) */
#define MPU6050_REG_SMPLRT_DIV      0x19U

/* 数字低通滤波器配置（DLPF_CFG） */
#define MPU6050_REG_CONFIG          0x1AU

/* 陀螺仪量程 FS_SEL[4:3] */
#define MPU6050_REG_GYRO_CONFIG     0x1BU

/* 加速度计量程 AFS_SEL[4:3] */
#define MPU6050_REG_ACCEL_CONFIG    0x1CU

/* 加速度计硬件偏移标定寄存器 */
#define MPU6050_REG_ACCEL_XOFFS_H   0x06U
#define MPU6050_REG_ACCEL_XOFFS_L   0x07U
#define MPU6050_REG_ACCEL_YOFFS_H   0x08U
#define MPU6050_REG_ACCEL_YOFFS_L   0x09U
#define MPU6050_REG_ACCEL_ZOFFS_H   0x0AU
#define MPU6050_REG_ACCEL_ZOFFS_L   0x0BU

#define MPU6050_REG_FIFO_EN         0x23U

/* I2C 旁路、中断引脚电平/开漏配置 */
#define MPU6050_REG_INT_PIN_CFG     0x37U
#define MPU6050_REG_INT_ENABLE      0x38U
#define MPU6050_REG_INT_STATUS      0x3AU

/* 数据输出寄存器（14 字节连续读，一次突发读完 accel + temp + gyro） */
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_ACCEL_XOUT_L    0x3CU
#define MPU6050_REG_ACCEL_YOUT_H    0x3DU
#define MPU6050_REG_ACCEL_YOUT_L    0x3EU
#define MPU6050_REG_ACCEL_ZOUT_H    0x3FU
#define MPU6050_REG_ACCEL_ZOUT_L    0x40U
#define MPU6050_REG_TEMP_OUT_H      0x41U
#define MPU6050_REG_TEMP_OUT_L      0x42U
#define MPU6050_REG_GYRO_XOUT_H     0x43U
#define MPU6050_REG_GYRO_XOUT_L     0x44U
#define MPU6050_REG_GYRO_YOUT_H     0x45U
#define MPU6050_REG_GYRO_YOUT_L     0x46U
#define MPU6050_REG_GYRO_ZOUT_H     0x47U
#define MPU6050_REG_GYRO_ZOUT_L     0x48U

/* 连续读取的字节数：AX AY AZ GX GY GZ + TEMP */
#define MPU6050_BURST_LEN           14U
/** 数据起始寄存器（与 MPU6050_BURST_LEN 搭配使用） */
#define MPU6050_REG_BURST_START     MPU6050_REG_ACCEL_XOUT_H

/* 外部传感器状态（本工程未使用，读取后可用于判断 I2C 从机旁路状态） */
#define MPU6050_REG_EXT_SENS_DATA_00 0x49U
#define MPU6050_REG_EXT_SENS_DATA_01 0x4AU

#define MPU6050_REG_SIGNAL_PATH_RST 0x68U
#define MPU6050_REG_USER_CTRL       0x6AU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU
#define MPU6050_REG_FIFO_COUNTH     0x72U
#define MPU6050_REG_FIFO_COUNTL     0x73U
#define MPU6050_REG_FIFO_R_W        0x74U
#define MPU6050_REG_WHO_AM_I        0x75U

/* ========================= 关键位定义 ========================= */

/** WHO_AM_I 寄存器的固定应答值 */
#define MPU6050_WHO_AM_I_VALUE      0x68U

/* --- PWR_MGMT_1 (0x6B) --- */
#define MPU6050_PWR1_DEVICE_RESET   (1U << 7)   /* 复位所有内部寄存器 */
#define MPU6050_PWR1_SLEEP          (1U << 6)   /* 进入睡眠 */
#define MPU6050_PWR1_CYCLE          (1U << 5)   /* 循环模式 */
#define MPU6050_PWR1_TEMP_DIS       (1U << 3)   /* 关闭温度传感器 */

/** PWR_MGMT_1[2:0] 时钟源选择 */
#define MPU6050_CLKSEL_INTERNAL     0x00U       /* 内部 8MHz RC            */
#define MPU6050_CLKSEL_PLL_XGYRO    0x01U       /* PLL(X 轴陀螺) —— 推荐   */
#define MPU6050_CLKSEL_PLL_YGYRO    0x02U
#define MPU6050_CLKSEL_PLL_ZGYRO    0x03U
#define MPU6050_CLKSEL_EXT_32K      0x04U
#define MPU6050_CLKSEL_EXT_19M      0x05U
/** PLL 时钟参考失效时自动切换为内部 RC，避免芯片“假死” */
#define MPU6050_CLKSEL_STOP_WHEN_UNSTABLE (0x20U)

/* --- SIGNAL_PATH_RESET (0x68) --- */
#define MPU6050_SIGPATH_GYRO_RESET  (1U << 2)
#define MPU6050_SIGPATH_ACCEL_RESET (1U << 1)
#define MPU6050_SIGPATH_TEMP_RESET  (1U << 0)

/* --- USER_CTRL (0x6A) --- */
#define MPU6050_USERCTRL_FIFO_EN    (1U << 6)
#define MPU6050_USERCTRL_I2C_MST_EN (1U << 5)
#define MPU6050_USERCTRL_FIFO_RESET (1U << 2)
#define MPU6050_USERCTRL_I2C_IF_DIS (1U << 4)

/* --- INT_PIN_CFG (0x37) --- */
#define MPU6050_INT_LEVEL           (1U << 7)   /* 1=高有效，0=低有效 */
#define MPU6050_INT_OPEN            (1U << 6)   /* 1=开漏，0=推挽     */
#define MPU6050_LATCH_INT_EN        (1U << 5)   /* 1=锁存，0=50us 脉冲*/
#define MPU6050_INT_RD_CLEAR        (1U << 4)   /* 1=读中断状态即清零 */
#define MPU6050_I2C_BYPASS_EN       (1U << 1)   /* 主模式旁路使能     */

/* --- INT_ENABLE / INT_STATUS (0x38 / 0x3A) --- */
#define MPU6050_INT_DATA_RDY_EN     (1U << 0)
#define MPU6050_INT_DMP_EN          (1U << 1)
#define MPU6050_INT_FIFO_OFLOW_EN   (1U << 4)
#define MPU6050_INT_I2C_MST_EN      (1U << 5)

/* --- GYRO_CONFIG (0x1B) 量程位 --- */
#define MPU6050_GYRO_FS_250         (0x00U << 3)  /* ±250  °/s  */
#define MPU6050_GYRO_FS_500         (0x01U << 3)  /* ±500  °/s  */
#define MPU6050_GYRO_FS_1000        (0x02U << 3)  /* ±1000 °/s  */
#define MPU6050_GYRO_FS_2000        (0x03U << 3)  /* ±2000 °/s  */

/* --- ACCEL_CONFIG (0x1C) 量程位 --- */
#define MPU6050_ACCEL_FS_2          (0x00U << 3)  /* ±2  g */
#define MPU6050_ACCEL_FS_4          (0x01U << 3)  /* ±4  g */
#define MPU6050_ACCEL_FS_8          (0x02U << 3)  /* ±8  g */
#define MPU6050_ACCEL_FS_16         (0x03U << 3)  /* ±16 g */

/* --- CONFIG (0x1A) 数字低通滤波器带宽 --- */
#define MPU6050_DLPF_BW_260HZ       0x00U
#define MPU6050_DLPF_BW_184HZ       0x01U
#define MPU6050_DLPF_BW_94HZ        0x02U
#define MPU6050_DLPF_BW_44HZ        0x03U   /* 100Hz 采样推荐 */
#define MPU6050_DLPF_BW_21HZ        0x04U
#define MPU6050_DLPF_BW_10HZ        0x05U
#define MPU6050_DLPF_BW_5HZ         0x06U

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_REG_H */
