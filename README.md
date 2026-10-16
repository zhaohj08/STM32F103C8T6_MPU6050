# STM32F103C8T6 + MPU6050 姿态解算工程

基于 **STM32F103C8T6**（最小系统板 / Blue Pill）与 **MPU6050** 六轴惯性传感器的姿态解算例程。
工程使用 **STM32CubeMX** 生成配置、**STM32CubeIDE** 作为开发环境，底层基于 **STM32 HAL 库**。

固件以 100 Hz 通过硬件 I2C 读取 MPU6050 原始数据，经陀螺仪零偏标定后交给姿态解算模块，
实时输出 Roll / Pitch / Yaw 欧拉角，并以 10 Hz 通过串口输出，可直接用配套的上位机脚本绘图。

---

## 1. 功能特性

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103C8T6，LQFP48，Flash 64 KB / SRAM 20 KB |
| 系统时钟 | HSE 8 MHz 晶振 → PLL ×9 → **SYSCLK 72 MHz**（APB1 36 MHz / APB2 72 MHz） |
| 传感器接口 | 硬件 **I2C1** @ 400 kHz（PB6 = SCL，PB7 = SDA） |
| 采样率 | **100 Hz** 定步长采样，步长由 TIM2 微秒计时器实测得到 |
| 姿态解算 | **Mahony 四元数 AHRS**（默认）/ 一阶互补滤波 / 一维卡尔曼滤波，可一键切换 |
| 零偏标定 | 上电自动采集 500 点静止数据，计算三轴陀螺零偏并实时扣除 |
| 调试输出 | USART1 @ 115200 8N1，输出人类可读的 CSV 风格数据帧 |
| 状态指示 | PC13 板载 LED：在线 500 ms 慢闪，传感器异常 100 ms 快闪 |
| 容错 | I2C 通信失败自动标记离线并每 2 s 尝试重新初始化；支持 0x68/0x69 地址自动探测 |

实测资源占用（`-O0`，Debug 配置）：

```
   text    data     bss     dec     hex   filename
  29964     104    3360   33428    8294   STM32F103C8T6_MPU6050.elf
FLASH 使用约 30 KB / 64 KB（45.7%），RAM 使用约 3.4 KB / 20 KB（16.9%）
```

---

## 2. 硬件连接

| STM32F103C8T6 | MPU6050 模块 | 说明 |
| --- | --- | --- |
| PB6 | SCL | I2C1 时钟，模块自带 4.7 kΩ 上拉时无需外接 |
| PB7 | SDA | I2C1 数据 |
| 3V3 | VCC | **务必使用 3.3 V**，MPU6050 不耐 5 V |
| GND | GND | 共地 |
| — | AD0 | 接 GND → 地址 0x68（默认）；接 VCC → 0x69，固件会自动探测 |

调试串口（可选，用于观察输出）：

| STM32F103C8T6 | USB-TTL 模块 |
| --- | --- |
| PA9 (USART1_TX) | RX |
| PA10 (USART1_RX) | TX |
| GND | GND |

> 提示：如果使用不带上拉电阻的裸芯片（非模块），需要在 SCL/SDA 上各接一个 4.7 kΩ 上拉到 3V3。

---

## 3. 目录结构

```
STM32F103C8T6_MPU6050/
├── STM32F103C8T6_MPU6050.ioc      # STM32CubeMX 工程配置（时钟树、引脚、外设）
├── .mxproject                     # CubeMX 生成记录，供 IDE 增量更新使用
├── .project / .cproject           # STM32CubeIDE (Eclipse CDT) 工程文件
├── STM32F103C8TX_FLASH.ld         # GCC 链接脚本（64 KB Flash / 20 KB RAM）
├── Core/
│   ├── Inc/
│   │   ├── main.h                 # 引脚宏与全局声明
│   │   ├── stm32f1xx_hal_conf.h   # HAL 模块裁剪配置
│   │   ├── stm32f1xx_it.h
│   │   ├── mpu6050.h              # MPU6050 驱动接口
│   │   ├── mpu6050_reg.h          # MPU6050 寄存器/位定义
│   │   ├── attitude.h             # 姿态解算接口
│   │   └── dbg_uart.h             # 调试串口接口
│   ├── Src/
│   │   ├── main.c                 # 时钟/外设初始化 + 主循环调度
│   │   ├── mpu6050.c              # I2C 读写、量程配置、数据换算、零偏标定
│   │   ├── attitude.c             # Mahony / 互补滤波 / 卡尔曼 三种算法
│   │   ├── dbg_uart.c             # printf 重定向 + 浮点定点格式化
│   │   ├── stm32f1xx_hal_msp.c    # 外设 GPIO/时钟底层初始化
│   │   ├── stm32f1xx_it.c / syscalls.c / sysmem.c / system_stm32f1xx.c
│   ├── Startup/startup_stm32f103c8tx.s
├── Drivers/                       # ST 官方 HAL 驱动 + CMSIS（STM32Cube FW_F1 V1.8.7）
├── Docs/
│   └── attitude_algorithms.md     # 三种姿态解算算法原理与参数整定说明
├── tools/
│   ├── build.sh                   # 命令行编译脚本（CI / 无 IDE 环境）
│   └── plot_attitude.py           # 上位机：串口实时绘图 + CSV 记录
└── README.md
```

---

## 4. 编译与烧录

### 4.1 方式一：STM32CubeIDE（推荐）

1. 打开 STM32CubeIDE → `File` → `Import...` → `General` → `Existing Projects into Workspace`；
2. `Select root directory` 选择本工程根目录（`STM32F103C8T6_MPU6050`），Finish；
3. 工具栏选择配置 `Debug`，点击 🔨 编译；
4. 连接 ST-Link，点击 🐞 `Debug` 或 ▶ `Run` 完成下载。

> 双 击根目录下的 `STM32F103C8T6_MPU6050.ioc` 可以直接在 CubeMX 图形界面里查看/修改
> 时钟树与引脚配置。修改后请按 `GENERATE CODE` 重新生成，
> **用户代码请写在 `/* USER CODE BEGIN */ ... /* USER CODE END */` 之间**，
> 否则重新生成时会被覆盖。

### 4.2 方式二：命令行编译

需要 `arm-none-eabi-gcc`（STM32CubeIDE 自带）与 `bash` 环境（Windows 下可用 Git Bash）：

```bash
# 把 CubeIDE 自带工具链加入 PATH（路径按本机安装版本调整）
export PATH="/d/Program Files/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/\
com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin:$PATH"

bash tools/build.sh
```

编译产物位于 `build/` 目录：`.elf` / `.hex` / `.bin` / `.map`。
可用 STM32CubeProgrammer 直接烧录 `.hex` 或 `.bin`。

---

## 5. 串口输出协议

上电后串口会先打印自检信息，随后按 10 Hz 输出数据帧：

```
=====================================================
 STM32F103C8T6 + MPU6050 Attitude Estimation (HAL)
=====================================================
SYSCLK   : 72000000 Hz
I2C1     : PB6(SCL) / PB7(SDA) @ 400kHz
USART1   : PA9(TX)  / PA10(RX) @ 115200 8N1
Sampling : 100 Hz, output 10 Hz
Filter   : Mahony quaternion AHRS
-----------------------------------------------------
[ OK ] MPU6050 found at address 0x68, WHO_AM_I = 0x68
[ OK ] gyro range +/-2000dps, accel range +/-2g, DLPF 44Hz
[ .. ] calibrating gyroscope, keep the sensor still ...
[ OK ] gyro bias = (0.412, -1.035, 0.688) dps
# $ATT,t_ms,roll_deg,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,temp_c
$ATT,1234,1.27,-3.41,88.52,0.31,-0.08,0.15,0.011,0.006,0.998,28.35
$ATT,1335,1.29,-3.40,88.61,0.28,-0.06,0.14,0.010,0.007,0.999,28.36
```

数据帧字段含义：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `t_ms` | 自开机起的毫秒计数 | ms |
| `roll` / `pitch` / `yaw` | 姿态角 | ° |
| `gx` / `gy` / `gz` | 角速度（已扣除零偏） | °/s |
| `ax` / `ay` / `az` | 加速度 | g |
| `temp` | 芯片内部温度 | ℃ |

### 上位机实时绘图

```bash
pip install pyserial matplotlib
python tools/plot_attitude.py --list                 # 查看串口号
python tools/plot_attitude.py --port COM3            # 实时绘制三轴姿态角
python tools/plot_attitude.py --port COM3 --save data.csv   # 同时存 CSV
python tools/plot_attitude.py --port COM3 --no-plot --save data.csv --duration 60
```

---

## 6. 坐标系与角度定义

以芯片丝印方向为准（右手坐标系），传感器水平放置时：

- **Roll（横滚角 φ）**：绕 X 轴旋转，水平时约 0°，范围 (-180°, +180°]
- **Pitch（俯仰角 θ）**：绕 Y 轴旋转，水平时约 0°，范围 [-90°, +90°]
- **Yaw（航向角 ψ）**：绕 Z 轴旋转，**仅靠陀螺积分，长时间会漂移**

> 说明：只有加速度计（6 轴）时，重力矢量只能观测 Roll 与 Pitch 两个自由度；
> Yaw 缺少绝对参考（需要磁力计）与地球自转参考（需要高精度陀螺），
> 因此 Yaw 会随时间线性漂移，这是算法层面的物理限制，不是固件缺陷。
> 若需要无漂移的 Yaw，可外接 HMC5883L / QMC5883L 磁力计并改用九轴 Mahony 算法。

---

## 7. 姿态解算算法

工程内置三种算法，通过 `Core/Src/main.c` 中的宏一键切换：

```c
#define APP_FILTER_DEFAULT   ATTITUDE_FILTER_MAHONY
/* 可选：ATTITUDE_FILTER_COMPLEMENTARY / ATTITUDE_FILTER_KALMAN */
```

| 算法 | 原理 | 优点 | 缺点 |
| --- | --- | --- | --- |
| `MAHONY`（默认） | 用加速度计观测的重力方向与四元数推算的估计方向做叉积，经 PI 控制器反馈修正陀螺仪积分，全程在四元数域运算 | 无万向锁、动态响应好、可直接得到 Yaw | 参数需要整定 |
| `COMPLEMENTARY` | 高通（陀螺积分）+ 低通（加速度计解算角）加权融合 | 计算量最小、参数只有一个时间常数 | 大机动时误差较大 |
| `KALMAN` | 对 Roll / Pitch 分别建立 `[角度, 陀螺零偏]` 二维状态方程做最优估计 | 可在线估计陀螺零偏、抗噪能力强 | 只适用于 Roll/Pitch，计算量较大 |

详细的公式推导、参数含义与整定方法见 [`Docs/attitude_algorithms.md`](Docs/attitude_algorithms.md)。

---

## 8. 关键实现说明

### 8.1 微秒级定步长采样

TIM2 预分频 71 后计数频率为 **1 MHz**（1 计数 = 1 µs），周期 65535，约 65.5 ms 回绕一次。
主循环利用 **16 位无符号减法自动处理回绕**的特性计算相邻两次采样的间隔：

```c
now_us   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
delta_us = (uint16_t)(now_us - s_last_tick_us);   /* 回绕也能得到正确差值 */
```

积分步长直接用**实测间隔**而不是标称的 10 ms，因此主循环偶发的抖动不会引入解算误差。

### 8.2 为什么不用 `printf("%f")`

`newlib-nano` 为节省 Flash 默认裁剪了浮点格式化，直接输出 `%f` 会得到空白。
常规做法是在链接选项里加 `-u _printf_float`（会增加约 6 KB Flash）。
本工程改用 `DBG_FloatToStr()` 先把浮点四舍五入成定点数再格式化，
**既不依赖链接选项，也不需要额外 Flash**，并且正确处理了 `-0.5` 被打印成 `0.5` 的符号问题。

### 8.3 一次突发读取

MPU6050 的加速度、温度、角速度寄存器是地址连续的（`0x3B` 起 14 字节），
固件用一次 `HAL_I2C_Mem_Read()` 读完，保证同一帧内各轴数据来自同一采样时刻，
比分开读取更高效也更准确。

### 8.4 上电自检与容错

- 自动探测从机地址（0x68 / 0x69）；
- 回读 `PWR_MGMT_1` 确认 SLEEP 位已清除，避免"假在线"；
- 通信异常时置离线标志，LED 转为 100 ms 快闪，并每 2 s 自动重试初始化，
  方便现场排查接线而不必反复复位。

---

## 9. 常见问题排查

| 现象 | 可能原因与处理 |
| --- | --- |
| 串口打印 `MPU6050 init failed: device not found` | 接线错误或未供电：确认 SCL→PB6、SDA→PB7、VCC→3V3 且共地；万用表测模块 VCC 是否有 3.3 V |
| 打印 `I2C communication error` 但能识别到器件 | 总线被拉死或缺少上拉电阻：SCL/SDA 各加 4.7 kΩ 上拉到 3V3；降低 I2C 速率到 100 kHz 试试 |
| 静止时欧拉角持续缓慢变化 | 零偏标定期间传感器被触碰：上电后 1 秒内保持完全静止；也可增大 `APP_GYRO_CALIB_SAMPLES` |
| 串口无任何输出 | PA9/PA10 接反（需交叉 TX↔RX）；波特率不是 115200；USB-TTL 电平为 5 V 时建议加电平转换 |
| 电脑串口助手乱码 | 编码/波特率不匹配；确认 8 位数据、1 位停止、无校验 |
| `region FLASH overflowed` | 若自行改回未裁剪的 HAL 编译选项，请确认已启用 `-ffunction-sections -fdata-sections` 与 `--gc-sections` |
| Yaw 一直缓慢漂移 | 六轴系统的固有限制，见第 6 节说明 |

---

## 10. 二次开发建议

- **换用 DMP**：MPU6050 内置 DMP 可直接输出四元数，但需要移植 InvenSense 的闭源驱动库，
  且对国产兼容芯片支持不佳；本工程选择软件解算，代码完全可控、便于教学与移植。
- **改用中断采集**：MPU6050 的 INT 引脚（数据就绪中断）已配置为推挽高有效，
  接任意 EXTI 引脚即可把轮询改为中断触发采样，进一步降低抖动。
- **外接磁力计**：在 `attitude.c` 中扩展为九轴 Mahony，即可获得无漂移的 Yaw。
- **提高采样率**：修改 `APP_SAMPLE_FREQ_HZ` 与 `MPU6050_SetDLPF()` 的采样率参数即可，
  但需同步确认 I2C 带宽（400 kHz 下单帧 14 字节约 0.4 ms，理论上限约 2 kHz）。

---

## 11. 许可说明

- `Drivers/` 目录为 STMicroelectronics 官方 HAL 驱动与 CMSIS，遵循其自带许可（BSD-3-Clause），
  版权归 STMicroelectronics 所有，详见 `Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt`；
- `Core/`、`tools/`、`Docs/` 下的应用层代码可自由用于学习、教学与二次开发。
