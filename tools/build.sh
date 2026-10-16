#!/usr/bin/env bash
#
# tools/build.sh - 命令行编译脚本
#
# 用途：
#   1) 不打开 STM32CubeIDE 时快速验证工程能否编译通过（CI 友好）；
#   2) 生成可直接烧录的 .hex / .bin 文件。
#
# 使用前提：arm-none-eabi-gcc 已在 PATH 中。
#   - STM32CubeIDE 自带的工具链路径示例（Windows / Git Bash）：
#       export PATH="/d/Program Files/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/\
#       com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_*/\tools/bin:$PATH"
#     也可以直接通过 ARM_PREFIX 指定工具链前缀，例如：
#       ARM_PREFIX=/c/arm-gnu-toolchain/bin/arm-none-eabi- ./tools/build.sh
#
# 输出：build/ 目录下的 .elf / .hex / .bin / .map
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT}/build"
TARGET="STM32F103C8T6_MPU6050"
LDSCRIPT="${ROOT}/STM32F103C8TX_FLASH.ld"

PREFIX="${ARM_PREFIX:-arm-none-eabi-}"
CC="${PREFIX}gcc"
SIZE="${PREFIX}size"
OBJCOPY="${PREFIX}objcopy"

if ! command -v "${CC}" >/dev/null 2>&1; then
  echo "ERROR: 未找到 ${CC}，请先将其所在目录加入 PATH，或设置 ARM_PREFIX。" >&2
  exit 1
fi

CPU_FLAGS="-mcpu=cortex-m3 -mthumb -mfloat-abi=soft"
DEFS="-DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG"
INCLUDES=(
  "-I${ROOT}/Core/Inc"
  "-I${ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc"
  "-I${ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"
  "-I${ROOT}/Drivers/CMSIS/Device/ST/STM32F1xx/Include"
  "-I${ROOT}/Drivers/CMSIS/Include"
)
CFLAGS=(-std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections -Wall)
ASFLAGS=(-x assembler-with-cpp -g3)
LDFLAGS=(
  "-T${LDSCRIPT}"
  "-Wl,-Map=${BUILD_DIR}/${TARGET}.map"
  -Wl,--gc-sections -static
  --specs=nano.specs --specs=nosys.specs
  -Wl,--start-group -lc -lm -Wl,--end-group
)

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}/obj"

mapfile -t C_SRCS < <(find "${ROOT}/Core/Src" "${ROOT}/Drivers/STM32F1xx_HAL_Driver/Src" \
                        -name '*.c' | sort)
mapfile -t ASM_SRCS < <(find "${ROOT}/Core/Startup" -name '*.s' | sort)

OBJS=()
echo "==> 编译 C 源文件"
for src in "${C_SRCS[@]}"; do
  obj="${BUILD_DIR}/obj/$(basename "${src%.c}").o"
  printf '    CC  %s\n' "$(basename "${src}")"
  "${CC}" ${CPU_FLAGS} ${DEFS} "${INCLUDES[@]}" "${CFLAGS[@]}" -c "${src}" -o "${obj}"
  OBJS+=("${obj}")
done

echo "==> 编译汇编启动文件"
for src in "${ASM_SRCS[@]}"; do
  obj="${BUILD_DIR}/obj/$(basename "${src%.s}").o"
  printf '    AS  %s\n' "$(basename "${src}")"
  "${CC}" ${CPU_FLAGS} ${DEFS} "${INCLUDES[@]}" "${ASFLAGS[@]}" -c "${src}" -o "${obj}"
  OBJS+=("${obj}")
done

echo "==> 链接"
"${CC}" "${OBJS[@]}" ${CPU_FLAGS} "${LDFLAGS[@]}" -o "${BUILD_DIR}/${TARGET}.elf"

echo "==> 生成烧录文件"
"${SIZE}" "${BUILD_DIR}/${TARGET}.elf"
"${OBJCOPY}" -O ihex   "${BUILD_DIR}/${TARGET}.elf" "${BUILD_DIR}/${TARGET}.hex"
"${OBJCOPY}" -O binary "${BUILD_DIR}/${TARGET}.elf" "${BUILD_DIR}/${TARGET}.bin"

echo "==> 完成：${BUILD_DIR}"
