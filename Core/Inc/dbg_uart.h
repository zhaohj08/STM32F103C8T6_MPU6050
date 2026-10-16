/**
  ******************************************************************************
  * @file    dbg_uart.h
  * @brief   USART1 调试串口：printf 重定向 + 浮点定点格式化
  ******************************************************************************
  * 为什么不用 %f：
  *   newlib-nano 为了节省 Flash，默认裁剪掉了 printf 的浮点支持，
  *   直接写 printf("%f") 会输出空白或乱码，必须在链接选项里加 -u _printf_float。
  *   本工程改用 DBG_FloatToStr() 把浮点转成字符串后再用 %s 输出，
  *   既不依赖链接选项，也避免了额外的 Flash 开销。
  ******************************************************************************
  */

#ifndef __DBG_UART_H
#define __DBG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
  * @brief  绑定调试串口，绑定后 printf() 才会真正输出
  * @param  huart : 已初始化的串口句柄（本工程为 &huart1）
  */
void DBG_Init(UART_HandleTypeDef *huart);

/**
  * @brief  获取当前绑定的串口句柄
  */
UART_HandleTypeDef *DBG_GetHandle(void);

/**
  * @brief  将浮点数格式化为定长小数的字符串
  * @param  value     : 待格式化数值
  * @param  decimals  : 小数位数（0 ~ 6）
  * @param  buf       : 输出缓冲区，建议不小于 16 字节
  * @param  buf_size  : 缓冲区长度
  * @note   处理了 ±0.5 的进位与负零问题；非有限值（NaN/Inf）输出 "nan"。
  */
void DBG_FloatToStr(float value, uint8_t decimals, char *buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __DBG_UART_H */
