/**
  ******************************************************************************
  * @file    dbg_uart.c
  * @brief   USART1 调试串口实现
  ******************************************************************************
  */

#include "dbg_uart.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
/** @brief 当前绑定的串口句柄，NULL 表示未绑定（printf 静默丢弃） */
static UART_HandleTypeDef *s_dbg_uart = NULL;

/* Private define ------------------------------------------------------------*/
#define DBG_TX_TIMEOUT_MS   100U

/* Exported functions --------------------------------------------------------*/

void DBG_Init(UART_HandleTypeDef *huart)
{
  s_dbg_uart = huart;
}

UART_HandleTypeDef *DBG_GetHandle(void)
{
  return s_dbg_uart;
}

/**
  * @brief  newlib 底层批量写接口（覆盖 syscalls.c 中的弱符号实现）
  * @note   printf/snprintf 最终会调用到这里；一次性发送整行，
  *         比逐字节调用 HAL_UART_Transmit 效率高得多。
  */
int _write(int file, char *ptr, int len)
{
  (void)file;

  if ((s_dbg_uart != NULL) && (ptr != NULL) && (len > 0))
  {
    (void)HAL_UART_Transmit(s_dbg_uart, (uint8_t *)ptr, (uint16_t)len, DBG_TX_TIMEOUT_MS);
  }
  return len;
}

/**
  * @brief  newlib 逐字符输出接口，供 putchar() 使用
  */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  if (s_dbg_uart != NULL)
  {
    (void)HAL_UART_Transmit(s_dbg_uart, &c, 1U, DBG_TX_TIMEOUT_MS);
  }
  return ch;
}

/**
  * @brief  newlib 逐字符输入接口（本工程未使用串口输入，保留空实现以便扩展）
  */
int __io_getchar(void)
{
  uint8_t c = 0U;

  if (s_dbg_uart != NULL)
  {
    (void)HAL_UART_Receive(s_dbg_uart, &c, 1U, DBG_TX_TIMEOUT_MS);
  }
  return (int)c;
}

void DBG_FloatToStr(float value, uint8_t decimals, char *buf, uint16_t buf_size)
{
  int32_t scale = 1;
  int32_t fixed;
  int32_t magnitude;
  int32_t int_part;
  int32_t frac_part;
  float   scaled;
  uint8_t i;

  if ((buf == NULL) || (buf_size < 8U))
  {
    return;
  }
  if (decimals > 6U)
  {
    decimals = 6U;
  }

  /* 计算定标因子 10^decimals */
  for (i = 0U; i < decimals; i++)
  {
    scale *= 10;
  }

  scaled = value * (float)scale;

  /* NaN / ±Inf / 超出 int32 可表示范围时降级输出 */
  if (!((scaled > -2000000000.0f) && (scaled < 2000000000.0f)))
  {
    (void)snprintf(buf, buf_size, "nan");
    return;
  }

  /* 四舍五入到定点整数 */
  if (scaled >= 0.0f)
  {
    fixed = (int32_t)(scaled + 0.5f);
  }
  else
  {
    fixed = (int32_t)(scaled - 0.5f);
  }

  if (decimals == 0U)
  {
    (void)snprintf(buf, buf_size, "%ld", (long)fixed);
    return;
  }

  /* 拆分整数位与小数位，避免 -0.5 被打印成 0.5 */
  if (fixed < 0)
  {
    magnitude = -fixed;
    int_part  = magnitude / scale;
    frac_part = magnitude % scale;
    (void)snprintf(buf, buf_size, "-%ld.%0*ld",
                   (long)int_part, (int)decimals, (long)frac_part);
  }
  else
  {
    int_part  = fixed / scale;
    frac_part = fixed % scale;
    (void)snprintf(buf, buf_size, "%ld.%0*ld",
                   (long)int_part, (int)decimals, (long)frac_part);
  }
}
