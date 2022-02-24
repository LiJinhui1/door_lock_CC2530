/******************************************************************************
  Filename:       AT_uart0.h
  Author:         Yang Wang
******************************************************************************/
#ifndef AT_UART0_H
#define AT_UART0_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ZComDef.h"
#include "hal_board.h"

extern void AT_Uart0_Init(void);
extern void AT_Uart0_Send(uint8 *buf, int len);

#ifdef __cplusplus
}
#endif

#endif /* AT_UART0_H */
