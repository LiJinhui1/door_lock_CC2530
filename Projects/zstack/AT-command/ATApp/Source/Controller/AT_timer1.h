/******************************************************************************
  Filename:       AT_timer1.h
  Author:         Yang Wang
******************************************************************************/
#ifndef AT_TIMER1_H
#define AT_TIMER1_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ZComDef.h"
#include "hal_board.h"

extern void AT_Timer1_Set_Clear_Start_US(uint16 us);
extern uint16 AT_Timer1_Stop_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* AT_TIMER1_H */
