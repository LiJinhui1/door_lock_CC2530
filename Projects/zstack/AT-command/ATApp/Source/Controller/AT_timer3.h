/******************************************************************************
  Filename:       AT_timer3.h
  Author:         Jinhui Li
******************************************************************************/
#ifndef AT_TIMER3_H
#define AT_TIMER3_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ZComDef.h"
#include "hal_board.h"

extern void AT_Timer3_Set_Clear_Start_US(uint8 us);
extern uint8 AT_Timer3_Stop_Get(void);
extern uint8 Tim3_7000_flag;
extern uint8 Tim3_ovf_count;
#ifdef __cplusplus
}
#endif

#endif /* AT_TIMER3_H */
