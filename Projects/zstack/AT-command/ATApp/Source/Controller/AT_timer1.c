/******************************************************************************
  Filename:       AT_timer1.c
  Author:         Yang Wang
******************************************************************************/
#include "AT_timer1.h"
#include "hal_defs.h"

void AT_Timer1_Set_Clear_Start_US(uint16 us)
{
  // Set timer 1 channel 0 compare mode 
  T1CCTL0 |= BV(2);

  // Set the overflow value
  T1CC0L = LO_UINT16( us );
  T1CC0H = HI_UINT16( us );

  // Set the 16-bit counter to 0x0000
  T1CNTL = 0x00;
  T1CNTH = 0x00; // actually invalid

  // Set prescaler divider value, operating mode and start timer 1
  T1CTL = 0x0A;

  T1IF = 0;
}
uint16 AT_Timer1_Stop_Get(void)
{
  T1CTL = 0x00;

  uint8 low = T1CNTL;
  uint8 high = T1CNTH;

  return BUILD_UINT16(low, high);
}
