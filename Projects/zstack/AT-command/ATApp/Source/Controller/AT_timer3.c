/******************************************************************************
  Filename:       AT_timer3.c
  Author:         Jinhui Li
******************************************************************************/
#include "AT_timer3.h"
#include "hal_defs.h"
#include "AT_printf.h"
uint8 Tim3_ovf_count=0;
uint8 Tim3_7000_flag=0;
void AT_Timer3_Set_Clear_Start_US(uint8 us)
{

  // Set timer 3 channel 0 compare mode 
  T3CCTL0 |= BV(2);

  // Set the overflow value
  T3CC0 = us;

  // Set the 8-bit counter to 0x00
  T3CNT = 0x00;

  // Set prescaler divider value, operating mode and start timer 3
  //f1/32,f3/128
  T3CTL |= 0xFE;

  //Timer 3 interrupt flag. 
  //Set to 1 when Timer 1 interrupt occurs and cleared when CPU
  //vectors to the interrupt service routine
    //interrupt enable
  IEN0 |= BV(7);
  IEN1 |= BV(3);
  
  //TIMIF &= ~BV(0);
  //TIMIF &= ~BV(1);
  T3IF = 0;

}
uint8 AT_Timer3_Stop_Get(void)
{
  IEN1 &= ~BV(3);
  T3CTL &= ~BV(4);

  return T3CNT;
}

HAL_ISR_FUNCTION( tim3_ovf_Isr, T3_VECTOR )
{
  //HAL_ENTER_ISR();
  IEN1 &= ~BV(3);
  Tim3_ovf_count++;
    if(Tim3_ovf_count==6)
    {
      printf("down");
      Tim3_7000_flag=1;
    }
  
  //HAL_EXIT_ISR();
}


