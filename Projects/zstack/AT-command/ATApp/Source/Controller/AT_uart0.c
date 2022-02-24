/******************************************************************************
  Filename:       AT_uart0.c
  Author:         Yang Wang
  baudRate:       115200
******************************************************************************/
#include "AT_uart0.h"

void AT_Uart0_Init(void)
{
  PERCFG &= ~0x01;
  P0SEL   =  0x0C;
  P2DIR  &= ~0xC0;

  U0CSR |= 0x80;
  U0GCR |= 11;
  U0BAUD |= 216;
  UTX0IF = 0;
}

void AT_Uart0_Send(uint8 *buf, int len)
{
  int i;

  for(i=0; i<len; i++)
  {
    U0DBUF = *buf++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}
