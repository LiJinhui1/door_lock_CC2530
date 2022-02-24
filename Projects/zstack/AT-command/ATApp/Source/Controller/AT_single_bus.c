/******************************************************************************
  Filename:       AT_single_bus.c
  Author:         Yang Wang
******************************************************************************/
#include "AT_single_bus.h"
#include "AT_doorlock.h"
#include "AT_timer1.h"
#include "zcl_doorlock.h"
#include "OnBoard.h"
#include "AT_printf.h"

uint8 single_bus_rcv_buf[SINGLE_BUS_RCV_MAX] = {0x00};
uint8 single_bus_rcv_len = 0;
uint8 single_bus_rcv_bit = 0;

uint16 single_bus_rcv_total = 0;
uint16 single_bus_rcv_high = 0;
uint16 single_bus_rcv_low = 0;

static void AT_single_bus_input(void);
static void AT_single_bus_output(void);
static void AT_single_bus_send_byte(uint8 dataByte);

static void AT_single_bus_input(void)
{
  SINGLE_BUS_SEL &= ~SINGLE_BUS_BV;
  SINGLE_BUS_DIR &= ~SINGLE_BUS_BV;
  SINGLE_BUS_INP &= ~SINGLE_BUS_BV;

  P1IFG = 0x00;
  P1IF  = 0x00;

  IEN2  |= BV(4); // enable port interrupt
  P1IEN |= SINGLE_BUS_BV; // enable pin interrupt

  P2INP &= ~BV(6); // pull up
  PICTL |=  SINGLE_BUS_EDGE_BV; // falling edge
}
static void AT_single_bus_output(void)
{
  SINGLE_BUS_SEL &= ~SINGLE_BUS_BV;
  SINGLE_BUS_DIR |=  SINGLE_BUS_BV;

  IEN2  &= ~BV(4); // disable port interrupt
  P1IEN &= ~SINGLE_BUS_BV; // disable pin interrupt
}
void AT_single_bus_init(void)
{
  AT_single_bus_input();

  // set IPG4(ENC/T4/P1INT) to the highest priority
  IP0 |= BV(4);
  IP1 |= BV(4);
}
static void AT_single_bus_send_byte(uint8 dataByte)
{
  uint8 i;

  for(i=0; i<8; i++)
  {
    if(dataByte & 0x01)
    {
      SINGLE_BUS_PIN = SINGLE_BUS_HIGH;
      MicroWait(191); // more close to 160 us

      SINGLE_BUS_PIN = SINGLE_BUS_LOW;
      MicroWait(92); // more close to 80 us
    }
    else
    {
      SINGLE_BUS_PIN = SINGLE_BUS_HIGH;
      MicroWait(92); // more close to 80 us

      SINGLE_BUS_PIN = SINGLE_BUS_LOW;
      MicroWait(191); // more close to 160 us
    }

    dataByte >>= 1;
  }
}
void AT_single_bus_send_buf(uint8 *buf, uint8 len)
{
  uint8 i;

  printf("|down|-----|%02d bytes|: ", len);
  for(i=0; i<len; i++)
  {
    printf("%02X ", buf[i]);
  }
  printf("\r\n");

  AT_single_bus_output();

  SINGLE_BUS_PIN = SINGLE_BUS_LOW;
  MicroWait(4900); // more close to 4 ms

  for(i=0; i<len; i++)
  {
    AT_single_bus_send_byte(buf[i]);
  }

  SINGLE_BUS_PIN = SINGLE_BUS_HIGH;

  AT_single_bus_input();
}

HAL_ISR_FUNCTION( single_bus_Isr, P1INT_VECTOR )
{
  HAL_ENTER_ISR();

  IEN2  &= ~BV(4); // disable port interrupt
  P1IEN &= ~SINGLE_BUS_BV; // disable pin interrupt

  if(SINGLE_BUS_PIN)
  {
    goto isr_exit;
  }

  // start to capture the head (4 ms)
  AT_Timer1_Set_Clear_Start_US(7000);
  while(1)
  {
#ifdef WDT_IN_PM1
    WatchDogClear();
#endif
    if(T1IF)
    {
      T1IF = 0;
      goto isr_exit;
    }
    if(SINGLE_BUS_PIN)
      break;
  }
  single_bus_rcv_low = AT_Timer1_Stop_Get();
  if(single_bus_rcv_low < 2000)
  {
    goto isr_exit;
  }

  single_bus_rcv_len = 0;
  single_bus_rcv_bit = 0;

  // start to capture data
  while(1)
  {
#ifdef WDT_IN_PM1
    WatchDogClear();
#endif

    // get high level time
    AT_Timer1_Set_Clear_Start_US(400);
    while(SINGLE_BUS_PIN)
    {
      if(T1IF)
      {
        T1IF = 0;

        if((single_bus_rcv_len > 0) && (single_bus_rcv_bit == 0))
        {
          osal_set_event(zclDoorLock_TaskID, DOORLOCK_HANDLE_RSP_EVT);
          goto isr_exit;
        }

        goto isr_exit;
      }
    }
    single_bus_rcv_high = AT_Timer1_Stop_Get();

    // get low level time
    AT_Timer1_Set_Clear_Start_US(400);
    while(!SINGLE_BUS_PIN)
    {
      if(T1IF)
      {
        T1IF = 0;
        goto isr_exit;
      }
    }
    single_bus_rcv_low = AT_Timer1_Stop_Get();

    // check the total time
    single_bus_rcv_total = single_bus_rcv_high + single_bus_rcv_low;
    if ((single_bus_rcv_total < 120) || (single_bus_rcv_total > 350))
      goto isr_exit; // tolerance of 30%

    // save the data bit
    if(single_bus_rcv_high > single_bus_rcv_low)
      single_bus_rcv_buf[single_bus_rcv_len] |= BV(single_bus_rcv_bit++);
    else
      single_bus_rcv_buf[single_bus_rcv_len] &= ~BV(single_bus_rcv_bit++);

    // increase the index
    if(single_bus_rcv_bit == 8) // get a whole byte (8 bits)
    {
      single_bus_rcv_bit = 0;
      single_bus_rcv_len++;
    }
  }

isr_exit:

  IEN2  |= BV(4); // enable port interrupt
  P1IEN |= SINGLE_BUS_BV; // enable pin interrupt

  P1IFG = 0x00;
  P1IF = 0x00;

  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}
