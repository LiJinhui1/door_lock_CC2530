/******************************************************************************
  Filename:       AT_pwr_single_bus.c
  Author:         Jinhui Li
******************************************************************************/
#include "AT_pwr_single_bus.h"
#include "AT_doorlock.h"
#include "AT_timer1.h"
#include "zcl_doorlock.h"
#include "OnBoard.h"
#include "AT_printf.h"
#include "AF.h"
#include "zcl_closures.h"
#include "hal_defs.h"
#include "bdb.h"

uint8 pwr_single_bus_rcv_buf[PWR_SINGLE_BUS_RCV_MAX] = {0x00};
uint8 pwr_single_bus_rcv_len = 0;
uint8 pwr_single_bus_rcv_bit = 0;

uint16 pwr_single_bus_rcv_total = 0;
uint16 pwr_single_bus_rcv_high = 0;
uint16 pwr_single_bus_rcv_low = 0;


static ZStatus_t AT_DoorLock_SendOperationEventNotification_pwr(uint8 *buf);
void AT_pwr_single_bus_input(void)
{
  PWR_SINGLE_BUS_SEL &= ~PWR_SINGLE_BUS_BV;
  PWR_SINGLE_BUS_DIR &= ~PWR_SINGLE_BUS_BV;
  PWR_SINGLE_BUS_INP &= ~PWR_SINGLE_BUS_BV;

  P2IFG = 0x00;
  P2IF  = 0x00;

  
  IEN2  |= BV(1); // enable port interrupt
  P2IEN |= PWR_SINGLE_BUS_BV; // enable pin interrupt

  P2INP &= ~BV(7); // pull up
  PICTL |=  PWR_SINGLE_BUS_EDGE_BV; // falling edge
}
void AT_pwr_single_bus_output(void)
{
  PWR_SINGLE_BUS_SEL &= ~PWR_SINGLE_BUS_BV;
  PWR_SINGLE_BUS_DIR |=  PWR_SINGLE_BUS_BV;

  IEN2  &= ~BV(1); // disable port interrupt
  P2IEN &= ~PWR_SINGLE_BUS_BV; // disable pin interrupt
}
void AT_pwr_single_bus_init(void)
{
  AT_pwr_single_bus_input();

  // set IPG1(ADC/T1/P2INT) to the highest priority
  IP0 |= BV(1);
  IP1 |= BV(1);
}
void AT_pwr_single_bus_send_byte(uint8 dataByte)
{
  uint8 i;

  for(i=0; i<8; i++)
  {
    if(dataByte & 0x01)
    {
      PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_HIGH;
      MicroWait(191); // more close to 160 us

      PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_LOW;
      MicroWait(92); // more close to 80 us
    }
    else
    {
      PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_HIGH;
      MicroWait(92); // more close to 80 us

      PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_LOW;
      MicroWait(191); // more close to 160 us
    }

    dataByte >>= 1;
  }
}
void AT_pwr_single_bus_send_buf(uint8 *buf, uint8 len)
{
  uint8 i;

  printf("|down|-----|%02d bytes|: ", len);
  for(i=0; i<len; i++)
  {
    printf("%02X ", buf[i]);
  }
  printf("\r\n");

  AT_pwr_single_bus_output();

  PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_LOW;
  MicroWait(4900); // more close to 4 ms

  for(i=0; i<len; i++)
  {
    AT_pwr_single_bus_send_byte(buf[i]);
  }

  PWR_SINGLE_BUS_PIN = PWR_SINGLE_BUS_HIGH;

  AT_pwr_single_bus_input();
}
static ZStatus_t AT_DoorLock_SendOperationEventNotification(uint8 *buf)
{
    ZStatus_t status;

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.addr.shortAddr = NWK_PAN_COORD_ADDR;// default send to Coordinator
  dstAddr.endPoint = 0x0B;//GENERIC_ENDPOINT on Coordinator

  //build DoorLock Operation Event Notification
  zclDoorLockOperationEventNotification_t pPayload;
  pPayload.operationEventSource = 0xFE;
  pPayload.operationEventCode = buf[0];
  pPayload.userID = BUILD_UINT16(buf[1],buf[2]);
  pPayload.pin = buf[3]; // it is battery level actually
  pPayload.zigBeeLocalTime = BUILD_UINT32(buf[4],buf[5],buf[6],buf[7]);

  pPayload.pData = (uint8 *)osal_mem_alloc(1);
  pPayload.pData[0] = 0;

  status = zclClosures_SendDoorLockOperationEventNotification( DOORLOCK_ENDPOINT,//uint8 srcEP,
                                                               &dstAddr,//afAddrType_t *dstAddr,
                                                               &pPayload,//zclDoorLockOperationEventNotification_t *pPayload,
                                                               TRUE,//uint8 disableDefaultRsp,
                                                               bdb_ZclTransactionSequenceNumber++);//uint8 seqNum

  osal_mem_free( pPayload.pData );

  return status;
}
HAL_ISR_FUNCTION( pwr_single_bus_Isr, P2INT_VECTOR )
{
  HAL_ENTER_ISR();

  IEN2  &= ~BV(1); // disable port interrupt
  P2IEN &= ~PWR_SINGLE_BUS_BV; // disable pin interrupt

  if(PWR_SINGLE_BUS_PIN)
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
    if(PWR_SINGLE_BUS_PIN)
      break;
  }
  pwr_single_bus_rcv_low = AT_Timer1_Stop_Get();
  if(pwr_single_bus_rcv_low < 2000)
  {
    goto isr_exit;
  }

  pwr_single_bus_rcv_len = 0;
  pwr_single_bus_rcv_bit = 0;

  // start to capture data
  while(1)
  {
#ifdef WDT_IN_PM1
    WatchDogClear();
#endif

    // get high level time
    AT_Timer1_Set_Clear_Start_US(400);
    while(PWR_SINGLE_BUS_PIN)
    {
      if(T1IF)
      {
        T1IF = 0;

        if((pwr_single_bus_rcv_len > 0) && (pwr_single_bus_rcv_bit == 0))
        {
          if(pwr_single_bus_rcv_len==8)
          AT_DoorLock_SendOperationEventNotification(pwr_single_bus_rcv_buf);
          osal_set_event(zclDoorLock_TaskID, DOORLOCK_HANDLE_RSP_EVT);
          goto isr_exit;
        }

        goto isr_exit;
      }
    }
    pwr_single_bus_rcv_high = AT_Timer1_Stop_Get();

    // get low level time
    AT_Timer1_Set_Clear_Start_US(400);
    while(!PWR_SINGLE_BUS_PIN)
    {
      if(T1IF)
      {
        T1IF = 0;
        goto isr_exit;
      }
    }
    pwr_single_bus_rcv_low = AT_Timer1_Stop_Get();

    // check the total time
    pwr_single_bus_rcv_total = pwr_single_bus_rcv_high + pwr_single_bus_rcv_low;
    if ((pwr_single_bus_rcv_total < 120) || (pwr_single_bus_rcv_total > 350)) //120/4|350/4
      goto isr_exit; // tolerance of 30%

    // save the data bit
    if(pwr_single_bus_rcv_high > pwr_single_bus_rcv_low)
      pwr_single_bus_rcv_buf[pwr_single_bus_rcv_len] |= BV(pwr_single_bus_rcv_bit++);
    else
      pwr_single_bus_rcv_buf[pwr_single_bus_rcv_len] &= ~BV(pwr_single_bus_rcv_bit++);

    // increase the index
    if(pwr_single_bus_rcv_bit == 8) // get a whole byte (8 bits)
    {
      pwr_single_bus_rcv_bit = 0;
      pwr_single_bus_rcv_len++;
    }
  }

isr_exit:
  IEN2  |= BV(1); // enable port interrupt
  P2IEN |= PWR_SINGLE_BUS_BV; // enable pin interrupt
  P2IFG = 0x00;
  P2IF  = 0x00;

  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}
