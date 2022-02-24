/*********************************************************************
  Filename:       AT_single_bus.h
  Author:         Yang Wang
*********************************************************************/
#ifndef AT_SINGLE_BUS_H
#define AT_SINGLE_BUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_defs.h"
#include "hal_mcu.h"

#define SINGLE_BUS_PIN     P1_6
#define SINGLE_BUS_BV      BV(6)
#define SINGLE_BUS_SEL     P1SEL
#define SINGLE_BUS_DIR     P1DIR
#define SINGLE_BUS_INP     P1INP
#define SINGLE_BUS_EDGE_BV BV(2)
#define SINGLE_BUS_UD_BV   BV(6)
#define READ_SINGLE_BUS    SINGLE_BUS_PIN

#define SINGLE_BUS_HIGH    1
#define SINGLE_BUS_LOW     0

#define SINGLE_BUS_RCV_MAX 100

extern uint8 single_bus_rcv_state;
extern uint8 single_bus_rcv_buf[SINGLE_BUS_RCV_MAX];
extern uint8 single_bus_rcv_len;

extern void AT_single_bus_init(void);
extern void AT_single_bus_send_buf(uint8 *buf, uint8 len);

#ifdef __cplusplus
}
#endif

#endif /* AT_SINGLE_BUS_H */
