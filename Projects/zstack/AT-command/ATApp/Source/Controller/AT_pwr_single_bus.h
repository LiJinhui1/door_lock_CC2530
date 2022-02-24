/*********************************************************************
  Filename:       AT_pwr_single_bus.h
  Author:         Jinhui Li
*********************************************************************/
#ifndef AT_PWR_SINGLE_BUS_H
#define AT_PWR_SINGLE_BUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_defs.h"
#include "hal_mcu.h"

#define PWR_SINGLE_BUS_PIN     P2_0
#define PWR_SINGLE_BUS_BV      BV(0)
#define PWR_SINGLE_BUS_SEL     P2SEL
#define PWR_SINGLE_BUS_DIR     P2DIR
#define PWR_SINGLE_BUS_INP     P2INP
#define PWR_SINGLE_BUS_EDGE_BV BV(3)
#define PWR_SINGLE_BUS_UD_BV   BV(6)
#define READ_PWR_SINGLE_BUS    PWR_SINGLE_BUS_PIN

#define PWR_SINGLE_BUS_HIGH    1
#define PWR_SINGLE_BUS_LOW     0

#define PWR_SINGLE_BUS_RCV_MAX 100

#define CMD_START   0xB3
#define CMD_END     0xB6
#define CMD_LOCATE_SHUTDOWN     0xB4
extern uint8 pwr_single_bus_rcv_state;
extern uint8 pwr_single_bus_rcv_buf[PWR_SINGLE_BUS_RCV_MAX];
extern uint8 pwr_single_bus_rcv_len;

extern void AT_pwr_single_bus_init(void);
extern void AT_pwr_single_bus_input(void);
extern void AT_pwr_single_bus_output(void);
extern void AT_pwr_single_bus_send_byte(uint8 dataByte);
extern void AT_pwr_single_bus_send_buf(uint8 *buf, uint8 len);

#ifdef __cplusplus
}
#endif

#endif /* AT_SINGLE_BUS_H */
