/**********************************************************************
  Filename:       AT_cmd.h

  Author:         Yasin Zhang
***********************************************************************/
#ifndef AT_CMD_H
#define AT_CMD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "AT_uart.h"
#include "AT_printf.h"

/*********************************************************************
 * MACROS
 */
/*********************
*  compile options
*/
#define AT_DEBUG_FUNC    FALSE
#define AT_ZDO_ACK       TRUE

#define AT_SEQ_STROE(x)       StoreSeqNumInBuffer( x-1 );

/*********************************************************************
 * CONSTANTS
 */
#define SEQ_BUFF_SIZE   5

/*********************************************************************
* TYPEDEFS
*/
typedef uint8 ATSeqBuffer[SEQ_BUFF_SIZE];

/*********************************************************************
 * FUNCTIONS
 */
/**
 * ZDO Command Confirm processer
 */
extern void StoreSeqNumInBuffer( uint8 seqNum );
extern void AT_Cmd_ProcessDataConfirm( afDataConfirm_t *afDataConfirm );

/*********************************
*  AT Command callback funcitons
*/
extern void AT_ZDO_Register( uint8 *pTask_id );
extern void AT_ZDO_ProcessMsgCBs( zdoIncomingMsg_t *inMsg );
extern void AT_Cmd_ESCAN_CB( NLME_EDScanConfirm_t *EDScanConfirm );
extern void AT_ZDO_ProcessMgmtNwkDiscRsp( zdoIncomingMsg_t *inMsg );
extern void AT_Cmd_FN_CB( ZStatus_t Status );
extern void* AT_ZDO_ProcessJOIN_CNF_CB(void *param);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_H */
