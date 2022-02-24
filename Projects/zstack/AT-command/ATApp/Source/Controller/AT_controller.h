/**********************************************************************
  Filename:       AT_controller.h

  Author:         Yasin Zhang
***********************************************************************/
#ifndef AT_CONTROLLER_H
#define AT_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "zcl.h"
#include "zcl_general.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define ZCL_CONTROLLER_ENDPOINT   100
#define ZCL_DEVICE_VERSION        0
#define ZCL_FLAGS                 0

/*********************
 *  command control message
 */
#define ZCL_EPCtrl_Cmd_req    0x01
#define ZCL_EPCtrl_Cmd_rsp    0x02
#define ATF_Cmd_req           0x03
#define ATF_Cmd_rsp           0x04
#define ATCCHANGE_Cmd_req     0x05
#define ATCCHANGE_Cmd_rsp     0x06

/**********************************
 * endPoint control command status
 */
#define EndpointDisabled   0
#define EndpointEnabled    1
#define EndpointUnknown    2

/*********************
 *  error message
 */
#define ZCL_CTRLCmdSend_SUCCESS   0x00
#define ZCL_CTRLCmd_MemError      0x01
#define ZCL_CTRLCmd_ParaError     0x02

/*********************************************************************
 * TYPEDEFS
 */
typedef void (*zclController_EP_CB) (bool isEnable);
typedef struct zclController_EP_Entry_t{
  uint8 ep;
  uint8 *status;
  zclController_EP_CB CB;
  struct zclController_EP_Entry_t *next;
} zclController_EP_List_t;

typedef struct{
  uint8 cmd;
} ZCLCtrlCmd_hdr;

/****************************
 * endPoint control command
 */
typedef struct {
  uint8 ep;
  uint8 isEnable;
} EPCtrlCmd_t;

typedef struct {
  uint8 ep;
  uint8 status;
} EPCtrlCmd_rsp;

/*********************************************************************
 * FUNCTIONS
 */
extern void AT_zclController_Init( uint8* task_id );
extern bool AT_Endpoint_Register( uint8 endPoint, uint8 *epStatus, zclController_EP_CB cbFc );
extern bool AT_Endpoint_Controller( uint8 endPoint, bool isEnable );
extern void ATApp_MessageMSGCB( afIncomingMSGPacket_t *pkt );
extern void AT_zclController_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg);

extern uint8 AT_SendCmd( uint8 srcEP, afAddrType_t *destAddr,
                           uint16 clusterID, uint8 cmd,
                           uint16 cmdFormatLen, uint8 *cmdFormat );
extern uint8 AT_SendEPCtrl(uint8 srcEP, afAddrType_t *dstAddr,
                            uint16 clusterID, EPCtrlCmd_t *epCtrlCmd);
extern uint8 AT_SendEPCtrl_rsp( uint8 srcEP, afAddrType_t *dstAddr,
                            uint16 clusterID, EPCtrlCmd_rsp *epCtrlCmd_rsp );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AT_CONTROLLER_H */
