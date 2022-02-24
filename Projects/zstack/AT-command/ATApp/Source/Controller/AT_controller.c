/*********************************************************************
  Filename:       AT_controller.c

  Author:         Yasin Zhang
**********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "ZDApp.h"
#include "bdb_interface.h"

#include "zcl.h"
#include "zcl_ha.h"

#include "onboard.h"

#include "AT_controller.h"
#include "ATApp.h"
#include "AT_uart.h"
#include "AT_printf.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
zclController_EP_List_t *zclController_EP_List = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 zclCtrl_TransID = 0;  // This is the unique message ID (counter)

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */

// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclCtrl_InClusterList[] =
{
 ZCL_CLUSTER_ID_GEN_BASIC,
 ZCL_CLUSTER_ID_GEN_IDENTIFY,

 // ZCL_CONTROLLER_TODO: Add application specific Input Clusters Here.
 //       See zcl.h for Cluster ID definitions
};
#define ZCL_CONTROLLER_MAX_INCLUSTERS   0//(sizeof(zclCtrl_InClusterList) / sizeof(zclCtrl_InClusterList[0]))


const cId_t zclCtrl_OutClusterList[] =
{
 ZCL_CLUSTER_ID_GEN_BASIC,

 // ZCL_CONTROLLER_TODO: Add application specific Output Clusters Here.
 //       See zcl.h for Cluster ID definitions
};
#define ZCL_CONTROLLER_MAX_OUTCLUSTERS  0//(sizeof(zclCtrl_OutClusterList) / sizeof(zclCtrl_OutClusterList[0]))

SimpleDescriptionFormat_t zclController_SimpleDesc =
{
 ZCL_CONTROLLER_ENDPOINT,         //  int Endpoint;
 ZCL_HA_PROFILE_ID,               //  uint16 AppProfId;
 // ZCL_CONTROLLER_TODO: Replace ZCL_HA_DEVICEID_REMOTE_CONTROL with application specific device ID
 ZCL_HA_DEVICEID_REMOTE_CONTROL,  //  uint16 AppDeviceId;
 ZCL_DEVICE_VERSION,              //  int   AppDevVer:4;
 ZCL_FLAGS,                       //  int   AppFlags:4;
 ZCL_CONTROLLER_MAX_INCLUSTERS,   //  byte  AppNumInClusters;
 NULL,//(cId_t *)zclCtrl_InClusterList,  //  byte *pAppInClusterList;
 ZCL_CONTROLLER_MAX_OUTCLUSTERS,  //  byte  AppNumInClusters;
 NULL//(cId_t *)zclCtrl_OutClusterList  //  byte *pAppInClusterList;
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AT_zclCtrl_REpCtrl_CB( afIncomingMSGPacket_t *pkt );
static void AT_ATF_CMD_CB( afIncomingMSGPacket_t *pkt );
static void AT_zclCtrl_REpCtrl_req( afIncomingMSGPacket_t *pkt );
static void AT_zclCtrl_REpCtrl_rsp( afIncomingMSGPacket_t *pkt );
static void AT_ATF_Cmd_req( afIncomingMSGPacket_t *pkt );
static uint8 zclCtrlCalcHdrSize( ZCLCtrlCmd_hdr *hdr );
static uint8* zclCtrlBuildHdr( ZCLCtrlCmd_hdr *hdr, uint8 *pData );

// Functions to process ZCL Foundation incoming Command/Response messages
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
static void zclController_PrintfAttrData( uint8 dataType, uint8 *attrData );
#endif
#ifdef ZCL_READ
static uint8 zclController_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclController_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
static uint8 zclController_ProcessInReadReportCfgRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclController_ProcessInConfigReportRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_REPORT_DESTINATION_DEVICE
static void zclController_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclController_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclController_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclController_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclController_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
 * @fn          zclController_Init
 *
 * @brief       Initialize the ZCL controller
 *
 * @param       uint8 task_id - the application that ZCL messages send to
 *
 * @return      none
 */
void AT_zclController_Init( uint8* task_id )
{
  // This app is part of the Home Automation Profile
  bdb_RegisterSimpleDescriptor( &zclController_SimpleDesc );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( *task_id );
}

/*********************************************************************
 * @fn          zclEndpoint_Register
 *
 * @brief       Register the endPoint in the controller
 *
 * @param       uint8 endPoint           - the endPoint need to register
 *              zclController_EP_CB cbFc - the endPoint's callback function
 *
 * @return      if register successfule, return true; otherwise return false
 */
bool AT_Endpoint_Register( uint8 endPoint, uint8 *epStatus, zclController_EP_CB cbFc )
{
  zclController_EP_List_t *pLoop;
  pLoop = zclController_EP_List;
  while (pLoop) {
    if (endPoint == pLoop->ep) {
      break;
    }
    pLoop = pLoop->next;
  }

  // if found, update the endPoint entry
  if (pLoop) {
    pLoop->ep = endPoint;
    pLoop->status = epStatus;
    pLoop->CB = cbFc;
    return true;
  } else {
    //not found, add into the List
    pLoop = osal_mem_alloc(sizeof(zclController_EP_List_t));
    if (pLoop) {
      pLoop->ep   = endPoint;
      pLoop->status = epStatus;
      pLoop->CB   = cbFc;
      pLoop->next = zclController_EP_List;
      zclController_EP_List = pLoop;
      return true;
    } else {
      return false;
    }
  }
}

/*********************************************************************
 * @fn          zclEndpoint_Controller
 *
 * @brief       Control the ZCL endpoint
 *
 * @param       uint8 endPoint - the controlled endPoint
 *              bool isEnable  - the status of the Endpoint
 *
 * @return      none
 */
bool AT_Endpoint_Controller( uint8 endPoint, bool isEnable )
{
  zclController_EP_List_t *pLoop;
  pLoop = zclController_EP_List;
  while (pLoop) {
    if (endPoint == pLoop->ep) {
      break;
    }
    pLoop = pLoop->next;
  }

  // if found, call the endPoint's callback function
  if (pLoop) {
    pLoop->CB(isEnable);
  } else {
    return false;
  }
  return true;
}

/******************************************************************************
 *
 *  Functions for processing application control incoming Command/Response messages
 *
 *****************************************************************************/
/*****************************************************************************
* @fn      ATApp_MessageMSGCB
*
* @brief   processing application control incoming Command/Response messages
*
* @param   pkt - ZCL control incoming message
*
* @return  None
*****************************************************************************/
void ATApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

  switch ( pkt->clusterId )
  {
    case ATApp_GENERIC_CLUSTER:
      AT_zclCtrl_REpCtrl_CB(pkt);
      break;

    case RESET_FACTORY_DEFAULT_CLUSTER:
      AT_ATF_CMD_CB(pkt);
      break;

    default:
      break;
  }
}

/*****************************************************************************
* @fn      AT_zclCtrl_REpCtrl_CB
*
* @brief   processing remote endpoint control incoming messages
*
* @param   pkt - incoming message
*
* @return  None
*****************************************************************************/
void AT_zclCtrl_REpCtrl_CB( afIncomingMSGPacket_t *pkt )
{
  uint8 cmdID = pkt->cmd.Data[0];
  if (cmdID == ZCL_EPCtrl_Cmd_req) {
    AT_zclCtrl_REpCtrl_req(pkt);
  } else {
    AT_zclCtrl_REpCtrl_rsp(pkt);
  }
}

/*****************************************************************************
* @fn      AT_ATF_CMD_CB
*
* @brief   processing remote ATF command messages
*
* @param   pkt - incoming message
*
* @return  None
*****************************************************************************/
void AT_ATF_CMD_CB( afIncomingMSGPacket_t *pkt )
{
  uint8 cmdID = pkt->cmd.Data[0];
  if (cmdID == ATF_Cmd_req) {
    AT_ATF_Cmd_req(pkt);
  }
}

/*****************************************************************************
* @fn      AT_zclCtrl_REpCtrl_req
*
* @brief   processing remote endpoint control incoming request messages
*
* @param   pkt - incoming message
*
* @return  None
*****************************************************************************/
void AT_zclCtrl_REpCtrl_req( afIncomingMSGPacket_t *pkt )
{
  uint8 state;
  EPCtrlCmd_t *epCtrlCmd = (EPCtrlCmd_t*) &(pkt->cmd.Data[1]);
  EPCtrlCmd_rsp *epCtrlCmd_rsp = (EPCtrlCmd_rsp*) osal_mem_alloc(sizeof(EPCtrlCmd_rsp));

  uint8 endPoint = epCtrlCmd->ep;
  uint8 isEnable = epCtrlCmd->isEnable;
  if (isEnable != 0) isEnable = 1;

  epCtrlCmd_rsp->ep = endPoint;
  if (isEnable == 0 || isEnable == 1) {
    if (isEnable) {
      epCtrlCmd_rsp->status = EndpointEnabled;
    } else {
      epCtrlCmd_rsp->status = EndpointDisabled;
    }
  } else {
    epCtrlCmd_rsp->status = EndpointUnknown;
  }

  state = AT_SendEPCtrl_rsp( ATApp_ENDPOINT, &(pkt->srcAddr),
                            ATApp_GENERIC_CLUSTER, epCtrlCmd_rsp );
  osal_mem_free ( epCtrlCmd_rsp );
  if(state != afStatus_SUCCESS) AT_ERROR(state);
}

/*****************************************************************************
* @fn      AT_zclCtrl_REpCtrl_rsp
*
* @brief   processing remote endpoint control incoming response messages
*
* @param   pkt - incoming message
*
* @return  None
*****************************************************************************/
void AT_zclCtrl_REpCtrl_rsp( afIncomingMSGPacket_t *pkt )
{
  EPCtrlCmd_rsp *epCtrlCmd_rsp = (EPCtrlCmd_rsp*) &(pkt->cmd.Data[1]);

  uint8 ep     = epCtrlCmd_rsp->ep;
  uint8 status = epCtrlCmd_rsp->status;

  uint8 str[3];
  AT_NEW_LINE();
  AT_RESP("REMOTE RESPONSE:", 16);
  if (status == EndpointEnabled) {
    AT_NEW_LINE();
    AT_RESP("ENABLED:", sizeof("ENABLED:")-1);
    AT_Int8toChar(ep, str);
    AT_RESP(str, 2);
    AT_NEW_LINE();
  } else if (status == EndpointDisabled) {
    AT_NEW_LINE();
    AT_RESP("DISABLED:", sizeof("DISABLED:")-1);
    AT_Int8toChar(ep, str);
    AT_RESP(str, 2);
    AT_NEW_LINE();
  } else if (status == EndpointUnknown) {
    AT_NEW_LINE();
    AT_RESP("UNKNOWNEP ERROR", sizeof("UNKNOWNEP ERROR")-1);
    AT_NEW_LINE();
  }
  AT_OK();
}

/*****************************************************************************
* @fn      AT_ATF_Cmd_req
*
* @brief   processing remote ATF command incoming request messages
*
* @param   pkt - incoming message
*
* @return  None
*****************************************************************************/
void AT_ATF_Cmd_req( afIncomingMSGPacket_t *pkt )
{
  // AT_clear_AT_SYSTEM_NVs(); if Application use NV, we need to define this function
  bdb_resetLocalAction();
}

/*********************************************************************
 * @fn      zclCtrlCalcHdrSize
 *
 * @brief   Calculate the number of bytes needed for an outgoing
 *          ZCL header.
 *
 * @param   hdr - outgoing header information
 *
 * @return  returns the number of bytes needed
 ***********************************************************************/
static uint8 zclCtrlCalcHdrSize( ZCLCtrlCmd_hdr *hdr )
{
  uint8 needed = sizeof(uint8);

  return needed;
}

/*********************************************************************
 * @fn      zclCtrlBuildHdr
 *
 * @brief   Build header of the ZCL format
 *
 * @param   hdr - outgoing header information
 * @param   pData - outgoing header space
 *
 * @return  pointer past the header
 ***********************************************************************/
static uint8* zclCtrlBuildHdr( ZCLCtrlCmd_hdr *hdr, uint8 *pData )
{
  // Add the command ID
  *pData++ = hdr->cmd;

  // Should point to the frame payload
  return ( pData );
}

/******************************************************************************
 * @fn      AT_SendCmd
 *
 * @brief   Used to send zcl control Command messages.
 *
 * @param   srcEp - source endpoint
 * @param   destAddr - destination address
 * @param   clusterID - cluster ID
 * @param   cmd - command ID
 * @param   cmdFormatLen - length of the command to be sent
 * @param   cmdFormat - command to be sent
 *
 * @return  ZCL_CTRL_SUCCESS
 ******************************************************************************/
uint8 AT_SendCmd( uint8 srcEP, afAddrType_t *destAddr,
                           uint16 clusterID, uint8 cmd,
                           uint16 cmdFormatLen, uint8 *cmdFormat )
{
  endPointDesc_t *epDesc;
  ZCLCtrlCmd_hdr hdr;
  uint8 *msgBuf;
  uint16 msgLen;
  uint8 *pBuf;
  uint8 status;

  epDesc = afFindEndPointDesc( srcEP );
  if ( epDesc == NULL )
  {
   return ( ZCL_CTRLCmd_ParaError ); // EMBEDDED RETURN
  }

  // build hdr
  hdr.cmd = cmd;

  // calculate the buffer size
  msgLen = zclCtrlCalcHdrSize( &hdr );
  msgLen += cmdFormatLen;

  // allocate the buffer needed
  msgBuf = osal_mem_alloc(msgLen);
  if ( msgBuf != NULL ) {
    // fill in the ZCL Header
    pBuf = zclCtrlBuildHdr(&hdr, msgBuf);

    // Fill in the command frame
    osal_memcpy( pBuf, cmdFormat, cmdFormatLen );

    status = AF_DataRequest( destAddr, epDesc, clusterID, msgLen, msgBuf,
                          &zclCtrl_TransID, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
    osal_mem_free ( msgBuf );
  } else {
    status = ZCL_CTRLCmd_MemError;
  }

  return status;
}

/*****************************************************************************
 * @fn      AT_SendEPCtrl
 *
 * @brief   Send remote endPoint control command
 *
 * @param   srcEP - Application's endpoint
 * @param   dstAddr - destination address
 * @param   clusterID - cluster ID
 * @param   epCtrlCmd - endPoint control command to be sent
 *
 * @return  status
 *****************************************************************************/
uint8 AT_SendEPCtrl(uint8 srcEP, afAddrType_t *dstAddr, uint16 clusterID, EPCtrlCmd_t *epCtrlCmd)
{
  uint16 dataLen;
  uint8 *buf;
  uint8 *pBuf;
  uint8 status;

  dataLen = sizeof(EPCtrlCmd_t);

  buf = osal_mem_alloc( dataLen );
  if ( buf != NULL )
  {
    // Load the buffer - serially
    pBuf = buf;
    *pBuf++ = epCtrlCmd->ep;
    *pBuf   = epCtrlCmd->isEnable;

    status = AT_SendCmd( srcEP, dstAddr, clusterID,
                              ZCL_EPCtrl_Cmd_req, dataLen, buf );
    osal_mem_free( buf );
  }
  else
  {
    status = ZCL_CTRLCmd_MemError;
  }

  return ( status );
}

/*****************************************************************************
 * @fn      AT_SendEPCtrl_rsp
 *
 * @brief   Send remote endPoint control command
 *
 * @param   srcEP - Application's endpoint
 * @param   dstAddr - destination address
 * @param   clusterID - cluster ID
 * @param   epCtrlCmd - endPoint control command to be sent
 *
 * @return  status
 *****************************************************************************/
uint8 AT_SendEPCtrl_rsp( uint8 srcEP, afAddrType_t *dstAddr, uint16 clusterID, EPCtrlCmd_rsp *epCtrlCmd_rsp )
{
  uint16 dataLen;
  uint8 *buf;
  uint8 *pBuf;
  uint8 status;

  dataLen = sizeof(EPCtrlCmd_rsp);

  buf = osal_mem_alloc( dataLen );
  if ( buf != NULL )
  {
    // Load the buffer - serially
    pBuf = buf;
    *pBuf++ = epCtrlCmd_rsp->ep;
    *pBuf   = epCtrlCmd_rsp->status;

    status = AT_SendCmd( srcEP, dstAddr, clusterID,
                              ZCL_EPCtrl_Cmd_rsp, dataLen, buf );
    osal_mem_free( buf );
  }
  else
  {
    status = ZCL_CTRLCmd_MemError;
  }

  return ( status );
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      AT_zclController_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
void AT_zclController_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclController_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclController_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORTING_DEVICE
    case ZCL_CMD_CONFIG_REPORT:
      break;
#endif
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
    case ZCL_CMD_CONFIG_REPORT_RSP:
      zclController_ProcessInConfigReportRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORTING_DEVICE
    case ZCL_CMD_READ_REPORT_CFG:
      break;
#endif
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
    case ZCL_CMD_READ_REPORT_CFG_RSP:
      zclController_ProcessInReadReportCfgRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT_DESTINATION_DEVICE
    case ZCL_CMD_REPORT:
      zclController_ProcessInReportCmd( pInMsg );
      break;
#endif

    case ZCL_CMD_DEFAULT_RSP:
      zclController_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclController_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclController_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclController_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclController_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_REPORT_CONFIGURING_DEVICE
/*********************************************************************
 * @fn      zclController_PrintfAttrData
 *
 * @brief   Tool Function. Print different types of Attributes' Data
 *
 * @param   dataType - data types defined in zcl.h
 * @param   attrData - pointer to the attribute data
 *
 * @return  none
 *********************************************************************/
static void zclController_PrintfAttrData( uint8 dataType, uint8 *attrData )
{
  uint8 *pStr;
//  uint16 len;

  if ( attrData == NULL )
  {
    return;
  }

  switch ( dataType )
  {
    case ZCL_DATATYPE_DATA8:
    case ZCL_DATATYPE_BOOLEAN:
    case ZCL_DATATYPE_BITMAP8:
    case ZCL_DATATYPE_INT8:
    case ZCL_DATATYPE_UINT8:
    case ZCL_DATATYPE_ENUM8:
      pStr = zcl_mem_alloc( 2 );
      AT_IntxtoChar(attrData, pStr, 8);
      AT_RESP(pStr, 2);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_DATA16:
    case ZCL_DATATYPE_BITMAP16:
    case ZCL_DATATYPE_UINT16:
    case ZCL_DATATYPE_INT16:
    case ZCL_DATATYPE_ENUM16:
    case ZCL_DATATYPE_SEMI_PREC:
    case ZCL_DATATYPE_CLUSTER_ID:
    case ZCL_DATATYPE_ATTR_ID:
      pStr = zcl_mem_alloc( 4 );
      AT_IntxtoChar(attrData, pStr, 16);
      AT_RESP(pStr, 4);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_DATA24:
    case ZCL_DATATYPE_BITMAP24:
    case ZCL_DATATYPE_UINT24:
    case ZCL_DATATYPE_INT24:
      pStr = zcl_mem_alloc( 6 );
      AT_IntxtoChar(attrData, pStr, 24);
      AT_RESP(pStr, 6);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_DATA32:
    case ZCL_DATATYPE_BITMAP32:
    case ZCL_DATATYPE_UINT32:
    case ZCL_DATATYPE_INT32:
    case ZCL_DATATYPE_SINGLE_PREC:
    case ZCL_DATATYPE_TOD:
    case ZCL_DATATYPE_DATE:
    case ZCL_DATATYPE_UTC:
    case ZCL_DATATYPE_BAC_OID:
      pStr = zcl_mem_alloc( 8 );
      AT_IntxtoChar(attrData, pStr, 32);
      AT_RESP(pStr, 8);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_UINT40:
    case ZCL_DATATYPE_INT40:
      pStr = zcl_mem_alloc( 10 );
      AT_IntxtoChar(attrData, pStr, 40);
      AT_RESP(pStr, 10);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_UINT48:
    case ZCL_DATATYPE_INT48:
      pStr = zcl_mem_alloc( 12 );
      AT_IntxtoChar(attrData, pStr, 48);
      AT_RESP(pStr, 12);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_UINT56:
    case ZCL_DATATYPE_INT56:
      pStr = zcl_mem_alloc( 14 );
      AT_IntxtoChar(attrData, pStr, 56);
      AT_RESP(pStr, 14);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_DOUBLE_PREC:
    case ZCL_DATATYPE_IEEE_ADDR:
    case ZCL_DATATYPE_UINT64:
    case ZCL_DATATYPE_INT64:
      pStr = zcl_mem_alloc( 16 );
      AT_IntxtoChar(attrData, pStr, 64);
      AT_RESP(pStr, 16);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_CHAR_STR:
    case ZCL_DATATYPE_OCTET_STR:
      pStr = (uint8*)attrData;
//      len = *pStr;
//      AT_RESP(pStr+1, len);
      break;

    case ZCL_DATATYPE_LONG_CHAR_STR:
    case ZCL_DATATYPE_LONG_OCTET_STR:
      pStr = (uint8*)attrData;
//      len = BUILD_UINT16( pStr[0], pStr[1] );
//      AT_RESP(pStr+2, len);
      break;

    case ZCL_DATATYPE_128_BIT_SEC_KEY:
      pStr = zcl_mem_alloc( SEC_KEY_LEN*2 );
      AT_IntxtoChar(attrData, pStr, SEC_KEY_LEN);
      AT_RESP(pStr, SEC_KEY_LEN*2);
      zcl_mem_free( pStr );
      break;

    case ZCL_DATATYPE_NO_DATA:
    case ZCL_DATATYPE_UNKNOWN:
      // Fall through

    default:
      break;
  }
}
#endif

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclController_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 *********************************************************************/
static uint8 zclController_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  uint8 str[17];
  AT_NEW_LINE();
  AT_RESP("---------ADDR,EP,C_ID,A_ID,ST,DT,DV", 35);
  AT_NEW_LINE();
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    AT_RESP("ReadAttr:", 9);
    // Node Address
    if (pInMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
      AT_Int16toChar(pInMsg->srcAddr.addr.shortAddr, str);
      AT_RESP(str, 4);
    } else {
      AT_GetIEEEAddrStr(pInMsg->srcAddr.addr.extAddr, str);
      AT_RESP(str, 16);
    }
    AT_RESP(",", 1);

    // Endpoint
    AT_Int8toChar(pInMsg->srcAddr.endPoint, str);
    AT_RESP(str, 2);
    AT_RESP(",", 1);

    // Cluster ID
    AT_Int16toChar(pInMsg->clusterId, str);
    AT_RESP(str, 4);
    AT_RESP(",", 1);

    // Attribute ID
    AT_Int16toChar(readRspCmd->attrList[i].attrID, str);
    AT_RESP(str, 4);
    AT_RESP(",", 1);

    // Status
    AT_Int8toChar(readRspCmd->attrList[i].status, str);
    AT_RESP(str,2);
    AT_RESP(",",1);

    // Data type
    AT_Int8toChar(readRspCmd->attrList[i].dataType, str);
    AT_RESP(str,2);
    AT_RESP(",",1);

    // Attribute value
    if (readRspCmd->attrList[i].dataType == ZCL_DATATYPE_CHAR_STR) {
      AT_RESP(readRspCmd->attrList[i].data+1, readRspCmd->attrList[i].data[0]);
    } else if (readRspCmd->attrList[i].dataType == ZCL_DATATYPE_DATA16||
               readRspCmd->attrList[i].dataType == ZCL_DATATYPE_UINT16||
               readRspCmd->attrList[i].dataType == ZCL_DATATYPE_INT16) {
      AT_Int16toChar(*((uint16*)readRspCmd->attrList[i].data),str);
      AT_RESP(str, 4);
    } else {
      AT_Int8toChar((uint8)readRspCmd->attrList[i].data[0],str);
      AT_RESP(str,2);
    }

    AT_NEW_LINE();
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclController_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  uint8 str[17];
  AT_NEW_LINE();
  AT_RESP("----------ADDR,EP,C_ID,ST", 25);
  AT_NEW_LINE();
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    AT_RESP("WriteAttr:",10);

    //node id
    if(pInMsg->srcAddr.addrMode==(afAddrMode_t)Addr16Bit){
      AT_Int16toChar(pInMsg->srcAddr.addr.shortAddr,str);
      AT_RESP(str,4);
    }else{
      AT_GetIEEEAddrStr(pInMsg->srcAddr.addr.extAddr,str);
      AT_RESP(str,16);
    }
    AT_RESP(",",1);

    //End ponit
    AT_Int8toChar(pInMsg->srcAddr.endPoint,str);
    AT_RESP(str,2);
    AT_RESP(",",1);

    //Cluster ID
    AT_Int16toChar(pInMsg->clusterId,str);
    AT_RESP(str,4);
    AT_RESP(",",1);

    //Attribute ID   Display when status != 0
    if (writeRspCmd->attrList[i].status != 0) {
      AT_Int16toChar(writeRspCmd->attrList[i].attrID,str);
      AT_RESP(str,4);
      AT_RESP(",",1);
    }

    //status
    AT_Int8toChar(writeRspCmd->attrList[i].status,str);
    AT_RESP(str,2);

    if(i < writeRspCmd->numAttr-1) AT_NEXT_LINE();
  }
  AT_NEW_LINE();

  return TRUE;
}
#endif // ZCL_WRITE

#ifdef ZCL_REPORT_CONFIGURING_DEVICE
/*********************************************************************
 * @fn      zclController_ProcessInReadReportCfgRspCmd
 *
 * @brief   Process the "Profile" Read Reporting Configuration Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInReadReportCfgRspCmd( zclIncomingMsg_t *pInMsg )
{
  uint16 nodeId   = pInMsg->srcAddr.addr.shortAddr;
  uint8  endpoint = pInMsg->srcAddr.endPoint;
  uint16 cID      = pInMsg->clusterId;
  zclReadReportCfgRspCmd_t *readReportCfgRspCmd;
  uint8 i;
  // uint8 status;
  // uint8 direction;
  // uint16 attrID;
  // uint8 dataType;
  // uint16 minReportInt;
  // uint16 maxReportInt;
  // uint16 timeoutPeriod;

  readReportCfgRspCmd = (zclReadReportCfgRspCmd_t *)pInMsg->attrCmd;

  AT_NEW_LINE();
  printf("READRCFGRSP:%04X,%02X,%04X,%02X", nodeId, endpoint, cID, readReportCfgRspCmd->numAttr);
  for ( i = 0; i < readReportCfgRspCmd->numAttr; i++ ) {
    zclReportCfgRspRec_t *reportRspRec = &(readReportCfgRspCmd->attrList[i]);
    AT_NEXT_LINE();
    printf("ATTR%02X:%02X,%02X,%04X", i, reportRspRec->status,
            reportRspRec->direction, reportRspRec->attrID);
    if ( reportRspRec->status == ZCL_STATUS_SUCCESS ) {
      if ( reportRspRec->direction == ZCL_SEND_ATTR_REPORTS ) {
        printf(",%02X,%04X,%04X,", reportRspRec->dataType,
                reportRspRec->minReportInt, reportRspRec->maxReportInt);
        if ( zclAnalogDataType( reportRspRec->dataType ) ) {
          zclController_PrintfAttrData(reportRspRec->dataType,
                                       reportRspRec->reportableChange);
        }
      } else {
        printf(",%04X", reportRspRec->timeoutPeriod);
      }
    }
  }
  AT_NEXT_LINE();
  AT_RESP("END\r\n", 5);
  AT_NEW_LINE();

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclController_ProcessInConfigReportRspCmd
 *
 * @brief   Process the "Profile" Configure Reporting Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInConfigReportRspCmd( zclIncomingMsg_t *pInMsg )
{
  uint16 nodeId   = pInMsg->srcAddr.addr.shortAddr;
  uint8  endpoint = pInMsg->srcAddr.endPoint;
  uint16 cID      = pInMsg->clusterId;
  zclCfgReportRspCmd_t *cfgReportRspCmd;
  uint8 i;
  // uint8  status;
  // uint8  direction;
  // uint16 attrID;

  cfgReportRspCmd = (zclCfgReportRspCmd_t *)pInMsg->attrCmd;

  AT_NEW_LINE();
  printf("CFGRPTRSP:%04X,%02X,%04X", nodeId, endpoint, cID);
  if (cfgReportRspCmd->numAttr == 0) {
  	AT_RESP(",00", 3);
  } else {
	for ( i = 0; i < cfgReportRspCmd->numAttr; i++ )
  	{
      printf(",%02X,%02X,%04X", cfgReportRspCmd->attrList[i].status,
	  	cfgReportRspCmd->attrList[i].direction, cfgReportRspCmd->attrList[i].attrID);
    }
  }
  AT_NEW_LINE();

  return ( TRUE );
}

#endif // ZCL_REPORT_CONFIGURING_DEVICE

#ifdef ZCL_REPORT_DESTINATION_DEVICE
/*********************************************************************
 * @fn      zclController_ProcessInReportCmd
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclController_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclReportCmd_t *pInDataReport;
  uint8 i;
  uint8 str[17];

  pInDataReport = (zclReportCmd_t *)pInMsg->attrCmd;

  AT_NEW_LINE();
  AT_RESP("REPORT:", 7);

  // Node Address
  if (pInMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    AT_Int16toChar(pInMsg->srcAddr.addr.shortAddr, str);
    AT_RESP(str, 4);
  } else {
    AT_GetIEEEAddrStr(pInMsg->srcAddr.addr.extAddr, str);
    AT_RESP(str, 16);
  }
  AT_RESP(",", 1);

  // Endpoint
  AT_Int8toChar(pInMsg->srcAddr.endPoint, str);
  AT_RESP(str, 2);
  AT_RESP(",", 1);

  // Cluster ID
  AT_Int16toChar(pInMsg->clusterId, str);
  AT_RESP(str, 4);
  AT_RESP(",", 1);

  // numAttr
  AT_Int8toChar(pInDataReport->numAttr, str);
  AT_RESP(str, 2);
  AT_NEW_LINE();

  for (i = 0; i < pInDataReport->numAttr; i++)
  {
    AT_RESP("RPTATTR:", 8);

    // Attribute ID
    AT_Int16toChar(pInDataReport->attrList[i].attrID, str);
    AT_RESP(str, 4);
    AT_RESP(",", 1);

    // Data type
    AT_Int8toChar(pInDataReport->attrList[i].dataType, str);
    AT_RESP(str,2);
    AT_RESP(",",1);

    // Attribute value
    if (pInDataReport->attrList[i].dataType == ZCL_DATATYPE_CHAR_STR) {
      AT_RESP(pInDataReport->attrList[i].attrData+1, pInDataReport->attrList[i].attrData[0]);
    } else if (pInDataReport->attrList[i].dataType == ZCL_DATATYPE_DATA16||
               pInDataReport->attrList[i].dataType == ZCL_DATATYPE_UINT16||
               pInDataReport->attrList[i].dataType == ZCL_DATATYPE_INT16) {
      AT_Int16toChar(*((uint16*)pInDataReport->attrList[i].attrData),str);
      AT_RESP(str, 4);
    } else {
      AT_Int8toChar((uint8)pInDataReport->attrList[i].attrData[0],str);
      AT_RESP(str,2);
    }

    AT_NEW_LINE();
  }
  AT_RESP("END\r\n", 5);
  AT_NEW_LINE();
}
#endif  // ZCL_REPORT_DESTINATION_DEVICE

/*********************************************************************
 * @fn      zclController_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDefaultRspCmd_t *defaultRspCmd;
  uint16 nodeId   = pInMsg->srcAddr.addr.shortAddr;
  uint8  endpoint = pInMsg->srcAddr.endPoint;
  uint16 cID = pInMsg->clusterId;

  defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  AT_NEW_LINE();
  printf("DFTRSP:%04X,%02X,%04X,%02X,%02X", nodeId, endpoint, cID, defaultRspCmd->commandID,
  									defaultRspCmd->statusCode);
  AT_NEW_LINE();

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclController_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclController_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;
  uint16 nodeId   = pInMsg->srcAddr.addr.shortAddr;
  uint8  endpoint = pInMsg->srcAddr.endPoint;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  AT_NEW_LINE();
  printf("DISCATTR:%04X,%02X,%02X,%02X", nodeId, endpoint, discoverRspCmd->numAttr,
  									discoverRspCmd->discComplete); // 0-Uncompleted 1-completed
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    AT_NEXT_LINE();
    printf("CLUS:%04X,ATTR:%04X,TYPE:%02X",
           pInMsg->clusterId,
           discoverRspCmd->attrList[i].attrID,
           discoverRspCmd->attrList[i].dataType);
  }
  AT_NEXT_LINE();
  AT_RESP("END\r\n", 5);
  AT_NEW_LINE();

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclController_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 ********************************************************************/
static uint8 zclController_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

/****************************************************************************
****************************************************************************/
