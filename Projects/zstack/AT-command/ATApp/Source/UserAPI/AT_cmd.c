/**************************************************************************************************
  Filename:       AT_uart.c

  Description:    AT command functions
  Author:         Yasin Zhang
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "ZDNwkMgr.h"
#include "ZDObject.h"
#include "zcl.h"

#include "bdb.h"
#include "bdb_interface.h"

#include "AT_cmd.h"
#include "ATApp.h"
#include "AT_controller.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 EBindSeq = 0xFF;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
// the Energy scan call back funtion
extern void ZDNwkMgr_EDScanConfirmCB( NLME_EDScanConfirm_t *EDScanConfirm );

/*********************************************************************
 * LOCAL VARIABLES
 */
static ATSeqBuffer seqBuff = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 GetSeqNumIndexInBuffer( uint8 seqNum );
static void ResetSeqNumInBuffer ( uint8 index );

static void AT_ZDO_ProcessPowerDescRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessActEpRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessSimpleDescRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessMatchDescRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessAnnceRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessMgmtLqiRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessMgmtRtgRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessMgmtBindRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessBindRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessUnbindRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessEDbindRsp( zdoIncomingMsg_t *inMsg );
static void AT_ZDO_ProcessMgmtLeaveRsp( zdoIncomingMsg_t *inMsg );

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*******************************************************************************
* @fn      AT_ZDO_Register
*
* @brief   Register ZDO messages
*
* @param   pTask_id - tack_id pointer
*
* @return  None
******************************************************************************/
void AT_ZDO_Register( uint8 *pTask_id )
{
  uint8 task_id = *pTask_id;
  ZDO_RegisterForZDOMsg( task_id, NWK_addr_rsp );
  ZDO_RegisterForZDOMsg( task_id, IEEE_addr_rsp );
  ZDO_RegisterForZDOMsg( task_id, Power_Desc_rsp );
  ZDO_RegisterForZDOMsg( task_id, Active_EP_rsp );
  ZDO_RegisterForZDOMsg( task_id, Simple_Desc_rsp );
  ZDO_RegisterForZDOMsg( task_id, Match_Desc_rsp );
  ZDO_RegisterForZDOMsg( task_id, Device_annce );
  ZDO_RegisterForZDOMsg( task_id, Mgmt_Lqi_rsp );
  ZDO_RegisterForZDOMsg( task_id, Mgmt_Rtg_rsp );
  ZDO_RegisterForZDOMsg( task_id, Mgmt_Bind_rsp );
  ZDO_RegisterForZDOMsg( task_id, Bind_rsp );
  ZDO_RegisterForZDOMsg( task_id, Unbind_rsp );
  ZDO_RegisterForZDOMsg( task_id, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( task_id, Mgmt_Leave_rsp );
}

void AT_ZDO_ProcessMsgCBs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case NWK_addr_rsp:
    case IEEE_addr_rsp:
    {
      notdoEUIREQcmd = TRUE;
      ZDO_NwkIEEEAddrResp_t *pAddrRsp = (ZDO_NwkIEEEAddrResp_t *)osal_mem_alloc(sizeof(ZDO_NwkIEEEAddrResp_t));
      pAddrRsp = ZDO_ParseAddrRsp( inMsg );
      if ( pAddrRsp )
      {
        AT_NEW_LINE();
        printf("AddrResp:%02X", pAddrRsp->status);
        if ( pAddrRsp->status == ZSuccess )
        {
          uint8 i;
          char str[16];
          AT_RESP(",", 1);
          AT_Int16toChar(pAddrRsp->nwkAddr, (uint8 *)str);
          AT_RESP(str, 4);
          AT_RESP(",", 1);
          AT_GetIEEEAddrStr(pAddrRsp->extAddr, (uint8 *)str);
          AT_RESP(str, 16);
          for (i = 0; i < pAddrRsp->numAssocDevs; i++) {
            AT_NEXT_LINE();
            AT_Int8toChar(i, (uint8 *)str);
            AT_RESP(str, 2); AT_RESP(". ", 2);
            AT_Int16toChar(pAddrRsp->devList[i], (uint8 *)str);
            AT_RESP(str, 4);
          }
        }
		AT_NEW_LINE();
        osal_mem_free( pAddrRsp );
      }
    }
      break;
    case Power_Desc_rsp:
      AT_ZDO_ProcessPowerDescRsp( inMsg );
      break;
    case Active_EP_rsp:
      AT_ZDO_ProcessActEpRsp( inMsg );
      break;
    case Simple_Desc_rsp:
      AT_ZDO_ProcessSimpleDescRsp( inMsg );
      break;
    case Match_Desc_rsp:
      AT_ZDO_ProcessMatchDescRsp( inMsg );
      break;
    case Device_annce:
      AT_ZDO_ProcessAnnceRsp( inMsg );
      break;
    case Mgmt_Lqi_rsp:
      AT_ZDO_ProcessMgmtLqiRsp( inMsg );
      break;
    case Mgmt_Rtg_rsp:
      AT_ZDO_ProcessMgmtRtgRsp( inMsg );
      break;
    case Mgmt_Bind_rsp:
      AT_ZDO_ProcessMgmtBindRsp( inMsg );
      break;
    case Bind_rsp:
      AT_ZDO_ProcessBindRsp( inMsg );
      break;
    case Unbind_rsp:
      AT_ZDO_ProcessUnbindRsp( inMsg );
      break;
    case End_Device_Bind_rsp:
      AT_ZDO_ProcessEDbindRsp( inMsg );
      break;
    case Mgmt_Leave_rsp:
      AT_ZDO_ProcessMgmtLeaveRsp( inMsg );
      break;
    default:
      break;
  }
}

/**
 * @brief display ACK or NACK prompt
 */
void AT_Cmd_ProcessDataConfirm( afDataConfirm_t *afDataConfirm )
{
  uint8 indexInSeqBuff;
  uint8 sentEP = afDataConfirm->endpoint;
  uint8 sentStatus = afDataConfirm->hdr.status;
  uint8 ZDOTransID = afDataConfirm->transID;

  if (sentEP == 0) {
    indexInSeqBuff = GetSeqNumIndexInBuffer(ZDOTransID);
    if (indexInSeqBuff != 0xFF) {
      if (sentStatus == 0) {
        printf("\r\nACK:%02X\r\n", ZDOTransID);
      } else {
        printf("\r\nNACK:%02X\r\n", ZDOTransID);
      }
      ResetSeqNumInBuffer(indexInSeqBuff);
    } else if (EBindSeq == ZDOTransID) {
      printf("\r\nEBINDACK\r\n");
    }
  }
}

/**
 * @brief  Store seqNum in the buffer
 */
void StoreSeqNumInBuffer( uint8 seqNum )
{
  uint8 indexInSeqBuff = GetSeqNumIndexInBuffer( 0xFF );
  if (indexInSeqBuff != 0xFF) {
    seqBuff[indexInSeqBuff] = seqNum;
  }
}

/**
 * @brief  Check if the seqNum is in the buffer
 * @return index - seqNum index in the buffer    0xFF - Not found
 */
static uint8 GetSeqNumIndexInBuffer( uint8 seqNum )
{
  uint8 rtnIndex = 0xFF;
  uint8 index;
  for ( index = 0; index < SEQ_BUFF_SIZE; index++) {
    if (seqBuff[index] == seqNum) {
      rtnIndex = index;
      break;
    }
  }

  return rtnIndex;
}

/**
 * @brief  Reset specific seqNum in the buffer
 */
static void ResetSeqNumInBuffer ( uint8 index )
{
  seqBuff[index] = 0xFF;
}

/*******************************************************************************
 * @fn      AT_Cmd_ESCAN_CB
 *
 * @brief   AT+SCAN callback function
 *          Response: +ESCAN:  11:XX
 *                              ...
 *                             26:XX
 *                             OK
 *                             or ERROR:<errorcode> .
 *                   XX represents the average energy on the respective channel
 *
 * @param       scannedChannels  - scanned channels
 * @param       energyDetectList - measured energy for channels
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ESCAN_CB( NLME_EDScanConfirm_t *EDScanConfirm)
{
  AT_NEW_LINE();
  AT_RESP("+ESCAN:", 7);
  AT_NEXT_LINE();
  uint8 i;
  for ( i = 0; i < ED_SCAN_MAXCHANNELS; i++ )
  {
    if ( ( (uint32)1 << i ) & EDScanConfirm->scannedChannels )
    {
      printf("%d:%02X", i, EDScanConfirm->energyDetectList[i]);
      if(i < ED_SCAN_MAXCHANNELS-1) AT_NEXT_LINE();
    }
  }
  AT_NEW_LINE();
  AT_OK();
  NLME_NwkDiscTerm();

  // recover the Energy scan call back funtion to ZDO layer so that ZDO can
  // get the energy scan result
  pZDNwkMgr_EDScanConfirmCB = ZDNwkMgr_EDScanConfirmCB;
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMgmtNwkDiscRsp
 *
 * @brief   Response: +PANSCAN:<channel>,<PANID>,<EPANID>,XX,b
 *                    OK or ERROR:<errorcode> .
 *
 *                    <channel> - represents the channel,
 *                    <PANID>   - the PAN ID,
 *                    <EPANID>  - the extended PAN ID,
 *                                The node gives a list of all PANs found.
 *                    XX - the ZigBee stack profile
 *                         (00 = Custom, 01 = ZigBee, 02 = ZigBee PRO)
 *                    b - indicates whether the network is allowing additional
 *                        nodes to join (1 = joining permitted).
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMgmtNwkDiscRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmNwkDiscRsp_t *MgmNwkDiscRsp;
  uint8 i;
  mgmtNwkDiscItem_t *NetworkList, *pNwkDesc;
  // uint8 ResultCount = 0;
  uint8 count = 0; //record the lik quality >0 networks

  MgmNwkDiscRsp = ZDO_ParseMgmNwkDiscRsp(inMsg);

  // ResultCount = MgmNwkDiscRsp->networkCount;
  count = MgmNwkDiscRsp->networkListCount;
  NetworkList = MgmNwkDiscRsp->list;
  pNwkDesc = NetworkList;

  AT_NEW_LINE();
  // printf("%d result(s)\n\r",count);
  // if (count > 0) {
  //   //print:Channel | PANID | EPANID | StackProfile | perimit
  //   printf("Channel | PANID | EPANID | StackProfile | perimit");
  // }
  for ( i = 0; i < count; i++ ) {
    uint16 *ext = (uint16*)pNwkDesc->extendedPANID;
    AT_NEXT_LINE();
    AT_RESP("+PANSCAN:", 9);
    printf("%02d,%04X,%04X%04X%04X%04X,%02X,%X",
    pNwkDesc->logicalChannel,
    pNwkDesc->PANId,
    ext[3],ext[2],ext[1],ext[0],
    pNwkDesc->stackProfile,
    pNwkDesc->permitJoining );

    pNwkDesc++;
  }
  AT_NEW_LINE();
  AT_OK();
  notdoSCANCmd = TRUE;
}

/*******************************************************************************
 * @fn      AT_Cmd_FN_CB
 *
 * @brief   Response: JPAN:<channel>,<PANID>,<EPANID>
 *                    OK or ERROR:<errorcode> .
 *
 *                    <channel> - represents the channel,
 *                    <PANID>   - the PAN ID,
 *                    <EPANID>  - the extended PAN ID
 *
 * @param   Status - Result of NLME_NetworkFormationRequest()
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_FN_CB( ZStatus_t Status )
{
  // recover the parameters
  // set BDB channel attribute to default
  bdb_setChannelAttribute( TRUE, BDB_DEFAULT_PRIMARY_CHANNEL_SET );
  bdb_setChannelAttribute( FALSE, BDB_DEFAULT_SECONDARY_CHANNEL_SET );

  zgConfigPANID = 0xFFFF;
  uint8 status = osal_nv_item_init( ZCD_NV_PANID, sizeof(zgConfigPANID), &zgConfigPANID );
  if ( status == ZSUCCESS ) { // TODO: if status != ZSUCCESS, should we do something??
    osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
  }

  AT_NEW_LINE();
  if (Status == ZSUCCESS) {
    AT_OK();
    AT_NEXT_LINE();
    uint16* ext = (uint16*)_NIB.extendedPANID;
    printf("JPAN:%02d,%04X,%04X%04X%04X%04X", _NIB.nwkLogicalChannel,
            _NIB.nwkPanId,ext[3],ext[2],ext[1],ext[0]);
  } else {
    AT_ZDO_ERROR(Status);
  }
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessJOIN_CNF_CB
 *
 * @brief   Response: JPAN:<channel>,<PANID>,<EPANID>
 *                    OK or ERROR:<errorcode> .
 *
 *                    <channel> - represents the channel,
 *                    <PANID>   - the PAN ID,
 *                    <EPANID>  - the extended PAN ID
 *
 * @param   param - Join confirm message
 *
 * @return  None
 ******************************************************************************/
void* AT_ZDO_ProcessJOIN_CNF_CB(void *param)
{
    zdoJoinCnf_t *joinCnf = (zdoJoinCnf_t*) param;

    // recover the parameters
    // set BDB channel attribute to default
    bdb_setChannelAttribute( TRUE, BDB_DEFAULT_PRIMARY_CHANNEL_SET );
    bdb_setChannelAttribute( FALSE, BDB_DEFAULT_SECONDARY_CHANNEL_SET );

    zgConfigPANID = 0xFFFF;
    uint8 status = osal_nv_item_init( ZCD_NV_PANID, sizeof(zgConfigPANID), &zgConfigPANID );
    if ( status == ZSUCCESS ) { // TODO: if status != ZSUCCESS, should we do something??
      osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
    }

    // invalid address enable the device allow all the ExtPANid
    osal_memcpy( ZDO_UseExtendedPANID, "\0\0\0\0\0\0\0\0", 8);

    //cancel the call, because the command will finished in this function
    ZDO_RegisterForZdoCB(ZDO_JOIN_CNF_CBID, NULL);

    AT_NEW_LINE();
    if (joinCnf->status == ZSUCCESS) {
      AT_OK();
      AT_NEXT_LINE();
      uint16* ext = (uint16*)_NIB.extendedPANID;
      printf("JPAN:%02d,%04X,%04X%04X%04X%04X", _NIB.nwkLogicalChannel,
              _NIB.nwkPanId, ext[3], ext[2], ext[1], ext[0]);
    } else {
      // This function is called by ZDO_JoinConfirmCB
      // only return join error status, so we can not catche errors
      // from NLME_NetworkDiscoveryRequest and bdb_rejoinNwk
      AT_ZDO_ERROR(joinCnf->status);
    }
    AT_NEW_LINE();
    return NULL;
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessPowerDescRsp
 *
 * @brief   Prompt: PowerDesc:<NodeID>,<errorcode>[,<PowerDescriptor>]
 *                  <NodeID> - the Remote node’s Node ID.
 *                  <PowerDescriptor> - displayed as a 16 bit hexadecimal number
 *                                      as described in section 2.3.2.4. of
 *                                      ZigBee Pro Specification.
 *          Note: In case of an error an errorcode other than 00 will be displayed
 *                and the prompt will end after the errorcode
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessPowerDescRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_PowerRsp_t pRsp;
  ZDO_ParsePowerDescRsp( inMsg, &pRsp );

  AT_NEW_LINE();
  printf("PowerDesc:%04X,%02X", pRsp.nwkAddr, pRsp.status);
  if (pRsp.status == ZDP_SUCCESS) {
    AT_NEXT_LINE();
    printf("PowerMode:%02X", pRsp.pwrDesc.PowerMode);
    AT_NEXT_LINE();
    printf("AvailablePowerSources:%02X", pRsp.pwrDesc.AvailablePowerSources);
    AT_NEXT_LINE();
    printf("CurrentPowerSource:%02X", pRsp.pwrDesc.CurrentPowerSource);
    AT_NEXT_LINE();
    printf("CurrentPowerSourceLevel:%02X", pRsp.pwrDesc.CurrentPowerSourceLevel);
  }
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessActEpRsp
 *
 * @brief   Prompt: ActEpDesc:<NodeID>,<errorcode>[,XX,…]
 *                  <NodeID> - the Remote node’s Node ID.
 *                  [,XX,...] - active endpoint listed as 8-bit hexadecimal
 *                              numbers seperated by commas
 *          Note: In case of an error an errorcode other than 00 will be displayed
 *                and the prompt will end after the errorcode
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessActEpRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_ActiveEndpointRsp_t * pRsp = ZDO_ParseEPListRsp( inMsg );

  AT_NEW_LINE();
  printf("ActEpDesc:%04X,%02X", pRsp->nwkAddr, pRsp->status);
  if(pRsp->status == ZDP_SUCCESS){
    uint8 i;
    for (i = 0; i < pRsp->cnt; i++) {
      printf(",%02X", pRsp->epList[i]);
    }
  }
  AT_NEW_LINE();
  osal_mem_free(pRsp);
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessSimpleDescRsp
 *
 * @brief   Prompt: SimpleDesc:<NodeID>,<errorcode>
 *                  EP:XX
 *                  ProfileID:XXXX
 *                  DeviceID:XXXXvXX
 *                  InCluster:<Cluster List>
 *                  OutCluster:<Cluster List>
 *          Note: In case of an error an errorcode other than 00 will be displayed
 *                and the prompt will end after the errorcode
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessSimpleDescRsp( zdoIncomingMsg_t *inMsg )
{
  notdoSIMPLEDESCcmd = TRUE;
  ZDO_SimpleDescRsp_t Rsp;
  ZDO_ParseSimpleDescRsp( inMsg, &Rsp );

  AT_NEW_LINE();
  printf("SimpleDesc:%04X,%02X", Rsp.nwkAddr, Rsp.status);
  if (Rsp.status == ZDP_SUCCESS) {
    AT_NEXT_LINE();
    printf("EP:%02X", Rsp.simpleDesc.EndPoint);
    AT_NEXT_LINE();
    printf("ProfileID:%04X", Rsp.simpleDesc.AppProfId);
    AT_NEXT_LINE();
    printf("DeviceID:%04Xv%02X", Rsp.simpleDesc.AppDeviceId, Rsp.simpleDesc.AppDevVer);
    AT_NEXT_LINE();
    uint8 i;
    printf("InCluster: ");
    for (i = 0; i < Rsp.simpleDesc.AppNumInClusters; i++) {
        if(i) printf("           %04X",Rsp.simpleDesc.pAppInClusterList[i]);
        else printf("%04X",Rsp.simpleDesc.pAppInClusterList[i]);
        AT_NEXT_LINE();
    }

    printf("OutCluster: ");
    for (i = 0; i < Rsp.simpleDesc.AppNumOutClusters; i++) {
        if(i) printf("            %04X", Rsp.simpleDesc.pAppOutClusterList[i]);
        else printf("%04X",Rsp.simpleDesc.pAppOutClusterList[i]);
        if(i < Rsp.simpleDesc.AppNumOutClusters-1) AT_NEXT_LINE();
    }
  }
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMatchDescRsp
 *
 * @brief   Prompt: MatchDesc:<NodeID>,<errorcode>,<XX>
 *                  Where <NodeID> is the Remote node's NodeID. In addition all
 *                  endpoints of this node matching the search criterion are listed
 *                  as 8 bit hexadecimal numbers separated by commas.
 *          Note: In case of an error an errorcode other than 00 will be displayed
 *                and the prompt will end after the errorcode
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMatchDescRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );

  AT_NEW_LINE();
  printf("MatchDesc:%04X,%02X", pRsp->nwkAddr, pRsp->status);
  if (pRsp->status == ZDP_SUCCESS) {
    uint8 i;
    for (i = 0; i < pRsp->cnt; i++) {
      printf(",%02X", pRsp->epList[i]);
    }
  }
  AT_NEW_LINE();
  osal_mem_free(pRsp);
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessAnnceRsp
 *
 * @brief   Prompt: FFD:<EUI64>,<NodeID>
 *          Note: The prompt above will be displayed on all nodes which can
 *                hear the announcement.
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessAnnceRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_DeviceAnnce_t Rsp;
  ZDO_ParseDeviceAnnce( inMsg,&Rsp );

  uint16* ext= (uint16*) Rsp.extAddr;
  AT_NEW_LINE();
  printf("%s:%04X%04X%04X%04X,%04X",
         ((Rsp.capabilities&0x01) == CAPINFO_ALTPANCOORD) ? "COORD" :
         (((Rsp.capabilities&0x02) == CAPINFO_DEVICETYPE_FFD) ? "FFD" : "RFD")
         ,ext[3],ext[2],ext[1],ext[0],Rsp.nwkAddr);
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMgmtLqiRsp
 *
 * @brief   Prompt(example)
 *          NTable:<NodeID>,<errorcode>
 *          Length:03
 *          No.| Type | Relation |       EUI        |  ID  | LQI
 *          0. |  FFD |  PARENT  | 000D6F000015896B | BC04 | FF
 *          1. |  FFD |  CHILD   | 000D6F00000B3E77 | 739D | FF
 *          2. |  FFD |  SIBLING | 000D6F00000AAD11 | 75E3 | FF
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMgmtLqiRsp( zdoIncomingMsg_t *inMsg )
{
  uint8 i;
  ZDO_MgmtLqiRsp_t *pRsp = ZDO_ParseMgmtLqiRsp( inMsg );

  AT_NEW_LINE();
  // NTable:<NodeID>,<errorcode>
  if (inMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    printf("NTable:%04X,%02X\r\n",(uint16)inMsg->srcAddr.addr.shortAddr,
           pRsp->status);
  }

  // Length:XX
  printf("Length:%02X\r\n", pRsp->neighborLqiCount);

  // No.| Type | Relation | EUI | ID | LQI
  if(pRsp->neighborLqiCount != 0)
    printf("No.| Type | Relation |       EUI        |  ID  | LQI\n\r");
  for (i = 0; i < pRsp->neighborLqiCount; i++) {
    uint16 *ext = (uint16*) pRsp->list[i].extAddr;
    // print No.
    printf("\r\n%02X.| ",i+pRsp->startIndex);
    // print type
    if (pRsp->list[i].devType == ZDP_MGMT_DT_COORD)
      printf(" %-3s | ", "COO");
    else if (pRsp->list[i].devType == ZDP_MGMT_DT_ROUTER)
      printf(" %-3s | ", "RTR");
    else if (pRsp->list[i].devType == ZDP_MGMT_DT_ENDDEV)
      printf(" %-3s | ", "ZED");
    else
      printf(" %-3s | ", "UKN");  // for the unknown devices
    // print relation
    if (pRsp->list[i].relation == ZDP_MGMT_REL_PARENT)
      printf(" %-8s| ","PARENT");
    else if (pRsp->list[i].relation == ZDP_MGMT_REL_CHILD)
      printf(" %-8s| ","CHILD");
    else if (pRsp->list[i].relation == ZDP_MGMT_REL_SIBLING)
      printf(" %-8s| ","SIBLING");
    else
      printf(" %-8s| ","UNKNOWN");
    //print EUI ID LQI
    printf("%04X%04X%04X%04X | %04X | %02X",
           ext[3],ext[2],ext[1],ext[0],pRsp->list[i].nwkAddr,
           pRsp->list[i].lqi);
  }
  AT_NEW_LINE();
  osal_mem_free(pRsp);
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMgmtRtgRsp
 *
 * @brief   Prompt (example)
 *          RTable:<NodeID>,<errorcode>
 *          Length:03
 *          No.| Dest | Next | Status
 *          0. | 1234 | ABCD | 00
 *          1. | 4321 | 739D | 00
 *          2. | 0000 | 0000 | 03
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMgmtRtgRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmtRtgRsp_t* pRsp = (ZDO_MgmtRtgRsp_t*)ZDO_ParseMgmtRtgRsp(inMsg );

  AT_NEW_LINE();

  // RTable:<NodeID>,<errorcode>
  if (inMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    printf("RTable:%04X,%02X\r\n", (uint16)inMsg->srcAddr.addr.shortAddr,
           pRsp->status);
  }
  // Length:XX
  printf("Length:%02X\r\n", pRsp->rtgCount);

  // No.| Dest | Next | Status
 if (pRsp->rtgCount != 0) printf("No.| Dest | Next | Status\n\r");

  uint8 i;
  for (i = 0; i < pRsp->rtgListCount; i++) {
    // print No.| Dest | Next | Status
    printf("\r\n%02X.| %04X | %04X | %02X",
           i+pRsp->startIndex,
           pRsp->list[i].dstAddress,
           pRsp->list[i].nextHopAddress,
           pRsp->list[i].status);
  }
  AT_NEW_LINE();
  osal_mem_free( pRsp );
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMgmtBindRsp
 *
 * @brief   Prompt (example)
 *          AT+BTABLE:00,0000
 *          SEQ:01
 *          OK
 *          BTable:0000,00
 *          Length:03
 *          No. | SrcAddr | SrcEP | ClusterID | DstAddr | DstEP
 *          00. | 000D6F000059474E | 01 | DEAD |1234567887654321 | 12
 *          01. | 000D6F000059474E | 01 | DEAD |E012345678876543 | E0
 *          02. | 000D6F000059474E | 01 | DEAD | ABCD
 *          ACK:01
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMgmtBindRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmtBindRsp_t *pRsp = ZDO_ParseMgmtBindRsp( inMsg );

  uint8 i;
  char dstAddr[17];
  char dstEP[5];
  zAddrType_t* addr;

  AT_NEW_LINE();

  // BTable:<NodeID>,<errorcode>
  if (inMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    printf("BTable:%04X,%02X\r\n", (uint16)inMsg->srcAddr.addr.shortAddr,
           pRsp->status);
  }
  // Length:XX
  printf("Length:%02X", pRsp->bindingListCount);
  AT_NEXT_LINE();

  if(pRsp->bindingListCount)
    printf("No. |     SrcAddr      | SrcEP | ClusterID |     DstAddr      | DstEP");
  for (i = 0; i < pRsp->bindingListCount; i++) {
    addr = &pRsp->list[i].dstAddr;
    AT_NEXT_LINE();
    if (addr->addrMode == (afAddrMode_t)Addr64Bit) {
      AT_GetIEEEAddrStr(addr->addr.extAddr, (uint8*)dstAddr);
      sprintf(dstEP, "| %02X", pRsp->list[i].dstEP);
      dstAddr[16]='\0';
    }else{
      sprintf(dstAddr,"%04X",addr->addr.shortAddr);
      dstEP[0]='\0';
    }
    uint16 *srcAddr= (uint16*) pRsp->list[i].srcAddr;
    printf("%02X. | %04X%04X%04X%04X |  %02X   |   %04X    | %s %s ",
           i+pRsp->startIndex,srcAddr[3],srcAddr[2],srcAddr[1],srcAddr[0],
           pRsp->list[i].srcEP, pRsp->list[i].clusterID, dstAddr, dstEP);

  }
  AT_NEW_LINE();

  osal_mem_free(pRsp);
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessBindRsp
 *
 * @brief   +BIND - Create Binding on Remote Device (ZDO)
 *          Prompt - Bind:<NodeID>,<status>
 *
 *                   ACK:XX
 *          Note: In case of an error an status other than 00 will be displayed
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessBindRsp( zdoIncomingMsg_t *inMsg )
{
  uint8 status = ZDO_ParseBindRsp(inMsg);

  AT_NEW_LINE();
  if (inMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    printf("Bind:%04X,%02X", inMsg->srcAddr.addr.shortAddr, status);
  }else{
    printf("Bind:UNKNOWN,%02X", status);
  }
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessUnbindRsp
 *
 * @brief   +UNBIND - Delete Binding on Remote Device (ZDO)
 *          Prompt - Unbind:<NodeID>,<status>
 *
 *                   ACK:XX
 *          Note: In case of an error an status other than 00 will be displayed
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessUnbindRsp( zdoIncomingMsg_t *inMsg )
{
  uint8 status = ZDO_ParseBindRsp(inMsg);

  AT_NEW_LINE();
  if (inMsg->srcAddr.addrMode == (afAddrMode_t)Addr16Bit) {
    printf("Unbind:%04X,%02X", inMsg->srcAddr.addr.shortAddr, status);
  }else{
    printf("Unbind:UNKNOWN,%02X", status);
  }
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessEDbindRsp
 *
 * @brief   +EBIND - End Device Bind
 *          Prompt - EBINDACK
 *                   EBINDRSP:<Status>
 *                   OK
 *          Note: In case of an error an status other than 00 will be displayed
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessEDbindRsp( zdoIncomingMsg_t *inMsg )
{
  uint8 status = ZDO_ParseBindRsp(inMsg);

  AT_NEW_LINE();
  // AT_RESP("EBINDACK", 8);
  // AT_NEXT_LINE();
  printf("EBINDRSP:%02X", status);
  AT_NEW_LINE();
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_ZDO_ProcessMgmtLeaveRsp
 *
 * @brief   Prompt - LEFTPAN
 *          Note: Instruct device to leave the PAN
 *
 * @param   inMsg - incoming message (response)
 *
 * @return  None
 ******************************************************************************/
void AT_ZDO_ProcessMgmtLeaveRsp( zdoIncomingMsg_t *inMsg )
{
  uint8 status = ZDO_ParseMgmtLeaveRsp( inMsg );

  if ( (uint16)inMsg->srcAddr.addr.shortAddr == NLME_GetShortAddr() ) {
    AT_NEW_LINE();
    if (status == ZSuccess) {
      printf("\r\nLEFTPAN\r\n");
    }
    AT_NEW_LINE();
  }
}

/******************************************************************************
 ******************************************************************************/
