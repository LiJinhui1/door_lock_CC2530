/******************************************************************************
  Filename:       InterPAN.c

  Description:    Inter-PAN message Process
  Author:         Yasin Zhang
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "ZComDef.h"
#include "AF.h"
#include "OnBoard.h"

#if defined ( INTER_PAN )
  #include "stub_aps.h"
#endif

#include "AT_printf.h"
#include "ATApp.h"

#include "InterPAN.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
byte InterPAN_TransID;  // This is the InterPAN message counter

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*****************************************************************************
* @fn      InterPAN_Send
*
* @brief   Send Inter-PAN message
*          INTERPAN:<DstAddress>,<DstChannel>,<DstPANID>,<ClusterID>,<Data>
*
* @param   uint8 *dstAddr   - remote device's NodeID
* @param   uint8 *dstCH   - remote device's channel
* @param   uint8 *dstPan  - remote device's PANID
* @param   uint8 *clusterId - message's Cluster ID
* @param   uint8 *rawData - Inter-PAN message
*
* @return  status
******************************************************************************/
uint8 InterPAN_Send( afAddrType_t *dstAddr, uint8 ch, uint16 cId, uint8 dataLen, uint8 *rawData )
{
  uint8 currChannel;
  uint8 rtrn = FAILURE;
  endPointDesc_t *pEP = NULL;

  if ((pEP = afFindEndPointDesc(ATApp_ENDPOINT)) != NULL)
  {
    // StubAPS_RegisterApp(pEP);
    if (!StubAPS_SetInterPanChannel(ch)) { // Set channel for inter-pan communication.
      ZMacGetReq( ZMacChannel, &currChannel );
      rtrn = AF_DataRequest( dstAddr,
                             pEP,
                             cId,
                             (byte)dataLen,
                             (byte *)rawData,
                             &InterPAN_TransID,
                             AF_DISCV_ROUTE,
                             AF_DEFAULT_RADIUS
                           );
      if (StubAPS_SetIntraPanChannel()) { // Switch channel back to the NIB channel
        printf("Switch back channel failed!\r\n");
        return FAILURE;
      }
    } else {
      printf("Change InterPAN channel failed!\r\n");
      return FAILURE;
    }
  }
  else
  {
    printf("No such endpoint!\r\n");
    return FAILURE;
  }

  return rtrn;
}

/*****************************************************************************
* @fn      InterPAN_ProcessMSGCB
*
* @brief   Process Inter-PAN message
*
* @param   afIncomingMSGPacket_t *pkt
*
* @return  status
******************************************************************************/
void InterPAN_ProcessMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case ATApp_GENERIC_CLUSTER:
      printf("INTERPAN: \r\n");
      HalUARTWrite(HAL_UART_PORT_0, pkt->cmd.Data, pkt->cmd.DataLength);
      printf("\r\n");
      break;
  }
}
