/*********************************************************************
  Filename:       InterPAN.h

  Author:         Yasin Zhang
*********************************************************************/

#ifndef INTERPAN_H
#define INTERPAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern byte InterPAN_TransID;

/*********************************************************************
* FUNCTIONS
*/
extern uint8 InterPAN_Send( afAddrType_t *dstAddr, uint8 ch, uint16 cId, uint8 dataLen, uint8 *rawData );
extern void InterPAN_ProcessMSGCB( afIncomingMSGPacket_t *pkt );


#ifdef __cplusplus
}
#endif

#endif /* INTERPAN_H */
