/**************************************************************************************************
  Filename:       ATApp.c

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application is a template to get started writing an application
  from scratch.

  Look for the sections marked with "ATAPP_TODO" to add application
  specific code.

  Note: if you would like your application to support automatic attribute
  reporting, include the BDB_REPORTING compile flag.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDNwkMgr.h"
//#include "MT_SYS.h"
#include "OnBoard.h"

#include "nwk_util.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_diagnostic.h"
#include "ATApp.h"

#include "bdb.h"
#include "bdb_interface.h"

#if defined ( INTER_PAN )
#if defined ( BDB_TL_INITIATOR )
  #include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR
#if defined ( BDB_TL_TARGET )
  #include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET
#endif // INTER_PAN

#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "bdb_touchlink.h"
#endif

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

/* User defiene */
#include "AT_controller.h"
#include "AT_uart.h"
#include "AT_printf.h"
#include "AT_cmd.h"
#if defined ( INTER_PAN )
#include "InterPAN.h"
#include "stub_aps.h"
#endif

#include "AT_uart0.h"
#include "zcl_doorlock.h"

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
byte zclATApp_TaskID;
uint8 gChannel;
uint8 gNwkUpdateId;
bool getNewAddr = false;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 gPermitDuration = 0;    // permit joining default to disabled

uint8 gNwkUpdateTime = 0;

uint32 NwkSteeringTime = 3000;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AT_RegisterSimpleDescriptor( SimpleDescriptionFormat_t *simpleDesc );
static void zclATApp_HandleKeys( byte shift, byte keys );
static void zclATApp_HandleStateChange( uint8 state );
static void zclATApp_BindNotification( bdbBindNotificationData_t *data );
static void zclATApp_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);

/*********************************************************************
 * @fn          zclATApp_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclATApp_Init( byte task_id )
{
  zclATApp_TaskID = task_id;

  // This app is part of the User defined Profile
  AT_RegisterSimpleDescriptor( &ATApp_SimpleDesc );

  // Register the ZCL controller
  AT_zclController_Init( &zclATApp_TaskID );

  // Initialise UART
//  AT_UART_Init( zclATApp_TaskID );
  AT_Uart0_Init();
  printf("System starting...\r\n\r\n");

  // Register ZDO messages
  AT_ZDO_Register( &zclATApp_TaskID );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclATApp_TaskID );

  bdb_RegisterCommissioningStatusCB( zclATApp_ProcessCommissioningStatus );
  bdb_RegisterBindNotificationCB( zclATApp_BindNotification );

  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING );
}

/*********************************************************************
 * @fn      AT_RegisterSimpleDescriptor
 *
 * @brief   Fill the Simple descriptor and register it with the AF
 *
 * @param   simpleDesc - a pointer to a valid SimpleDescriptionFormat_t, must not be NULL.
 *
 * @return  none
 */
void AT_RegisterSimpleDescriptor( SimpleDescriptionFormat_t *simpleDesc )
{
  endPointDesc_t *epDesc;

  // Register the application's endpoint descriptor
  //  - This memory is allocated and never freed.
  epDesc = osal_mem_alloc( sizeof ( endPointDesc_t ) );
  if ( epDesc )
  {
    // Fill out the endpoint description.
    epDesc->endPoint = simpleDesc->EndPoint;
    epDesc->task_id = &zclATApp_TaskID;  // all messages send to this app
    epDesc->simpleDesc = simpleDesc;
    epDesc->latencyReq = noLatencyReqs;

    // Register the endpoint description with the AF
    afRegister( epDesc );
    #if defined (INTER_PAN)
  	// Register the endpoint description for INTERPAN
    StubAPS_RegisterApp( epDesc );
    #endif
  }
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclATApp_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclATApp_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case AF_INCOMING_MSG_CMD:
          switch (MSGpkt->srcAddr.endPoint)
          {
            #if defined (INTER_PAN)
            case STUBAPS_INTER_PAN_EP:
              InterPAN_ProcessMSGCB( MSGpkt );
              break;
            #endif

            default:
              ATApp_MessageMSGCB( MSGpkt );
              break;
          }
          break;
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          AT_zclController_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclATApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclATApp_HandleStateChange( MSGpkt->hdr.status );
          break;

        case ZDO_CB_MSG:
          AT_ZDO_ProcessMsgCBs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

#if ZG_BUILD_ENDDEVICE_TYPE
  if ( events & ATAPP_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ ATAPP_END_DEVICE_REJOIN_EVT );
  }
#endif

  if( events & AT_RESET_EVENT ){
    SystemReset();
  }

  if( events & AT_RESTORE_CMDDO_FLAG ){
    notdoFNCmd = TRUE;
    notdoJNCmd = TRUE;
    notdoSCANCmd = TRUE;
  	return ( events ^ AT_RESTORE_CMDDO_FLAG );
  }

  if ( events & AT_NWKUPDATE_EVENT )
  {
    //printf("nwkupdate\r\n");
    uint8 rxOnIdle;
    uint8 currChannel;
    gNwkUpdateTime++;
    if(gNwkUpdateTime == 3)
    {
      gNwkUpdateTime = 0;
      ZMacGetReq( ZMacChannel, &currChannel );
      if ( currChannel != gChannel ) {
        // turn MAC receiver off
        rxOnIdle = false;
        ZMacSetReq( ZMacRxOnIdle, &rxOnIdle );

        // set the NIB channel
        ZMacSetReq( ZMacChannel, &gChannel );

        // turn MAC receiver back on
        rxOnIdle = true;
        ZMacSetReq( ZMacRxOnIdle, &rxOnIdle );
        _NIB.nwkLogicalChannel = gChannel;
        // Our Channel has been changed -- notify to save info into NV
        ZDApp_NwkStateUpdateCB();
      }
      AT_OK();
    } else {
      // Build dstAddress
      zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
      dstAddr->addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
      dstAddr->addrMode = (afAddrMode_t)AddrBroadcast;

      ZDP_MgmtNwkUpdateReq(dstAddr, ((uint32)1 << gChannel), 0xFE, 0, gNwkUpdateId, 0);
      osal_mem_free(dstAddr);

      //send three times ensure network is updated
      osal_start_timerEx( zclATApp_TaskID, AT_NWKUPDATE_EVENT, ZDNWKMGR_BCAST_DELIVERY_TIME+50 );
    }

    return ( events ^ AT_NWKUPDATE_EVENT );
  }

  if ( events & AT_BDB_STEERING_EVT )
  {
    bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING );

    return ( events ^ AT_BDB_STEERING_EVT );
  }

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      zclATApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclATApp_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_7 )
  {
  }
  if ( keys & HAL_KEY_SW_6 )
  {
  }
  if ( keys & HAL_KEY_SW_5 )
  {
  }
}

static void zclATApp_HandleStateChange( uint8 state )
{
  static uint8 joinCnt = 0;
  static uint8 rejoinCnt = 0;

#if AT_SHOW_STATE_CHANGE
  printf("NwkState:%s\r\n",devStates_str[state]);
#endif

  switch( (devStates_t)state )
  {
    case DEV_HOLD:
      joinCnt++;
      printf("join failure %d", joinCnt);
      if(joinCnt < ATAPP_END_DEVICE_JOIN_TIMES)
      {
        printf(", join again after %d s\r\n\r\n", ATAPP_END_DEVICE_JOIN_DELAY/1000);
        osal_start_timerEx( zclATApp_TaskID, AT_BDB_STEERING_EVT, ATAPP_END_DEVICE_JOIN_DELAY );
      }
      else
      {
        joinCnt = 0;
        printf(", quit join\r\n\r\n");
      }
      break;

    case DEV_END_DEVICE:
      osal_start_timerEx(zclDoorLock_TaskID, DOORLOCK_SET_DORMANT_EVT, DOORLOCK_KEEP_ACTIVE_TIME_START);
      joinCnt = 0;
      rejoinCnt = 0;
      printf("join success\r\n\r\n");
      osal_start_reload_timer(zclDoorLock_TaskID, DOORLOCK_KEEP_ALIVE_EVT, DOORLOCK_KEEP_ALIVE_TIMEOUT);
      printf("enable DOORLOCK_KEEP_ALIVE_EVT\r\n");
      break;

    case DEV_NWK_ORPHAN:
      if(rejoinCnt < 10)
      {
        rejoinCnt++;
        printf("parent lost %d, rejoin after %d s\r\n\r\n", rejoinCnt, ATAPP_END_DEVICE_REJOIN_DELAY_1/1000);
        osal_start_timerEx(zclATApp_TaskID, ATAPP_END_DEVICE_REJOIN_EVT, ATAPP_END_DEVICE_REJOIN_DELAY_1);
      }
      else if(rejoinCnt < 20)
      {
        rejoinCnt++;
        printf("parent lost %d, rejoin after %d s\r\n\r\n", rejoinCnt, ATAPP_END_DEVICE_REJOIN_DELAY_2/1000);
        osal_start_timerEx(zclATApp_TaskID, ATAPP_END_DEVICE_REJOIN_EVT, ATAPP_END_DEVICE_REJOIN_DELAY_2);
      }
      else
      {
        printf("parent lost %d, rejoin after %d s\r\n\r\n", rejoinCnt, ATAPP_END_DEVICE_REJOIN_DELAY_3/1000);
        osal_start_timerEx(zclATApp_TaskID, ATAPP_END_DEVICE_REJOIN_EVT, ATAPP_END_DEVICE_REJOIN_DELAY_3);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      zclATApp_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclATApp_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        if (!getNewAddr)
        { // if get new address, devive would has annaounced
          ZDP_DeviceAnnce( NLME_GetShortAddr(), NLME_GetExtAddr(), _NIB.CapabilityFlags, 0 );
        }  
      }
      else
      {
        //See the possible errors for nwk steering procedure
        //No suitable networks found
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification

      //YOUR JOB:
      //We are on a network, what now?

    break;
#if ZG_BUILD_ENDDEVICE_TYPE
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
      }
    break;
#endif
  }
}

/*********************************************************************
 * @fn      zclATApp_BindNotification
 *
 * @brief   Called when a new bind is added.
 *
 * @param   data - pointer to new bind data
 *
 * @return  none
 */
static void zclATApp_BindNotification( bdbBindNotificationData_t *data )
{
  // ATAPP_TODO: process the new bind information
}

/****************************************************************************
****************************************************************************/
