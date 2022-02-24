/*****************************************************************************
  Filename:       zcl_doorlock.c
  Author:         Yang Wang

  Description:    Zigbee Cluster Library - sample device application.
******************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "bdb.h"
#include "bdb_interface.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_closures.h"
#include "zcl_ha.h"

#include "AT_controller.h"
#include "AT_printf.h"
#include "zcl_doorlock.h"

#include "onboard.h"

/* HAL */
#include "hal_led.h"

/* Device */
#include "AT_doorlock.h"
#include "AT_single_bus.h"
#include "AT_pwr_single_bus.h"
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
byte zclDoorLock_TaskID;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static ZStatus_t zclDoorLock_Report(uint8 endpoint, uint16 clusterId, uint16 attrId, uint8 dataType, void *dataPtr);
static void zclDoorLock_BasicResetCB( void );

// Functions to process ZCL Foundation incoming Command/Response messages
static ZStatus_t zclDoorLock_DoorLockCB ( zclIncoming_t *pInMsg, zclDoorLock_t *pInCmd );
static ZStatus_t zclDoorLock_DoorLockSetTemporaryPinCB ( zclIncoming_t *pInMsg, zclDoorLockSetTemporaryPin_t *pCmd );

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclDoorLock_CmdCallbacks =
{
  zclDoorLock_BasicResetCB,               // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  NULL,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * ZCL Closure cluster Callback table
 */
static zclClosures_DoorLockAppCallbacks_t zclDoorLock_DoorLockCmdCallbacks =
{
  zclDoorLock_DoorLockCB,                 // Lock/Unlock/Toggle command
  NULL,
  NULL,                                   // Unlock With Timeout command
  NULL,                                   // Get Log Record command
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  zclDoorLock_DoorLockSetTemporaryPinCB,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

static ZStatus_t zclDoorLock_Report(uint8 endpoint, uint16 clusterId, uint16 attrId, uint8 dataType, void *dataPtr)
{
  ZStatus_t result = ZFailure;

  afAddrType_t dstAddr;
  dstAddr.addr.shortAddr = NWK_PAN_COORD_ADDR;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.endPoint = 0x64;

  zclReportCmd_t *pReportCmd;
  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (1 * sizeof(zclReport_t)));
  if(pReportCmd != NULL)
  {
    pReportCmd->numAttr = 1;

    pReportCmd->attrList[0].attrID = attrId;
    pReportCmd->attrList[0].dataType = dataType;
    pReportCmd->attrList[0].attrData = dataPtr;

    result = zcl_SendReportCmd( endpoint,
                                &dstAddr,
                                clusterId,
                                pReportCmd,
                                ZCL_FRAME_SERVER_CLIENT_DIR,
                                BDB_REPORTING_DISABLE_DEFAULT_RSP,
                                bdb_getZCLFrameCounter());

    osal_mem_free(pReportCmd);
  }
  else
  {
    result = ZMemError;
  }

  return result;
}
/*********************************************************************
 * @fn          zclDoorLock_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclDoorLock_Init( byte task_id )
{
  zclDoorLock_TaskID = task_id;

  // This app is part of the Home Automation Profile
  bdb_RegisterSimpleDescriptor( &DoorLock_SimpleDesc );

//  // Register the ZCL endpoint control function
//  AT_Endpoint_Register(DOORLOCK_ENDPOINT, &zclDoorLock_OnOff, zclDoorLock_Enable);

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( DOORLOCK_ENDPOINT, &zclDoorLock_CmdCallbacks );

  // Register the application's attribute list
  zclDoorLock_ResetAttributesToDefaultValues();
  zcl_registerAttrList( DOORLOCK_ENDPOINT, zclDoorLock_NumAttributes, zclDoorLock_Attrs );

  //Register the ZCL DoorLock Cluster Library callback function
  zclClosures_RegisterDoorLockCmdCallbacks( DOORLOCK_ENDPOINT, &zclDoorLock_DoorLockCmdCallbacks );

  #ifdef ZCL_DISCOVER
    // Register the application's command list
    zcl_registerCmdList( DOORLOCK_ENDPOINT, zclCmdsArraySize, zclDoorLock_Cmds );
  #endif

  #ifdef ZCL_DIAGNOSTIC
    // Register the application's callback function to read/write attribute data.
    // This is only required when the attribute data format is unknown to ZCL.
    zcl_registerReadWriteCB( DOORLOCK_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

    if ( zclDiagnostic_InitStats() == ZSuccess )
    {
      // Here the user could start the timer to save Diagnostics to NV
    }
  #endif

  AT_DoorLock_Init();
}

/*********************************************************************
 * @fn          zclDoorLock_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclDoorLock_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclDoorLock_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & DOORLOCK_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclDoorLock_IdentifyTime > 0 )
      zclDoorLock_IdentifyTime--;
    // zclDoorLock_ProcessIdentifyTimeChange(DOORLOCK_ENDPOINT);

    return ( events ^ DOORLOCK_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & DOORLOCK_REPORT_LOCK_EVT )
  {
    zclDoorLock_LockState = CLOSURES_LOCK_STATE_LOCKED;

  #ifdef BDB_REPORTING
    ZStatus_t result = zclDoorLock_Report( DOORLOCK_ENDPOINT,
                                           ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
                                           ATTRID_CLOSURES_LOCK_STATE,
                                           ZCL_DATATYPE_ENUM8,
                                           (void *)&zclDoorLock_LockState);

    printf("AF, DOORLOCK_REPORT_LOCK_EVT: 0x%02X\r\n", result);
  #endif

    return ( events ^ DOORLOCK_REPORT_LOCK_EVT );
  }

  if ( events & DOORLOCK_REPORT_UNLOCK_EVT )
  {
    zclDoorLock_LockState = CLOSURES_LOCK_STATE_UNLOCKED;

  #ifdef BDB_REPORTING
    ZStatus_t result = zclDoorLock_Report( DOORLOCK_ENDPOINT,
                                           ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
                                           ATTRID_CLOSURES_LOCK_STATE,
                                           ZCL_DATATYPE_ENUM8,
                                           (void *)&zclDoorLock_LockState);

    printf("AF, DOORLOCK_REPORT_UNLOCK_EVT: 0x%02X\r\n", result);
  #endif

    // report the doorlock is relocked after DOORLOCK_RELOCK_TIMEOUT (s)
    osal_start_timerEx(zclDoorLock_TaskID, DOORLOCK_REPORT_LOCK_EVT, DOORLOCK_RELOCK_TIMEOUT);

    return ( events ^ DOORLOCK_REPORT_UNLOCK_EVT );
  }

  if ( events & DOORLOCK_SET_DORMANT_EVT )
  {
    NLME_SetPollRate(DOORLOCK_POLL_RATE_DORMANT);

    return ( events ^ DOORLOCK_SET_DORMANT_EVT );
  }

  if ( events & DOORLOCK_KEEP_ALIVE_EVT )
  {
  #ifdef BDB_REPORTING
    ZStatus_t result = zclDoorLock_Report( DOORLOCK_ENDPOINT,
                                           ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
                                           ATTRID_CLOSURES_LOCK_STATE,
                                           ZCL_DATATYPE_ENUM8,
                                           (void *)&zclDoorLock_LockState);

    printf("AF, DOORLOCK_KEEP_ALIVE_EVT: 0x%02X\r\n", result);
  #endif

    return ( events ^ DOORLOCK_KEEP_ALIVE_EVT );
  }

  if ( events & DOORLOCK_HANDLE_RSP_EVT )
  {
    AT_DoorLock_Handle_Rsp(single_bus_rcv_buf, single_bus_rcv_len);

    return ( events ^ DOORLOCK_HANDLE_RSP_EVT );
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclDoorLock_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclDoorLock_BasicResetCB( void )
{
  /* DoorLock_TODO: remember to update this function with any
     application-specific cluster attribute variables */

  zclDoorLock_ResetAttributesToDefaultValues();
}

/*********************************************************************
 * @fn      zclDoorLock_Enable
 *
 * @brief   Enable/disale the endpoint
 *
 * @param   none
 *
 * @return  none
 */
void zclDoorLock_Enable( bool isEnable)
{
  if (isEnable) {
//    zclDoorLock_OnOffCB(COMMAND_ON);
  } else {
    zclDoorLock_BasicResetCB();
  }
}

/*********************************************************************
 * @fn      zclDoorLock_DoorLockCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Door Lock cluster Command for this application.
 *
 * @param   pInMsg - process incoming message
 * @param   pInCmd - PIN/RFID code of command
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclDoorLock_DoorLockCB ( zclIncoming_t *pInMsg, zclDoorLock_t *pInCmd )
{
  uint8 newDoorLockState;
  uint8 cmd_start = CMD_START;
  uint8 cmd_end = CMD_END;
  uint8 cmd_locate_shutdown = CMD_LOCATE_SHUTDOWN;
  
  //start upload data
  if (pInCmd->pPinRfidCode[5]==0x36)
  {
    printf("start upload data");
    AT_pwr_single_bus_output();
    AT_pwr_single_bus_send_byte(cmd_start);
    AT_pwr_single_bus_input();
  }
  //stop locate module power
    if ((pInCmd->pPinRfidCode[0]==0x37)||(pInCmd->pPinRfidCode[5]==0x37))
  {
    printf("stop locate module power");
    AT_pwr_single_bus_output();
    AT_pwr_single_bus_send_byte(cmd_locate_shutdown);
    AT_pwr_single_bus_input();
  }
  //stop upload data
    if ((pInCmd->pPinRfidCode[0]==0x38)||(pInCmd->pPinRfidCode[5]==0x38))
  {
    printf("stop upload data");
    AT_pwr_single_bus_output();
    AT_pwr_single_bus_send_byte(cmd_end);
    AT_pwr_single_bus_input();
  }
  // judge what kind of action should be taken
  if ( pInMsg->hdr.commandID == COMMAND_CLOSURES_LOCK_DOOR ) // Lock the door
  {
    newDoorLockState = CLOSURES_LOCK_STATE_LOCKED;
  }
  else if ( pInMsg->hdr.commandID == COMMAND_CLOSURES_UNLOCK_DOOR ) // Unock the door
  {
    newDoorLockState = CLOSURES_LOCK_STATE_UNLOCKED;
  }
  else // Toggle the door
  {
    if (zclDoorLock_LockState == CLOSURES_LOCK_STATE_LOCKED)
      newDoorLockState = CLOSURES_LOCK_STATE_UNLOCKED;
    else if (zclDoorLock_LockState == CLOSURES_LOCK_STATE_UNLOCKED)
      newDoorLockState = CLOSURES_LOCK_STATE_LOCKED;
  }

  if( newDoorLockState != zclDoorLock_LockState )
  {
    // take action
    if( newDoorLockState == CLOSURES_LOCK_STATE_LOCKED )
      AT_DoorLock_Lock( pInCmd ); // Lock the door
    else if( newDoorLockState == CLOSURES_LOCK_STATE_UNLOCKED )
      AT_DoorLock_Unlock( pInCmd ); // Unlock the door
  }

  return ( ZCL_STATUS_CMD_HAS_RSP );
}

static ZStatus_t zclDoorLock_DoorLockSetTemporaryPinCB ( zclIncoming_t *pInMsg, zclDoorLockSetTemporaryPin_t *pCmd )
{
  AT_DoorLock_SetTemporaryPin_Req ( pCmd );

  return ( ZCL_STATUS_CMD_HAS_RSP );
}
