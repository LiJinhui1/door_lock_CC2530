/******************************************************************************
  Filename:       AT_doorlock.c
  Author:         Yang Wang

  Description:    DoorLock Device controller
*******************************************************************************/
#include "AT_doorlock.h"
#include "AT_single_bus.h"
#include "AT_pwr_single_bus.h"
#include "hal_uart.h"
#include "OSAL.h"
#include "bdb_interface.h"
#include "zcl_doorlock.h"
#include "OSAL_Clock.h"
#include "AT_uart0.h"
#include "AT_printf.h"
#include "ATApp.h"

/******************************************************************************
                                  utilities
******************************************************************************/
static uint8 sum_check(uint8 *buf, uint8 len);
static uint32 UTCTime_To_SEC(uint8 *buf);
static uint8 AT_DoorLock_GetOperationEventSource( uint8 type );
static uint8 AT_DoorLock_GetProgramEventCode_Add_User( uint8 type );
static uint8 AT_DoorLock_GetProgramEventCode_Delete_User( uint8 type );
static ZStatus_t AT_DoorLock_SendOperationEventNotification( uint16 userID,
                                                             uint32 zigBeeLocalTime,
                                                             uint8 batteryLevel,
                                                             uint8 operationEventSource,
                                                             uint8 operationEventCode );
static ZStatus_t AT_DoorLock_SendProgrammingEventNotification( uint16 userID,
                                                               uint32 zigBeeLocalTime,
                                                               uint8 batteryLevel,
                                                               uint8 programEventSource,
                                                               uint8 programEventCode );

/******************************************************************************
                                   uplink
******************************************************************************/
static ZStatus_t AT_DoorLock_Rsp(uint8 *buf, uint8 len);
static ZStatus_t AT_DoorLock_Upload(uint8 *buf, uint8 len);
static ZStatus_t AT_DoorLock_Ack(uint8 *buf, uint8 len);

/******************************************************************************
                                  utilities
******************************************************************************/
static uint8 sum_check(uint8 *buf, uint8 len)
{
  uint8 i = 0;
  uint16 sum = 0;

  for(i=0;i<len;i++)
  {
    sum += buf[i];
  }

  return ((sum&0xFF)^0xFF);
}
static uint32 UTCTime_To_SEC(uint8 *buf)
{
  UTCTimeStruct UTCTime;
  UTCTime.year    = buf[0]+2000;
  UTCTime.month   = buf[1];
  UTCTime.day     = buf[2];
  UTCTime.hour    = buf[3];
  UTCTime.minutes = buf[4];
  UTCTime.seconds = buf[5];

  uint32 Seconds;
  Seconds = osal_ConvertUTCSecs( &UTCTime );

  return Seconds;
}
static uint8 AT_DoorLock_GetOperationEventSource( uint8 type )
{
  switch( type )
  {
    case WITH_FINGERPRINT:
      return OPERATION_EVENT_SOURCE_FINGERPRINT;
      break;

    case WITH_KEYPAD_CODE:
      return OPERATION_EVENT_SOURCE_KEYPAD;
      break;

    case WITH_RFID_CARD:
      return OPERATION_EVENT_SOURCE_RFID;
      break;

    case WITH_REMOTE_CONTROL:
      return OPERATION_EVENT_SOURCE_REMOTE_CONTROL;
      break;

    case WITH_MANUAL_KEY:
      return OPERATION_EVENT_SOURCE_MANUAL;// "Reserved" for Programming Event
      break;

    case WITH_TEMP_PIN:
      return OPERATION_EVENT_SOURCE_TEMP_PIN;
      break;

    case WITH_REMOTE_APP:
      return OPERATION_EVENT_SOURCE_RF;
      break;

    default:
      return OPERATION_EVENT_SOURCE_INDETERMINATE;
      break;
  }
}
static uint8 AT_DoorLock_GetProgramEventCode_Add_User( uint8 type )
{
  switch( type )
  {
    case WITH_FINGERPRINT:
      return PROGRAMMING_EVENT_CODE_FINGERPRINT_ADDED;
      break;

    case WITH_KEYPAD_CODE:
      return PROGRAMMING_EVENT_CODE_PIN_CODE_ADDED;
      break;

    case WITH_RFID_CARD:
      return PROGRAMMING_EVENT_CODE_RFID_CODE_ADDED;
      break;

    default:
      return PROGRAMMING_EVENT_CODE_UNKNOWN_OR_MFG_SPECIFIC;
      break;
  }
}
static uint8 AT_DoorLock_GetProgramEventCode_Delete_User( uint8 type )
{
  switch( type )
  {
    case WITH_FINGERPRINT:
      return PROGRAMMING_EVENT_CODE_FINGERPRINT_DELETED;
      break;

    case WITH_KEYPAD_CODE:
      return PROGRAMMING_EVENT_CODE_PIN_CODE_DELETED;
      break;

    case WITH_RFID_CARD:
      return PROGRAMMING_EVENT_CODE_RFID_CODE_DELETED;
      break;

    default:
      return PROGRAMMING_EVENT_CODE_UNKNOWN_OR_MFG_SPECIFIC;
      break;
  }
}
static ZStatus_t AT_DoorLock_SendOperationEventNotification( uint16 userID,
                                                             uint32 zigBeeLocalTime,
                                                             uint8 batteryLevel,
                                                             uint8 operationEventSource,
                                                             uint8 operationEventCode )
{
  ZStatus_t status;

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.addr.shortAddr = NWK_PAN_COORD_ADDR;// default send to Coordinator
  dstAddr.endPoint = 0x0B;//GENERIC_ENDPOINT on Coordinator

  //build DoorLock Operation Event Notification
  zclDoorLockOperationEventNotification_t pPayload;
  pPayload.operationEventSource = operationEventSource;
  pPayload.operationEventCode = operationEventCode;
  pPayload.userID = userID;
  pPayload.pin = batteryLevel; // it is battery level actually
  pPayload.zigBeeLocalTime = zigBeeLocalTime;

  pPayload.pData = (uint8 *)osal_mem_alloc(1);
  pPayload.pData[0] = 0;

  status = zclClosures_SendDoorLockOperationEventNotification( DOORLOCK_ENDPOINT,//uint8 srcEP,
                                                               &dstAddr,//afAddrType_t *dstAddr,
                                                               &pPayload,//zclDoorLockOperationEventNotification_t *pPayload,
                                                               TRUE,//uint8 disableDefaultRsp,
                                                               bdb_getZCLFrameCounter());//uint8 seqNum

  osal_mem_free( pPayload.pData );

  return status;
}
static ZStatus_t AT_DoorLock_SendProgrammingEventNotification( uint16 userID,
                                                               uint32 zigBeeLocalTime,
                                                               uint8 batteryLevel,
                                                               uint8 programEventSource,
                                                               uint8 programEventCode )
{
  ZStatus_t status;

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.addr.shortAddr = NWK_PAN_COORD_ADDR;// default send to Coordinator
  dstAddr.endPoint = 0x0B;//GENERIC_ENDPOINT on Coordinator

  //build DoorLock Programming Event Notification
  zclDoorLockProgrammingEventNotification_t pPayload;
  pPayload.programEventSource = programEventSource;
  pPayload.programEventCode = programEventCode;
  pPayload.userID = userID;
  pPayload.pin = batteryLevel; // it is battery level actually
  pPayload.userType = USER_TYPE_UNRESTRICTED_USER;
  pPayload.userStatus = USER_STATUS_AVAILABLE;
  pPayload.zigBeeLocalTime = zigBeeLocalTime;

  pPayload.pData = (uint8 *)osal_mem_alloc(1);
  pPayload.pData[0] = 0;

  status = zclClosures_SendDoorLockProgrammingEventNotification( DOORLOCK_ENDPOINT,//uint8 srcEP,
                                                                 &dstAddr,//afAddrType_t *dstAddr,
                                                                 &pPayload,//zclDoorLockProgrammingEventNotification_t *pPayload,
                                                                 TRUE,//uint8 disableDefaultRsp,
                                                                 bdb_getZCLFrameCounter());//uint8 seqNum

  osal_mem_free( pPayload.pData );

  return status;
}

/******************************************************************************
                                     init
******************************************************************************/
void AT_DoorLock_Init(void)
{
  AT_single_bus_init();
  AT_pwr_single_bus_init();
}

/******************************************************************************
                                   downlink
******************************************************************************/
ZStatus_t AT_DoorLock_Unlock( zclDoorLock_t *pInCmd )
{
  uint8 *buf = (uint8 *)osal_mem_alloc( LEN_DOORLOCK_UNLOCK );
  if(buf == NULL)
    return ZMemError;
  else
    osal_memset(buf, 0x00, LEN_DOORLOCK_UNLOCK);

  buf[0] = CMD_DOORLOCK_UNLOCK;

  if(pInCmd->pPinRfidCode[0] == PIN_LEN_NULL)
  {
    buf[1] |=  CHECK_PIN_Y_N;   // do not check pin
    buf[1] &= ~CHECK_SET_TIME;  // do not check and set time
  }
  else if((pInCmd->pPinRfidCode[0] >= PIN_LEN_MIN) && (pInCmd->pPinRfidCode[0] <= PIN_LEN_MAX))
  {
    buf[1] &= ~CHECK_PIN_Y_N;   // check pin
    buf[1] |=  CHECK_PIN_A_U_T; // check administrator/user/temp
    buf[1] &= ~CHECK_SET_TIME;  // do not check and set time

    osal_memcpy(&buf[10], &(pInCmd->pPinRfidCode[1]), pInCmd->pPinRfidCode[0]);
  }
  else
  {
    osal_mem_free( buf );
    return ZFailure;
  }

  buf[LEN_DOORLOCK_UNLOCK-1] = sum_check(buf, LEN_DOORLOCK_UNLOCK-1);

  AT_single_bus_send_buf(buf, LEN_DOORLOCK_UNLOCK);

  osal_mem_free( buf );

  return ZSuccess;
}
ZStatus_t AT_DoorLock_Lock( zclDoorLock_t *pInCmd )
{
  uint8 *buf = (uint8 *)osal_mem_alloc( LEN_DOORLOCK_LOCK );
  if(buf == NULL)
    return ZMemError;
  else
    osal_memset(buf, 0x00, LEN_DOORLOCK_LOCK);

  buf[0] = CMD_DOORLOCK_LOCK;

  if(pInCmd->pPinRfidCode[0] == PIN_LEN_NULL)
  {
    // do nothing
  }
  else if((pInCmd->pPinRfidCode[0] >= PIN_LEN_MIN) && (pInCmd->pPinRfidCode[0] <= PIN_LEN_MAX))
  {
    osal_memcpy(&buf[10], &(pInCmd->pPinRfidCode[1]), pInCmd->pPinRfidCode[0]);
  }
  else
  {
    osal_mem_free( buf );
    return ZFailure;
  }

  buf[LEN_DOORLOCK_LOCK-1] = sum_check(buf, LEN_DOORLOCK_LOCK-1);

  AT_single_bus_send_buf(buf, LEN_DOORLOCK_LOCK);

  osal_mem_free( buf );

  return ZSuccess;
}
ZStatus_t AT_DoorLock_SetTemporaryPin_Req( zclDoorLockSetTemporaryPin_t *pCmd )
{
  uint8 *buf = (uint8 *)osal_mem_alloc( LEN_DOORLOCK_SET_TEMPORARY_PIN );
  if(buf == NULL)
    return ZMemError;
  else
    osal_memset(buf, 0x00, LEN_DOORLOCK_SET_TEMPORARY_PIN);

  buf[0] = CMD_DOORLOCK_SET_TEMPORARY_PIN;

  if(pCmd->adminPin[0] == PIN_LEN_NULL)
  {
    buf[1] |=  CHECK_PIN_Y_N;   // do not check admin pin
  }
  else if((pCmd->adminPin[0] >= PIN_LEN_MIN) && (pCmd->adminPin[0] <= PIN_LEN_MAX))
  {
    buf[1] &= ~CHECK_PIN_Y_N;   // check admin pin

    osal_memcpy(&buf[10], &(pCmd->adminPin[1]), pCmd->adminPin[0]);
  }
  else
  {
    osal_mem_free( buf );
    return ZFailure;
  }

  osal_memcpy(&buf[22], &(pCmd->tempPin[1]), pCmd->tempPin[0]);

  buf[34] = pCmd->activeTimes;

  buf[35] = BREAK_UINT32( pCmd->activeTime, 3 );
  buf[36] = BREAK_UINT32( pCmd->activeTime, 2 );
  buf[37] = BREAK_UINT32( pCmd->activeTime, 1 );
  buf[38] = BREAK_UINT32( pCmd->activeTime, 0 );

  buf[LEN_DOORLOCK_SET_TEMPORARY_PIN-1] = sum_check(buf, LEN_DOORLOCK_SET_TEMPORARY_PIN-1);

  AT_single_bus_send_buf(buf, LEN_DOORLOCK_SET_TEMPORARY_PIN);

  osal_mem_free( buf );

  return ZSuccess;
}

/******************************************************************************
                                   uplink
******************************************************************************/
static ZStatus_t AT_DoorLock_Rsp(uint8 *buf, uint8 len)
{
  ZStatus_t result = ZFailure;

  afAddrType_t dstAddr;
  dstAddr.addr.shortAddr = NWK_PAN_COORD_ADDR;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.endPoint = 0x0B;

  uint8 cmd = 0xFF;
  uint8 status = ZCL_STATUS_FAILURE;

  switch(buf[0])
  {
    case CMD_DOORLOCK_UNLOCK:
      cmd = COMMAND_CLOSURES_UNLOCK_DOOR_RSP;
      if(buf[1] == UNLOCK_SUCCESS)
        status = ZCL_STATUS_SUCCESS;
      else
        status = ZCL_STATUS_FAILURE;
      break;

    case CMD_DOORLOCK_LOCK:
      cmd = COMMAND_CLOSURES_LOCK_DOOR_RSP;
      status = ZCL_STATUS_SUCCESS;
      break;

    case CMD_DOORLOCK_SET_TEMPORARY_PIN:
      cmd = COMMAND_CLOSURES_SET_TEMPORARY_PIN_RSP;
      switch(buf[1])
      {
        case SET_TEMPORARY_PIN_SUCCESS:
          status = ZCL_STATUS_SUCCESS;
          break;

        case SET_TEMPORARY_PIN_FAILED:
          status = ZCL_STATUS_FAILURE;
          break;

        case SET_TEMPORARY_PIN_SAME_PIN:
          status = ZCL_STATUS_SAME_PIN;
          break;

        default:
          status = ZCL_STATUS_FAILURE;
          break;
      }
      break;

    default:
      break;
  }

  result = zclClosures_SendDoorLockStatusResponse( DOORLOCK_ENDPOINT,
                                                   &dstAddr,
                                                   cmd,
                                                   status,
                                                   TRUE,
                                                   bdb_getZCLFrameCounter() );

  printf("AF, AT_DoorLock_Rsp: 0x%02X\r\n", result);

  return result;
}
static ZStatus_t AT_DoorLock_Upload(uint8 *buf, uint8 len)
{
  ZStatus_t result = ZFailure;

  uint8 flag = EVENT_FLAG_UNKNOWN;

  uint16 userID = 0xFFFF;
  uint32 zigBeeLocalTime = 0xFFFFFFFF;
  uint8 batteryLevel = 0xFF;

  uint8 operationEventSource = OPERATION_EVENT_SOURCE_INDETERMINATE;
  uint8 operationEventCode = OPERATION_EVENT_CODE_UNKNOWN_OR_MFG_SPECIFIC;

  uint8 programEventSource = OPERATION_EVENT_SOURCE_INDETERMINATE;
  uint8 programEventCode = PROGRAMMING_EVENT_CODE_UNKNOWN_OR_MFG_SPECIFIC;

  switch(buf[0])
  {
    case CMD_DOORLOCK_WAKE_UP:
      flag = EVENT_FLAG_OPERATION;
      operationEventCode = OPERATION_EVENT_CODE_WAKE_UP;
      NLME_SetPollRate(DOORLOCK_POLL_RATE_ACTIVE);
      osal_start_timerEx(zclDoorLock_TaskID, DOORLOCK_SET_DORMANT_EVT, DOORLOCK_KEEP_ACTIVE_TIME_WAKEUP);
      break;

    case CMD_DOORLOCK_EVT_UPLOAD:
      switch(buf[13])
      {
        case EVT_DOORLOCK_UPLOAD_ADD_USER:
          flag = EVENT_FLAG_PROGRAMMING;
          userID = (buf[15]<<8) | buf[16];
          batteryLevel = buf[10];
          programEventSource = OPERATION_EVENT_SOURCE_KEYPAD;
          programEventCode = AT_DoorLock_GetProgramEventCode_Add_User( buf[14] );
          break;

        case EVT_DOORLOCK_UPLOAD_DELETE_USER:
          flag = EVENT_FLAG_PROGRAMMING;
          userID = (buf[15]<<8) | buf[16];
          batteryLevel = buf[10];
          programEventSource = OPERATION_EVENT_SOURCE_KEYPAD;
          programEventCode = AT_DoorLock_GetProgramEventCode_Delete_User( buf[14] );
          break;

        case EVT_DOORLOCK_UPLOAD_UNLOCK:
          flag = EVENT_FLAG_OPERATION;
          userID = (buf[15]<<8) | buf[16];
          zigBeeLocalTime = UTCTime_To_SEC( &buf[17] );
          batteryLevel = buf[10];
          operationEventSource = AT_DoorLock_GetOperationEventSource( buf[14] );
          operationEventCode = OPERATION_EVENT_CODE_UNLOCK;
          osal_set_event(zclDoorLock_TaskID, DOORLOCK_REPORT_UNLOCK_EVT);
          break;

        case EVT_DOORLOCK_UPLOAD_FORCE_PRY_LOCK:
          flag = EVENT_FLAG_OPERATION;
          batteryLevel = buf[10];
          operationEventCode = OPERATION_EVENT_CODE_FORCE_PRY_LOCK;
          break;

        case EVT_DOORLOCK_UPLOAD_ENTER_LOCKED_STATE:
          flag = EVENT_FLAG_OPERATION;
          batteryLevel = buf[10];
          operationEventCode = OPERATION_EVENT_CODE_ENTER_LOCKED_STATE;
          break;

        case EVT_DOORLOCK_UPLOAD_LOW_POWER:
          flag = EVENT_FLAG_OPERATION;
          batteryLevel = buf[10];
          operationEventCode = OPERATION_EVENT_CODE_LOW_POWER;
          break;

        case EVT_DOORLOCK_UPLOAD_DOORBELL:
          flag = EVENT_FLAG_OPERATION;
          batteryLevel = buf[10];
          operationEventCode = OPERATION_EVENT_CODE_DOORBELL;
          NLME_SetPollRate(DOORLOCK_POLL_RATE_ACTIVE);
          osal_start_timerEx(zclDoorLock_TaskID, DOORLOCK_SET_DORMANT_EVT, DOORLOCK_KEEP_ACTIVE_TIME_DOORBELL);
          break;

        case EVT_DOORLOCK_UPLOAD_EXIT_LOCKED_STATE:
          flag = EVENT_FLAG_OPERATION;
          batteryLevel = buf[10];
          operationEventCode = OPERATION_EVENT_CODE_EXIT_LOCKED_STATE;
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }

  if(flag == EVENT_FLAG_OPERATION)
  {
    result = AT_DoorLock_SendOperationEventNotification( userID,
                                                         zigBeeLocalTime,
                                                         batteryLevel,
                                                         operationEventSource,
                                                         operationEventCode );

    printf("AF, EVENT_FLAG_OPERATION: 0x%02X\r\n", result);
  }
  else if(flag == EVENT_FLAG_PROGRAMMING)
  {
    result = AT_DoorLock_SendProgrammingEventNotification( userID,
                                                           zigBeeLocalTime,
                                                           batteryLevel,
                                                           programEventSource,
                                                           programEventCode );

    printf("AF, EVENT_FLAG_PROGRAMMING: 0x%02X\r\n", result);
  }
  else
  {
    result = ZFailure;
    printf("AF, EVENT_FLAG_UNKNOWN: 0x%02X\r\n", result);
  }

  if(result == ZNwkInvalidRequest)
  {
    osal_stop_timerEx(zclATApp_TaskID, ATAPP_END_DEVICE_REJOIN_EVT);
    osal_set_event(zclATApp_TaskID, ATAPP_END_DEVICE_REJOIN_EVT);
  }

  return result;
}
static ZStatus_t AT_DoorLock_Ack(uint8 *buf, uint8 len)
{
  uint8 ackBuf[2] = {0xE0, 0x1F};

  AT_single_bus_send_buf(ackBuf, 2);

  return ZSuccess;
}
ZStatus_t AT_DoorLock_Handle_Rsp(uint8 *buf, uint8 len)
{
  ZStatus_t status = ZSuccess;
  uint8 i;

  if(sum_check(buf, len-1) == buf[len-1])
  {
    printf("| up |right|%02d bytes|: ", len);
    for(i=0; i<len; i++)
    {
      printf("%02X ", buf[i]);
    }
    printf("\r\n");
  }
  else
  {
    printf("\r\n| up |error|%02d bytes|: ", len);
    for(i=0; i<len; i++)
    {
      printf("%02X ", buf[i]);
    }
    printf("\r\n\r\n");
    status = ZFailure;
  }

  if(status == ZFailure)
    return status;

  switch(buf[0])
  {
    case CMD_DOORLOCK_UNLOCK:
    case CMD_DOORLOCK_LOCK:
    case CMD_DOORLOCK_SET_TEMPORARY_PIN:
      status = AT_DoorLock_Rsp( buf, len );
      break;

    case CMD_DOORLOCK_EVT_UPLOAD:
    case CMD_DOORLOCK_WAKE_UP:
      status = AT_DoorLock_Upload( buf, len );
      break;

    case CMD_DOORLOCK_KEY_SHORT_PRESS:
      bdb_resetLocalAction(); // factory new
      break;

    case CMD_DOORLOCK_RESET_TO_FACT_NEW:
      break;

    case CMD_DOORLOCK_MATCH:
      status = AT_DoorLock_Ack( buf, len );
      break;

    case CMD_DOORLOCK_RESET_TO_FACT_NEW_ALL:
      if((len==4) && (buf[3]==0x1F))
        bdb_resetLocalAction();//factory new
      break;

    default:
      break;
  }

  return status;
}
