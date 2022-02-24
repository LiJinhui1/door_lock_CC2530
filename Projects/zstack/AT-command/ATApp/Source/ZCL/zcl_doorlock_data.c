/**************************************************************************************************
  Filename:       zcl_doorlock_data.c
  Author:         Yang Wang

  Revised:        $Date: 2014-05-12 13:14:02 -0700 (Mon, 12 May 2014) $
  Revision:       $Revision: 38502 $

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
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_closures.h"
#include "zcl_ha.h"

#include "zcl_doorlock.h"

/*********************************************************************
 * CONSTANTS
 */

#define DOORLOCK_DEVICE_VERSION     0
#define DOORLOCK_FLAGS              0

#define DOORLOCK_ZCLVERSION         2
#define DOORLOCK_APPVERSION         2
#define DOORLOCK_STACKVERSION       1
#define DOORLOCK_HWVERSION          1

#define MUFC_LENTH        64
#define MODE_LENTH        64
#define DATE_LENTH        32
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Global attributes
const uint16 zclDoorLock_clusterRevision_all = 0x0001;

// Basic Cluster
#ifdef ZCL_BASIC
const uint8 zclDoorLock_ZCLVersion = DOORLOCK_ZCLVERSION;
const uint8 zclDoorLock_APPVersion = DOORLOCK_APPVERSION;
const uint8 zclDoorLock_StackVersion = DOORLOCK_STACKVERSION;
const uint8 zclDoorLock_HWRevision[] = { 9, 'D','L','O','C','K','0','4','A','4' };
const uint8 zclDoorLock_ManufacturerName[] = { 9, 'B','a','i','y','a','T','e','c','h' };
const uint8 zclDoorLock_ModelId[] = { 13, 'Z', 'N', 'M', 'S', '-', 'T', 'I', '-', 'Z', 'B', '-', '1', 'A' };
const uint8 zclDoorLock_DateCode[] = { 8, '2','0','2','1','0','9','2','7' };
const uint8 zclDoorLock_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
const uint8 zclDoorLock_SWBuildID[] = {14, '1', '.', '4', '.', '0', '.', '2', '0', '2', '1', '0', '9', '2', '7'};

//__root const CODE uint8 zclDoorLock_ManufacturerName_Flash[MUFC_LENTH] @ 0x7C000 = { 9, 'B','a','i','y','a','T','e','c','h' };
//__root const CODE uint8 zclDoorLock_ModelId_Flash[MODE_LENTH] @ 0x7C040 = { 13,'B','Y','-','Z','N','M','S','-','Z','B','1','-','A'};
//__root const CODE uint8 zclDoorLock_DateCode_Flash[DATE_LENTH]  @ 0x7C080 = { 16, '2','0','1','9','0','4','1','9',' ',' ',' ',' ',' ',' ',' ',' '};
//__root const CODE uint8 zclDoorLock_HWRevision_Flash @ 0x7C0A0 = DOORLOCK_HWVERSION;
#endif

// Identify Cluster
uint16 zclDoorLock_IdentifyTime;

// Doorlock Cluster
uint8 zclDoorLock_LockState; 
uint8 zclDoorLock_LockType = CLOSURES_LOCK_TYPE_DEADBOLT; 
bool zclDoorLock_ActuatorEnabled = TRUE; 
bool zclDoorLock_SendPinOta = FALSE; 
bool zclDoorLock_RequirePinForRfOperation = TRUE; 

uint16 zclDoorLock_KeypadOperationEventMask;
uint16 zclDoorLock_RFOperationEventMask;
uint16 zclDoorLock_ManualOperationEventMask;
uint16 zclDoorLock_RFIDOperationEventMask;

#if ZCL_DISCOVER
CONST zclCommandRec_t zclDoorLock_Cmds[] =
{
#ifdef ZCL_BASIC
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  COMMAND_BASIC_RESET_FACT_DEFAULT,
  CMD_DIR_SERVER_RECEIVED
}
#endif

#ifdef ZCL_IDENTIFY
  ,{
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    COMMAND_IDENTIFY,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    COMMAND_IDENTIFY_QUERY,
    CMD_DIR_SERVER_RECEIVED
  }
#endif // ZCL_IDENTIFY
};

CONST uint8 zclCmdsArraySize = ( sizeof(zclDoorLock_Cmds) / sizeof(zclDoorLock_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclDoorLock_Attrs[] =
{
#ifdef ZCL_BASIC
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,   
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,       
      ZCL_DATATYPE_UINT8,               
      ACCESS_CONTROL_READ,             
      (void *)&zclDoorLock_ZCLVersion 
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {  // Attribute record
      ATTRID_BASIC_APP_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_APPVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_STACK_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_StackVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_HWRevision
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclDoorLock_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclDoorLock_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclDoorLock_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_SW_BUILD_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclDoorLock_SWBuildID
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclDoorLock_clusterRevision_all
    }
  },
#endif

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclDoorLock_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclDoorLock_clusterRevision_all
    }
  },
#endif // ZCL_IDENTIFY

#ifdef ZCL_DOORLOCK
  // *** Door Lock Cluster Attributes ***
  //Basic Information Attribute Set
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_CLOSURES_LOCK_STATE,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_REPORTABLE),
      (void *)&zclDoorLock_LockState
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_CLOSURES_LOCK_TYPE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_LockType
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_CLOSURES_ACTUATOR_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_ActuatorEnabled
    }
  },
  //User, PIN, Schedule, Log Information Attribute Set
  //Operational Settings Attribute Set
  //Security Settings Attribute Set
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_SEND_PIN_OTA,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,                
      (void *)&zclDoorLock_SendPinOta
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_REQUIRE_PIN_FOR_RF_OPERATION,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,                   
      (void *)&zclDoorLock_RequirePinForRfOperation
    }
  },
  //Alarm and Event Masks Attribute Set
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_KEYPAD_OPERATION_EVENT_MASK,
      ZCL_DATATYPE_BITMAP16,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_KeypadOperationEventMask
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_RF_OPERATION_EVENT_MASK,
      ZCL_DATATYPE_BITMAP16,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_RFOperationEventMask
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_MANUAL_OPERATION_EVENT_MASK,
      ZCL_DATATYPE_BITMAP16,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_ManualOperationEventMask
    }
  },
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    { // Attribute record
      ATTRID_DOORLOCK_RFID_OPERATION_EVENT_MASK,
      ZCL_DATATYPE_BITMAP16,
      ACCESS_CONTROL_READ,
      (void *)&zclDoorLock_RFIDOperationEventMask
    }
  },
  //Global Attributes
  {
    ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclDoorLock_clusterRevision_all
    }
  }
#endif // ZCL_DOORLOCK
};

uint8 CONST zclDoorLock_NumAttributes = ( sizeof(zclDoorLock_Attrs) / sizeof(zclDoorLock_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t DoorLock_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK
};
#define DOORLOCK_MAX_INCLUSTERS   (sizeof(DoorLock_InClusterList) / sizeof(DoorLock_InClusterList[0]))

SimpleDescriptionFormat_t DoorLock_SimpleDesc =
{
  DOORLOCK_ENDPOINT,                      //  int Endpoint;
  ZCL_HA_PROFILE_ID,                      //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_HOME_GATEWAY,           //  uint16 AppDeviceId[2];
  DOORLOCK_DEVICE_VERSION,                //  int   AppDevVer:4;
  DOORLOCK_FLAGS,                         //  int   AppFlags:4;
  DOORLOCK_MAX_INCLUSTERS,                //  byte  AppNumInClusters;
  (cId_t *)DoorLock_InClusterList,        //  byte *pAppInClusterList;
  0,                                      //  byte  AppNumInClusters;
  NULL                                    //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void zclDoorLock_ResetAttributesToDefaultValues(void)
{
#ifdef ZCL_IDENTIFY
  zclDoorLock_IdentifyTime = 0;
#endif

  zclDoorLock_LockState = CLOSURES_LOCK_STATE_LOCKED;

  zclDoorLock_KeypadOperationEventMask = 0x0004;//0b 0000 0000 0000 0100
  zclDoorLock_RFOperationEventMask     = 0x0004;//0b 0000 0000 0000 0100
  zclDoorLock_ManualOperationEventMask = 0x0004;//0b 0000 0000 0000 0100
  zclDoorLock_RFIDOperationEventMask   = 0x0004;//0b 0000 0000 0000 0100
}

/****************************************************************************
****************************************************************************/
