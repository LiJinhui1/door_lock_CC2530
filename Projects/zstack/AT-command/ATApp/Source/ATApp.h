/**************************************************************************************************
  Filename:       ATApp.h

  Description:    This file contains the ZigBee Cluster Library Home
                  Automation Sample Application.


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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef ZCL_ATAPP_H
#define ZCL_ATAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"


// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "zcl_general.h"
  #include "bdb_tlCommissioning.h"
#endif

/*********************************************************************
 * CONSTANTS
 */
#define ATApp_ENDPOINT            150
// Added to include ZLL Target functionality
#define ATAPP_NUM_GRPS            2

// User defined profile
#define ATApp_PROFILE_ID                0xF011
#define ATApp_DEVICEID_REMOTE_CONTROL   0x0001

// user defiened Cluster
#define ATApp_GENERIC_CLUSTER          0x2000
#define RESET_FACTORY_DEFAULT_CLUSTER  0x2001

// Application Events
#define ATAPP_END_DEVICE_REJOIN_EVT    0x0001

/* ATAPP_TODO: define app events here */
#define AT_RESET_EVENT                 0x0010
#define AT_NWKUPDATE_EVENT             0x0020
#define AT_RESTORE_CMDDO_FLAG          0x0040
#define AT_BDB_STEERING_EVT            0x0080

#define ATAPP_END_DEVICE_JOIN_DELAY    10000
#define ATAPP_END_DEVICE_JOIN_TIMES    6

#define ATAPP_END_DEVICE_REJOIN_DELAY_1  60000
#define ATAPP_END_DEVICE_REJOIN_DELAY_2  600000
#define ATAPP_END_DEVICE_REJOIN_DELAY_3  3600000

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  extern bdbTLDeviceInfo_t tlATApp_DeviceInfo;
#endif

extern SimpleDescriptionFormat_t ATApp_SimpleDesc;

extern byte zclATApp_TaskID;
extern uint8 gChannel;
extern uint8 gNwkUpdateId;


// extern CONST zclCommandRec_t zclATApp_Cmds[];
//
// extern CONST uint8 zclCmdsArraySize;
//
// // attribute list
// extern CONST zclAttrRec_t zclATApp_Attrs[];
// extern CONST uint8 zclATApp_NumAttributes;
//
// // Identify attributes
// extern uint16 zclATApp_IdentifyTime;
// extern uint8  zclATApp_IdentifyCommissionState;
//
// // ATAPP_TODO: Declare application specific attributes here


/*********************************************************************
 * FUNCTIONS
 */
/*
* Initialization for the task
*/
extern void zclATApp_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclATApp_event_loop( byte task_id, UINT16 events );

// /*
//  *  Reset all writable attributes to their default values.
//  */
// extern void zclATApp_ResetAttributesToDefaultValues(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_ATAPP_H */
