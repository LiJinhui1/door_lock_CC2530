/**************************************************************************************************
  Filename:       AT_uart.c

  Description:    AT command module
  Author:         Xiao Wang, Yasin Zhang
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "AF.h"
#include "ZDNwkMgr.h"
#include "ZDObject.h"
#include "AddrMgr.h"
#include "ZDSecMgr.h"
#include "osal_Clock.h"
#include "mac_pib.h"
#include "mac_radio_defs.h"

#if defined (INTER_PAN)
#include "InterPAN.h"
#include "stub_aps.h"
#endif

#include "bdb.h"
#include "bdb_interface.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_lighting.h"
#include "zcl_closures.h"

#include "ATApp.h"
#include "AT_controller.h"
#include "AT_uart.h"
#include "AT_printf.h"
#include "AT_cmd.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define AT_HEAD_STATE1    0x00
#define AT_HEAD_STATE2    0x01
#define AT_DATA_STATE     0x02
#define AT_END_STATE      0x03
#define AT_FCS_STATE      0x04

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool notdoFNCmd = TRUE;
bool notdoJNCmd = TRUE;
bool notdoSCANCmd = TRUE;
bool notdoEUIREQcmd = TRUE;
bool notdoSIMPLEDESCcmd = TRUE;
bool notdoNODEDESEcmd = TRUE;

uint8 keySeqNum = 1;

const AT_Cmd_t AT_Cmd_Arr[] = {
  {"BLOAD",     AT_Cmd_BLoad,     "Enter The Boot Loader Menu"},
  {"TIME",      AT_Cmd_Time,      "Get or Set Local Time"},
  {"RAWZCL",    AT_Cmd_RawZCL,    "Send A Raw ZCL Message With Specified ProfileID And Source Endpoint"},
  {"RAWZDO",    AT_Cmd_RawZDO,    "Construct A Raw ZDO Message And Send To Target"},
  {"INTERPAN",  AT_Cmd_InterPAN,  "Send an Interpan Command"},
  {"SCAN",      AT_Cmd_Scan,      "Scan The Energy Of All Channels or Scan For Active PANs"},

  #if ( ZG_BUILD_RTR_TYPE )
  // on FFD device
  {"FN",        AT_Cmd_FN,        "Establish Network"},
  #endif
  #if (ZG_BUILD_JOINING_TYPE)
  // on Join type deviece
  {"JN",        AT_Cmd_JN,        "Join Network"},
  #endif
  {"LN",        AT_Cmd_LN,        "Make Local Device Leave PAN"},
  {"N",         AT_Cmd_N,         "Display Network Information"},
  #if ( ZG_BUILD_RTR_TYPE )
  // Coordinator or router
  {"PJ",        AT_Cmd_PJ,        "Permit joining"},
  #endif
  #if (ZG_DEVICE_RTRONLY_TYPE|ZG_BUILD_ENDDEVICE_TYPE)
  // Not Coordinator
  {"RJ",        AT_Cmd_RJ,        "Rejoin the network"},
  #endif

  {"KEYTAB",    AT_Cmd_KeyTab,    "Print Local Key And Link Key Table"},
  #if ( ZG_BUILD_RTR_TYPE )
  // Coordinator or router
  {"ADDKEY",    AT_Cmd_AddKey,    "Add a key entry to local key table"},
  #endif

  {"IDREQ",     AT_Cmd_IDREQ,     "Request Node's NodeID"},
  {"EUIREQ",    AT_Cmd_EUIREQ,    "Request Node's EUI64"},
  {"NODEDESC",  AT_Cmd_NODEDESC,  "Request Node's Descriptor"},
  {"POWERDESC", AT_Cmd_POWERDESC, "Request Node's Power Descriptor"},
  {"ACTEPDESC", AT_Cmd_ACTEPDESC, "Request Node's Active EndPoint List"},
  {"SIMPLEDESC",AT_Cmd_SIMPLEDESC,"Request Node's Simple Descriptor"},
  {"MATCHREQ",  AT_Cmd_MATCHREQ,  "Find Nodes which Match a Specific Descriptor"},

  {"ANNCE",     AT_Cmd_ANNCE,     "Anounce Local Device In The Network"},
  #if ( ZG_BUILD_RTR_TYPE )
  // FFD Devices
  {"PANNCE",    AT_Cmd_PANNCE,    "Send A Parent Announce In The Network"},
  // FFD,COO Devices
  {"NTABLE",    AT_Cmd_NTABLE,    "Display Neighbour Table"},
  // FFD,COO Devices
  {"RTABLE",    AT_Cmd_RTABLE,    "Display Routing Table"},
  #endif

  #if defined ( REFLECTOR )
  #if defined ( ZDO_MGMT_BIND_RESPONSE )
  {"BTABLE",    AT_Cmd_BTABLE,    "Display Binding Table"},
  #endif // ZDO_MGMT_BIND_RESPONSE
  {"BSET",      AT_Cmd_BSET,      "Set local Binding Table Entry"},
  {"BCLR",      AT_Cmd_BCLR,      "Clear local Binding Table Entry"},
  {"BIND",      AT_Cmd_BIND,      "Create Binding on Remote Device"},
  {"UNBIND",    AT_Cmd_UNBIND,    "Delete Binding on Remote Device"},
  {"EBIND",     AT_Cmd_EBIND,     "End Device Bind"},
  #endif // REFLECTOR

  {"DASSR",     AT_Cmd_DASSR,     "Disassociate Remote Node from PAN"},
  #if ( ZG_BUILD_COORDINATOR_TYPE )
  {"KEYUPD",    AT_Cmd_KEYUPD,    "Update the Network Key"},
  {"CCHANGE",   AT_Cmd_CCHANGE,   "Change the network's channel"},
  #endif
  {"RADIOCH",   AT_Cmd_RADIOCH,   "Set or get local radio channel"},
  {"ATABLE",    AT_Cmd_ATABLE,    "Display Address Table"},
  {"ASET",      AT_Cmd_ASET,      "Set Address Table Entry"},

  #ifdef ZCL_DISCOVER
  {"ATRDISC",   AT_Cmd_ATRDISC,   "Find Supported Defined Attributes On A Remote Device"},
  #endif
  #ifdef ZCL_REPORT_CONFIGURING_DEVICE
  {"READRCFG",  AT_Cmd_READRCFG,  "Read Reporting Configuration From Remote Device"},
  {"CFGRPT",    AT_Cmd_CFGRPT,    "Configure Attribute Reporting"},
  #endif
  #ifdef ZCL_READ
  {"READATTR",  AT_Cmd_ReadAttr,  "read the attribute data by attrID"},       // AT+READATTR:<Addr>,<EP>,<SendMode>,<ClusterID>,<AttrID>,...,<AttrID>"},
  #endif
  #ifdef ZCL_WRITE
  {"WRITEATTR", AT_Cmd_WriteAttr, "write the attribute data by attrID"},      // AT+WRITEATTR:<Addr>,<EP>,<SendMode>,<ClusterID>,<AttrID>,,<DataType>,<Data>"}
  #endif

  {"HELP",      AT_Cmd_Help,      "show all the AT commands"},
  {"GETADDR",   AT_Cmd_GetAddr,   "show self short address"},
  {"EPCTRL",    AT_Cmd_EpCtrl,    "control the local endpoint"},              // AT+EPCTRL:<0/1>,<EP>"},
  {"REPCTRL",   AT_Cmd_REpCtrl,   "control the remote endpoint"},             // AT+REPCTRL:<shortAddr>,<0/1>,<EP>"},
  {"TEST",      AT_Cmd_Test,      "Debug test"}
};

const uint16 AT_CMD_SZ = sizeof(AT_Cmd_Arr) / sizeof(AT_Cmd_Arr[0]);

#if AT_SHOW_STATE_CHANGE
const uint8* devStates_str[]=
{
  "DEV_HOLD",                                // Initialized - not started automatically
  "DEV_INIT",                                // Initialized - not connected to anything
  "DEV_NWK_DISC",                            // Discovering PAN's to join
  "DEV_NWK_JOINING",                         // Joining a PAN
  "DEV_NWK_SEC_REJOIN_CURR_CHANNEL",         // ReJoining a PAN in secure mode scanning in current channel, only for end devices
  "DEV_END_DEVICE_UNAUTH",                   // Joined but not yet authenticated by trust center
  "DEV_END_DEVICE",                          // Started as device after authentication
  "DEV_ROUTER",                              // Device joined, authenticated and is a router
  "DEV_COORD_STARTING",                      // Started as Zigbee Coordinator
  "DEV_ZB_COORD",                            // Started as Zigbee Coordinator
  "DEV_NWK_ORPHAN",                          // Device has lost information about its parent..
  "DEV_NWK_KA",                              // Device is sending KeepAlive message to its parent
  "DEV_NWK_BACKOFF",                         // Device is waiting before trying to rejoin
  "DEV_NWK_SEC_REJOIN_ALL_CHANNEL",          // ReJoining a PAN in secure mode scanning in all channels, only for end devices
  "DEV_NWK_TC_REJOIN_CURR_CHANNEL",          // ReJoining a PAN in Trust center mode scanning in current channel, only for end devices
  "DEV_NWK_TC_REJOIN_ALL_CHANNEL"            // ReJoining a PAN in Trust center mode scanning in all channels, only for end devices
};
#endif

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern devStartModes_t devStartMode;
extern uint8 _tmpRejoinState;

extern uint8 EBindSeq;

typedef struct
{
  uint16            ami;
  uint16            keyNvId;   // index to the Link Key table in NV
  ZDSecMgr_Authentication_Option authenticateOption;
} ZDSecMgrEntry_t;
extern ZDSecMgrEntry_t* ZDSecMgrEntries;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
// the Energy scan call back funtion
extern void ZDNwkMgr_EDScanConfirmCB( NLME_EDScanConfirm_t *EDScanConfirm );

/*********************************************************************
 * LOCAL VARIABLES
 */
byte AT_UART_TaskID;   // Task ID for internal task/event processing
                        // This variable will be received when
                        // ATApp_Init() is called.
uint8 AT_RxBuffer[AT_CMD_BUFF_MAX];      // UART Rx buffer
uint8 at_state = AT_HEAD_STATE1;         // AT_command_Rx_state
uint8 AT_templen = 0;                    // the length of AT_command

char FWReversion[] = "AT-Command 1.0";

devStartModes_t startMode;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void  AT_UartProcess( uint8 port, uint8 event );
static uint8 getLength( uint8 *msg_ptr, uint8 fn );
static uint8 AT_get_next_cmdUnit( AT_CmdUnit* cmdUnit, uint8 start_point, uint8* msg );
static void  AT_UpperCaseCmd( AT_CmdUnit *cmdUnit );
static int8  AT_CmdCmp( AT_CmdUnit* cmdUnit, uint8* str2 );
static uint8 AT_ChartoInt( uint8 n );
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
static uint8* AT_AttrData_ChartoInt( uint8 dataType, AT_CmdUnit *cmdUnit );
#endif
#if AT_FCS_VERIFY
static byte AT_UartCalcFCS( uint8 *msg_ptr, uint8 len );
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
* @fn      AT_UART_Init
*
* @brief   Initialization function for the UART Task.
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  uint8 - the status of UART configuration
*/
uint8 AT_UART_Init( byte task_id )
{
  AT_UART_TaskID = task_id;

  halUARTCfg_t uartConfig;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = AT_UART_BR;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = MT_UART_THRESHOLD;
  uartConfig.rx.maxBufSize        = AT_UART_RX_BUFF_MAX;
  uartConfig.tx.maxBufSize        = AT_UART_TX_BUFF_MAX;
  uartConfig.idleTimeout          = MT_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = AT_UartProcess;

  AT_RxBuffer[0] = '\r';

  return HalUARTOpen(AT_UART_PORT, &uartConfig);
}

/***************************************************************************************************
 * @fn      AT_UartProcess
 *
 * @brief   | Head  |   Data   |  End  |  FCS   |
 *          |   2   |   0-Len  |   1   |   1    |
 *          |  AT   |     ?    | '\r'  | verify |
 *
 * @field processed by AT_HandleCMD()  : Data + End('\r')
 * @field verified  by AT_UartCalcFCS(): Data + End('\r')
 *
 *          Parses the data and send the data to correct place (AT or APP)
 *
 * @param   port     - UART port
 * @param   event    - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void AT_UartProcess ( uint8 port, uint8 event )
{
  uint8  ch;
  (void)event;  // Intentionally unreferenced parameter

  /*
   * due to the AT_UartProcess may be re-entried during AT_HandleCMD aused by HallUARTPollISR,
   * which is denergerous, As a result of some command ERROR, so we prevent the function from being re-entryed
   */
  static uint8 isProcessing = false;
  if(isProcessing) return;
  isProcessing = true;

  while (Hal_UART_RxBufLen(port)) {
    HalUARTRead (port, &ch, 1);

    switch (at_state) {
      case AT_HEAD_STATE1:
        if (ch == 'A' || ch == 'a')
          at_state = AT_HEAD_STATE2;
        //else still in AT_HEAD_STATE1;
        break;

      case AT_HEAD_STATE2:
        if (ch == 'T' || ch == 't')
          at_state = AT_DATA_STATE;
        else if (ch == 'A' || ch == 'a')
          at_state = AT_HEAD_STATE2;
        else
          at_state = AT_HEAD_STATE1;
        break;

      case AT_DATA_STATE:
#if AT_UART_BACKSPACE
        if(ch=='\b'||ch=='\x7f'){         // for backspace function, allow user to delete characters
          if(AT_templen>0)
            AT_templen--;
          break;
        }
#endif
        if (ch == '\r') {
          AT_RxBuffer[AT_templen++] = '\r';
#if AT_FCS_VERIFY
          at_state = AT_FCS_STATE;
          break;
#else
          AT_templen = 0;
          AT_HandleCMD(AT_RxBuffer);
          at_state = AT_HEAD_STATE1;
#endif
        } else {
          if (AT_templen < (AT_CMD_BUFF_MAX - 1)) {
            AT_RxBuffer[AT_templen++] = ch;
            // still in AT_DATA_STATE;
          } else {
            osal_memset(AT_RxBuffer, 0, AT_UART_RX_BUFF_MAX);
            AT_templen = 0;
            at_state = AT_HEAD_STATE1;
          }
        }

        break;

#if AT_FCS_VERIFY
      case AT_FCS_STATE:

        /* Make sure it's correct */
        if ((AT_UartCalcFCS (AT_RxBuffer, AT_templen) == ch))
        {
          AT_HandleCMD(AT_RxBuffer);
          osal_memset(AT_RxBuffer, 0, AT_templen);
          AT_templen = 0;
        }
        else
        {
          AT_ERROR(AT_FATAL_ERROR);  // SEND FCS ERROR MSG AT_FATAL_ERROR
        }

        /* Reset the state, send or discard the buffers at this point */
        at_state = AT_HEAD_STATE1;

        break;
#endif

      default:
       break;
    }
  }
  isProcessing = false;
}

/***************************************************************************************************
 * @fn      AT_HandleCMD
 *
 * @brief   Parse the AT_commands and call it's response function
 *
 * @param   byte *msg_ptr - message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_HandleCMD( uint8 *msg_ptr )
{
  uint8 cmd_ptr = 0;
  uint16 i;
  AT_DEBUG("\r\n", 2);
  AT_DEBUG(msg_ptr, getLength(msg_ptr, 0x02));

  // Get next cmdUnit and upper case it
  AT_CmdUnit cmdUnit;
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnit, cmd_ptr, msg_ptr);
  AT_UpperCaseCmd(&cmdUnit);

  // process the cmdUnit
  if (cmdUnit.symbol == '\r') {  // which means there is no followed operator or command
    AT_OK();
  }
  else if (cmdUnit.symbol == '\0') {
    if (AT_CmdCmp(&cmdUnit, "I") == 0) {
      AT_Cmd_ATI(cmd_ptr, msg_ptr);
    } else if (AT_CmdCmp(&cmdUnit, "Z") == 0) {
      AT_DEBUG("\r\nSoftware Reset\r\n", sizeof("\r\nSoftware Reset\r\n"));
      AT_Cmd_ATZ(cmd_ptr, msg_ptr);
    } else if (AT_CmdCmp(&cmdUnit, "F") == 0) {
      AT_DEBUG("\r\nRestore Local Device's Factory Defaults\r\n", sizeof("\r\nRestore Local Device's Factory Defaults\r\n"));
      AT_Cmd_ATF(cmd_ptr, msg_ptr);
    } else {
      AT_ERROR(AT_LACK_OPERATOR);
    }
  }
  else if (cmdUnit.symbol == '+') {
    for (i = 0; i < AT_CMD_SZ; i++) {
      if (AT_CmdCmp(&cmdUnit, (uint8*)AT_Cmd_Arr[i].AT_Cmd_str) == 0) {
#if AT_DEBUG_INF_SHOW
        AT_NEXT_LINE();
        AT_RESP(AT_Cmd_Arr[i].ATCmdDescription, strlen(AT_Cmd_Arr[i].ATCmdDescription));
#endif
        AT_Cmd_Arr[i].AT_CmdFn(cmd_ptr, msg_ptr);
        break;
      }
    }
    if (AT_CMD_SZ == i) {
      if (AT_CmdCmp(&cmdUnit, "") == 0) {
        AT_ERROR(AT_LACK_CMD);
      } else {
        AT_ERROR(AT_UNKNOWN_CMD);
      }
    }
  }
  else {
    AT_ERROR(AT_INVALID_PARA);
  }
}

#if AT_FCS_VERIFY
/***************************************************************************************************
 * @fn      AT_UartCalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include Head fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
byte AT_UartCalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult;

  xorResult = 0;

  for ( x = 0; x < len; x++, msg_ptr++ )
    xorResult = xorResult ^ *msg_ptr;

  return ( xorResult );
}
#endif

/******************************************************************************
* @fn      AT_HalUARTWrite
*
* @brief   Override HalUARTWrite function to write a buffer to the UART.
*          Note: CALL HalUARTPoll() TO GET A FULL TEXT DISPLAY.
*                AVOID TEST LOST.
*
* @param   port - UART port
* @param   buf  - pointer to the buffer that will be written, not freed
* @param   len  - length of the buffer
*
* @return  length of the buffer that was sent
*****************************************************************************/
uint16 AT_HalUARTWrite( uint8 port, uint8 *buf, uint16 len ) {
  uint16 cnt = 0;
  if(len > 0 && len < AT_UART_TX_BUFF_MAX) {   // if the len is not checked, the system will fail. if len==0, the system will loop here all the time.
    while ((cnt = HalUARTWrite(port, buf, len)) < len) {
      buf += cnt;
      len -= cnt;
      HalUARTPoll();//wait until the text is sent successfully
                    //when using Z-Stack 2.51a, we have to set the HAL_UART_ISR=1 and HAL_UART_DMA=0 compile flags to enable ISR mode
                    //maybe the bug of the Z-Stack, the DMA mode does work with this
                    //if using Z-Stack 2.3.0, both the DMA mode and ISR mode are OK
      //while(1);
    }
  }
  return cnt;
}

/***************************************************************************************************
 * @fn      getLength
 *
 * @brief   Get the length of a string (end by '\0')
 *                         or a command (end by '\r')
 *
 * @param   byte   *msg_ptr - message pointer
 * @param   uint8  fn - select different functions
 *                       0x01 - string    0x02 - command
 *
 * @return  length
 ***************************************************************************************************/
uint8 getLength( uint8 *msg_ptr, uint8 fn )
{
  uint8 i;
  if (fn == 0x01) {
    for (i = 0; i < 255; i++) {
      if (msg_ptr[i] == '\0') break;
    }
  } else {
    for (i = 0; i < 255; i++) {
      if (msg_ptr[i] == '\r') break;
    }
  }

  return i;
}

/***************************************************************************************************
 * @fn      AT_get_next_cmdUnit
 *
 * @brief   The command have servel unit, this funciton get the next unit
 *          and save it in cmdUnit(type of AT_CmdUnit, see in AT_UART.h)
 *
 * @param   AT_CmdUnit* cmdUnit - the place to save the cmd unit
 * @param   uint8 start_point   - the point show the place we start scan the command
 * @param   uint8* msg          - the command to scan
 *
 * @return  start_point
 ***************************************************************************************************/
uint8 AT_get_next_cmdUnit( AT_CmdUnit* cmdUnit, uint8 start_point, uint8* msg )
{
  cmdUnit->unitLen=0;
  for(;;start_point++){
    if(msg[start_point] == ' '|| msg[start_point] == '\0'){
      continue;
    }
    else if((msg[start_point]<='z' && msg[start_point]>='a') ||
       (msg[start_point]<='Z' && msg[start_point]>='A') ||
       (msg[start_point]<='9' && msg[start_point]>='0')){
       cmdUnit->symbol ='\0';                 //indicate no operator
       break;
    }
    else if(msg[start_point] == '\r'){        //indicate the end of one command
      cmdUnit->symbol =msg[start_point];
      return start_point;
    }
    else {
      cmdUnit->symbol =msg[start_point];
      start_point++;
      break;
    }
  }

  for(;;start_point++){
    if(msg[start_point] == ' '|| msg[start_point] == '\0'){
      continue;
    }
    else while((msg[start_point]<='z' && msg[start_point]>='a') ||
       (msg[start_point]<='Z' && msg[start_point]>='A') ||
       (msg[start_point]<='9' && msg[start_point]>='0') )   {
       if(cmdUnit->unitLen==0) cmdUnit->unit = &msg[start_point];
       cmdUnit->unitLen++;
       start_point++;
    }
    return start_point;
  }
}

/***************************************************************************************************
 * @fn      AT_UpperCaseCmd
 *
 * @brief   Upper case the command
 *
 * @param   AT_CmdUnit* cmdUnit - the cmd unit need to be upper cased
 *
 * @return  None
 ***************************************************************************************************/
void AT_UpperCaseCmd( AT_CmdUnit *cmdUnit )
{
  uint8 i;
  for (i = 0; i < cmdUnit->unitLen; i++) {
    if (cmdUnit->unit[i] <= 'z' && cmdUnit->unit[i] >= 'a') {
      cmdUnit->unit[i] += ('A' - 'a');
    }
  }
}

/***************************************************************************************************
 * @fn      AT_CmdCmp
 *
 * @brief   Compare with str2, if equal, return 0; else return cmdUnit.unit[i] - str2[i]
 *
 * @param   AT_CmdUnit* cmdUnit - the cmd unit need to be upper cased
 * @param   uint8* str2 - the string to compare with
 *
 * @return  int8
 ***************************************************************************************************/
int8 AT_CmdCmp( AT_CmdUnit* cmdUnit, uint8* str2 )
{
  int i;
  for(i = 0; i < cmdUnit->unitLen; i++) {
    if (cmdUnit->unit[i] != str2[i]) {
      return cmdUnit->unit[i] - str2[i];
    }
  }
  return 0 - str2[cmdUnit->unitLen];
}

/***************************************************************************************************
 * @fn      AT_UARTWriteErrMsg
 *
 * @brief   Send error messages
 *          Respose :
 *                    ERROR:<error code>
 *
 *
 * @param   uint8 errCode - indicate different error
 * @param   uint8 fn - type of error
 *
 * @return  None
 ***************************************************************************************************/
void AT_UARTWriteErrMsg( uint8 errCode, uint8 fn )
{
  if (fn == 1) {
    uint8* errMsg_t = "\r\nERROR:XX\r\n";
    uint8 errMsg[sizeof("\r\nERROR:XX\r\n")];
    uint8 ch;
    uint8* pStr = &errMsg[sizeof("\r\nERROR:")-1];
    for (int i = 0; i < sizeof(errMsg); i++) {
      errMsg[i] = errMsg_t[i];
    }
    ch = (errCode >> 4) & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    ch = errCode & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    AT_RESP(errMsg, sizeof("\r\nERROR:XX\r\n"));
  } else if (fn == 2) {
    uint8* errMsg_t = "\r\nSENDERROR:XX\r\n";
    uint8 errMsg[sizeof("\r\nSENDERROR:XX\r\n")];
    uint8 ch;
    uint8* pStr = &errMsg[sizeof("\r\nSENDERROR:")-1];
    for (int i = 0; i < sizeof(errMsg); i++) {
      errMsg[i] = errMsg_t[i];
    }
    ch = (errCode >> 4) & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    ch = errCode & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    AT_RESP(errMsg, sizeof("\r\nSENDERROR:XX\r\n"));
  } else if (fn == 3) {
    uint8* errMsg_t = "\r\nZDOERROR:XX\r\n";
    uint8 errMsg[sizeof("\r\nZDOERROR:XX\r\n")];
    uint8 ch;
    uint8* pStr = &errMsg[sizeof("\r\nZDOERROR:")-1];
    for (int i = 0; i < sizeof(errMsg); i++) {
      errMsg[i] = errMsg_t[i];
    }
    ch = (errCode >> 4) & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    ch = errCode & 0x0F;
    *pStr++ = ch + ((ch < 10) ? '0' : '7');
    AT_RESP(errMsg, sizeof("\r\nZDOERROR:XX\r\n"));
  }
}

/***************************************************************************************************
 * @fn      AT_GetIEEEAddrStr
 *
 * @brief   Tool Functions
 *          Get the IEEE address of the device
 *
 * @param   uint8* pIeeeAddr - the pointer of IEEE address
 * @param   uint8* pStr      - store the IEEE address
 *
 * @return  None
 ***************************************************************************************************/
void AT_GetIEEEAddrStr( uint8* pIeeeAddr, uint8* pStr )
{
  uint8 i;
  uint8 *xad = pIeeeAddr + Z_EXTADDR_LEN - 1;

  for (i = 0; i < 16; xad--)
  {
    uint8 ch;
    ch = (*xad >> 4) & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
    ch = *xad & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
  }
}

/***************************************************************************************************
 * @fn      AT_Int8toChar
 *
 * @brief   Tool Functions
 *          Convert a byte to a string
 *
 * @param   uint8  pbyte - the byte to be converted
 * @param   uint8* pStr  - store the new character
 *
 * @return  None
 ***************************************************************************************************/
void AT_Int8toChar( uint8 pbyte, uint8* str )
{
  str[1] = pbyte%16 < 10 ? pbyte%16+'0' : (pbyte%16-10)+'A';
  str[0] = pbyte/16 < 10 ? pbyte/16+'0' : (pbyte/16-10)+'A';
}

/***************************************************************************************************
 * @fn      AT_Int8toChar
 *
 * @brief   Tool Functions
 *          Convert a word to a string
 *
 * @param   uint8  pWord - the word to be converted
 * @param   uint8* pStr  - store the new character
 *
 * @return  None
 ***************************************************************************************************/
void AT_Int16toChar( uint16 pWord, uint8* str )
{
  uint16 ch = pWord >> 8;
  str[1] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[0] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
  ch = pWord & 0x00ff;
  str[3] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[2] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
}

/***************************************************************************************************
 * @fn      AT_Int32toChar
 *
 * @brief   Tool Functions
 *          Convert double words to a string
 *
 * @param   uint32 pDWord - the double words to be converted
 * @param   uint8* pStr   - store the new character
 *
 * @return  None
 ***************************************************************************************************/
void AT_Int32toChar( uint32 pDWord, uint8* str )
{
  uint16 ch = pDWord >> 24;
  str[1] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[0] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
  ch = (pDWord >> 16) & 0x00ff;
  str[3] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[2] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
  ch = (pDWord >> 8) & 0x0000ff;
  str[5] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[4] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
  ch = pDWord & 0x000000ff;
  str[7] = ch%16 < 10 ? ch%16+'0' : (ch%16-10)+'A';
  str[6] = ch/16 < 10 ? ch/16+'0' : (ch/16-10)+'A';
}

/***************************************************************************************************
 * @fn      AT_IntxtoChar
 *
 * @brief   Tool Function
 *
 * @param   data - a point to the data
 * @param   pStr - a point to the string
 * @param   x - the bit length of the data
 *
 * @return  None
 ***************************************************************************************************/
void AT_IntxtoChar( uint8* data, uint8 *pStr, uint8 x )
{
  uint8 len = x/8;
  uint8 i;
  uint8 k = len*2-1;
  for ( i = 0; i < len; i++) {
    pStr[i] = 0;
  }
  for ( i = 0; i < len; i++ ) {
    pStr[k--] = data[i]%16 < 10 ? data[i]%16+'0' : (data[i]%16-10)+'A';
    pStr[k--] = data[i]/16 < 10 ? data[i]/16+'0' : (data[i]/16-10)+'A';
  }
}

/***************************************************************************************************
 * @fn      AT_ChartoInt8
 *
 * @brief   Tool Functions
 *          Convert a character to integer
 *
 * @param   AT_CmdUnit *cmdUnit - a point to the AT_CmdUnit
 *
 * @return  None
 ***************************************************************************************************/
uint8 AT_ChartoInt( uint8 n )
{
  if (n >= '0' && n <= '9') return n-'0';
  if (n >= 'a' && n <= 'f') return n-'a'+10;
  if (n >= 'A' && n <= 'F') return n-'A'+10;
  return 0xff;
}

/***************************************************************************************************
 * @fn      AT_ChartoInt8
 *
 * @brief   Tool Functions
 *          Convert 1-2 character to a byte
 *
 * @param   AT_CmdUnit *cmdUnit - a point to the AT_CmdUnit
 *
 * @return  uint8
 ***************************************************************************************************/
uint8 AT_ChartoInt8( AT_CmdUnit *cmdUnit )
{
  uint8 result=0;
  if(cmdUnit->unitLen>0)
    result |= AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-1]);
  if(cmdUnit->unitLen>1)
    result |= AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-2])<<4;
  return result;
}

/***************************************************************************************************
 * @fn      AT_ChartoInt16
 *
 * @brief   Tool Functions
 *          Convert 1-4 character to a word
 *
 * @param   AT_CmdUnit *cmdUnit - a point to the AT_CmdUnit
 *
 * @return  uint8
 ***************************************************************************************************/
uint16 AT_ChartoInt16( AT_CmdUnit *cmdUnit )
{
  uint16 result=0;
  if(cmdUnit->unitLen>0)
    result |= (uint16) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-1]);
  if(cmdUnit->unitLen>1)
    result |= ((uint16) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-2]))<<4;
  if(cmdUnit->unitLen>2)
    result |= ((uint16) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-3]))<<8;
  if(cmdUnit->unitLen>3)
    result |= ((uint16) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-4]))<<12;
  return result;
}

/***************************************************************************************************
 * @fn      AT_ChartoInt32
 *
 * @brief   Tool Functions
 *          Convert 1-8 character to a double word
 *
 * @param   AT_CmdUnit *cmdUnit - a point to the AT_CmdUnit
 *
 * @return  uint32
 ***************************************************************************************************/
uint32 AT_ChartoInt32( AT_CmdUnit *cmdUnit )
{
  uint32 result=0;
  if(cmdUnit->unitLen>0)
    result |= (uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-1]);
  if(cmdUnit->unitLen>1)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-2]))<<4;
  if(cmdUnit->unitLen>2)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-3]))<<8;
  if(cmdUnit->unitLen>3)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-4]))<<12;
  if(cmdUnit->unitLen>4)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-5]))<<16;
  if(cmdUnit->unitLen>5)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-6]))<<20;
  if(cmdUnit->unitLen>6)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-7]))<<24;
  if(cmdUnit->unitLen>7)
    result |= ((uint32) AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-8]))<<28;
  return result;
}

/***************************************************************************************************
 * @fn      AT_ChartoIntx
 *
 * @brief   Tool Function
 *
 * @param   AT_CmdUnit *cmdUnit - a point to the AT_CmdUnit
 * @param   uint8 *pHex - a point to the Intx
 * @param   x - the bit length of the destination
 *
 * @return  uint32
 ***************************************************************************************************/
void AT_ChartoIntx( AT_CmdUnit *cmdUnit,uint8 *pHex, uint8 x )
{
  uint8 len = x/8;
  uint8 i;
  for ( i = 0; i < len; i++) {
    pHex[i] = 0;
  }
  for ( i = 0; i < len; i++ ) {
    if(cmdUnit->unitLen>2*i)
      pHex[i] |= AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-1-2*i]);
    else
      break;
    if(cmdUnit->unitLen>2*i+1)
      pHex[i] |= AT_ChartoInt(cmdUnit->unit[cmdUnit->unitLen-2-2*i])<<(1*4);
    else
      break;
  }
}

#ifdef ZCL_REPORT_CONFIGURING_DEVICE
/*********************************************************************
 * @fn      AT_AttrData_ChartoInt
 *
 * @brief   Tool Function. Convert different types of Attributes' Data
 *
 * @param   dataType - data types defined in zcl.h
 * @param   attrData - pointer to the attribute data
 *
 * @return  none
 *********************************************************************/
static uint8* AT_AttrData_ChartoInt( uint8 dataType, AT_CmdUnit *cmdUnit )
{
  uint8 *pStr = NULL;

  switch ( dataType )
  {
    case ZCL_DATATYPE_DATA8:
    case ZCL_DATATYPE_BOOLEAN:
    case ZCL_DATATYPE_BITMAP8:
    case ZCL_DATATYPE_INT8:
    case ZCL_DATATYPE_UINT8:
    case ZCL_DATATYPE_ENUM8:
      pStr = zcl_mem_alloc( 1 );
      AT_ChartoIntx(cmdUnit, pStr, 8);
      break;

    case ZCL_DATATYPE_DATA16:
    case ZCL_DATATYPE_BITMAP16:
    case ZCL_DATATYPE_UINT16:
    case ZCL_DATATYPE_INT16:
    case ZCL_DATATYPE_ENUM16:
    case ZCL_DATATYPE_SEMI_PREC:
    case ZCL_DATATYPE_CLUSTER_ID:
    case ZCL_DATATYPE_ATTR_ID:
      pStr = zcl_mem_alloc( 2 );
      AT_ChartoIntx(cmdUnit, pStr, 16);
      break;

    case ZCL_DATATYPE_DATA24:
    case ZCL_DATATYPE_BITMAP24:
    case ZCL_DATATYPE_UINT24:
    case ZCL_DATATYPE_INT24:
      pStr = zcl_mem_alloc( 3 );
      AT_ChartoIntx(cmdUnit, pStr, 24);
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
      pStr = zcl_mem_alloc( 4 );
      AT_ChartoIntx(cmdUnit, pStr, 32);
      break;

    case ZCL_DATATYPE_UINT40:
    case ZCL_DATATYPE_INT40:
      pStr = zcl_mem_alloc( 5 );
      AT_ChartoIntx(cmdUnit, pStr, 40);
      break;

    case ZCL_DATATYPE_UINT48:
    case ZCL_DATATYPE_INT48:
      pStr = zcl_mem_alloc( 6 );
      AT_ChartoIntx(cmdUnit, pStr, 48);
      break;

    case ZCL_DATATYPE_UINT56:
    case ZCL_DATATYPE_INT56:
      pStr = zcl_mem_alloc( 7 );
      AT_ChartoIntx(cmdUnit, pStr, 56);
      break;

    case ZCL_DATATYPE_DOUBLE_PREC:
    case ZCL_DATATYPE_IEEE_ADDR:
    case ZCL_DATATYPE_UINT64:
    case ZCL_DATATYPE_INT64:
      pStr = zcl_mem_alloc( 8 );
      AT_ChartoIntx(cmdUnit, pStr, 64);
      break;

    case ZCL_DATATYPE_CHAR_STR:
    case ZCL_DATATYPE_OCTET_STR:
      pStr = zcl_mem_alloc( 1+(cmdUnit->unitLen) );
      *pStr = cmdUnit->unitLen;
      zcl_memcpy( pStr+1, cmdUnit->unit, cmdUnit->unitLen );
      break;

    case ZCL_DATATYPE_LONG_CHAR_STR:
    case ZCL_DATATYPE_LONG_OCTET_STR:
      // cmdUnit don't support
      // pStr = zcl_mem_alloc( 1+(cmdUnit->unitLen) );
      // *pStr = cmdUnit->unitLen;
      // zcl_memcpy( pStr+1, cmdUnit->unit, cmdUnit->unitLen );
      break;

    case ZCL_DATATYPE_128_BIT_SEC_KEY:
      pStr = zcl_mem_alloc( SEC_KEY_LEN );
      AT_ChartoIntx(cmdUnit, pStr, 128);
      break;

    case ZCL_DATATYPE_NO_DATA:
    case ZCL_DATATYPE_UNKNOWN:
      // Fall through

    default:
      break;
  }

  return ( pStr );
}
#endif

#if AT_CMD_PATTERN_CHECK
static uint8 AT_Pattern_Check( char* pattern, AT_CmdUnit* cmdUnitArr );
/***************************************************************************************************
 * @fn      AT_Pattern_Check
 *
 * @brief   Check the command pattern
 *
 * @param   char* pattern  - the pattern to be checked
 * @param   AT_CmdUnit* cmdUnitArr - the command unit
 *
 * @return  uint8
 ***************************************************************************************************/
uint8 AT_Pattern_Check( char* pattern, AT_CmdUnit* cmdUnitArr )
{
  uint8 i=0;
  for(;pattern[i+1]!='\0';i++) {
    if(pattern[i] != cmdUnitArr[i].symbol){
      if(cmdUnitArr[i].symbol =='\0') return AT_LACK_OPERATOR;
      else if(cmdUnitArr[i].symbol =='\r') return AT_LACK_PARA;
      else return AT_INVALID_PARA;
    }
  }
  if(pattern[i]=='\r') {
    if(cmdUnitArr[i].symbol!='\r') return AT_INVALID_PARA;
  }
  else{
    if(pattern[i] != cmdUnitArr[i].symbol){
      if(cmdUnitArr[i].symbol =='\0') return AT_LACK_OPERATOR;
      else if(cmdUnitArr[i].symbol =='\r') return AT_LACK_PARA;
      else return AT_INVALID_PARA;
    }
  }
  return AT_NO_ERROR;
}
#endif

/***************************************************************************************************
 * @fn      AT_Cmd_ATI
 *
 * @brief   Display the product Identification Information
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_ATI( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  uint8 str[17];
  uint8 version[2];
  AT_GetIEEEAddrStr(NLME_GetExtAddr(), str);
  AT_Int8toChar(_NIB.nwkProtocolVersion, version);

  AT_NEW_LINE();
  //AT_RESP("AT-Command Project 1.0", sizeof("AT-Command Project 1.0"));
  //AT_NEXT_LINE();
#if ( ZG_BUILD_COORDINATOR_TYPE )
  AT_RESP("COORDINATOR", sizeof("COORDINATOR"));
#elif ( ZG_BUILD_RTR_TYPE )
  AT_RESP("ROUTER", sizeof("ROUTER"));
#elif ( ZG_BUILD_ENDDEVICE_TYPE )
  AT_RESP("ENDDEVICE", sizeof("ENDDEVICE"));
#else
  AT_RESP("<unknow error>", sizeof("<unknow error>"));
#endif
  AT_NEXT_LINE();
  printf(FWReversion);
  AT_NEXT_LINE();
  AT_RESP("EUI64:", sizeof("EUI64:"));
  AT_RESP(str, 16);
  AT_NEXT_LINE();
  AT_OK();
  AT_NEW_LINE();
}

/***************************************************************************************************
 * @fn      AT_Cmd_ATZ
 *
 * @brief   ATZ - Software Reset
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_ATZ( uint8 cmd_ptr, uint8* msg )
{
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);
  AT_OK();

  osal_start_timerEx( AT_UART_TaskID, AT_RESET_EVENT, 50 ); //set timer ensure OK response from AT command is sent
}

/***************************************************************************************************
 * @fn      AT_Cmd_ATF
 *
 * @brief   AT&F - Restore Local Device's Factory Defaults
 *           Note: Module performs a factory reset.
 *                 All non-volatile S Registers are updated with their factory
 *                 defaults and the node leaves the currently joined network
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_ATF( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;
  AT_CmdUnit cmdUnitArr[3];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);

  if(cmdUnitArr[0].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

    // AT_clear_AT_SYSTEM_NVs(); if Application use NV, we need to define this function
    bdb_resetLocalAction();
    AT_OK();
  } else {
    for (int i = 1; i < 3; i++) {
      cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    }
	  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

    uint16 addr = AT_ChartoInt16(&cmdUnitArr[0]);
    uint8  ep   = AT_ChartoInt8(&cmdUnitArr[1]);

    // build destination address
    afAddrType_t dstAddr;
    dstAddr.endPoint = ep;
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    dstAddr.addr.shortAddr = addr;

    status = zclGeneral_SendBasicResetFactoryDefaults( ZCL_CONTROLLER_ENDPOINT, &dstAddr,
                        TRUE, bdb_getZCLFrameCounter() );
    if(status != afStatus_SUCCESS) {
      AT_SEND_ERROR(status);
    }
    else AT_OK();
  }

}

/*******************************************************************************
 * @fn      AT_Cmd_BLoad
 *
 * @brief   AT+BLOAD - Enter The Boot Loader Menu
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_BLoad( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  osal_start_timerEx( AT_UART_TaskID, AT_RESET_EVENT, 100 ); //set timer ensure OK response from AT command is sent

  AT_OK();
  AT_NEW_LINE();
  AT_RESP("Enter BootLoader", sizeof("Enter BootLoader")-1);
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_Cmd_Time
 *
 * @brief   AT+Time - Get or Set Local Time
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_Time( uint8 cmd_ptr, uint8* msg_ptr )
{
  UTCTime time_c;
  uint8 str[8];
  AT_CmdUnit cmdUnitArr[2];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);

  AT_NEW_LINE();
  if(cmdUnitArr[0].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

    time_c = osal_getClock();
    AT_Int32toChar( time_c, str );
    AT_RESP("TIME:", 5);
    AT_RESP(str, 8);
  } else {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[1], cmd_ptr, msg_ptr);
	  AT_PARSE_CMD_PATTERN_ERROR(":\r",cmdUnitArr);

    time_c = AT_ChartoInt32(&cmdUnitArr[0]);
    osal_setClock(time_c);
    AT_RESP("TIME:", 5);
    AT_RESP(cmdUnitArr[0].unit, 8);
  }
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_RawZCL
 *
 * @brief   AT+RAWZCL - Send A Raw ZCL Message With Specified ProfileID
 *                      And Source Endpoint
 *          Format: AT+RAWZCL:<NodeID>,<DstEP>,[SrcEP],[ProfileID],
 *                             <ClusterID>,<data>
 *                  <NodeID> - 16 bit hexadecimal number, network address of
 *                             a remote device.
 *                  <DstEp>  - 8 bit hexadecimal number, destination endpoint
 *                             of a remote device.
 *                  [SrcEP]  - 8 bit hexadecimal number, source endpoint of
 *                             a local device. If it is omitted, source
 *                             endpoint is set to 0x64
 *                  [ProfileID] - 16 bit hexadecimal number which represents
 *                                profile ID. E.g. 0x0104 for ZigBee home
 *                                automation profile. If it is omitted, profile
 *                                ID is set to 0x0104
 *                  <ClusterID> - 16 bit hexadecimal number which represents
 *                                cluster ID
 *                  <data>   - a constructed ZCL command in hexadecimal format
 *                          (please check ZigBee Cluster Library for references)
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_RawZCL( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint16 dataLen;
  uint8 *buf;
  uint8 *pBuf;
  // uint16 tempdata;
  AT_CmdUnit cmdUnitArr[7];

  for ( i = 0; i < 7; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,,,,,\r",cmdUnitArr);

  uint16 addr = AT_ChartoInt16(&cmdUnitArr[0]);
  uint8 dstEP = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8 srcEP;
  if (cmdUnitArr[2].unitLen != 0) {
    srcEP = AT_ChartoInt8(&cmdUnitArr[2]);
  } else {
    srcEP = ZCL_CONTROLLER_ENDPOINT;
  }
  // uint16 profileID;
  // if (cmdUnitArr[3].unitLen != 0) {
  //   uint16 profileID = AT_ChartoInt8(&cmdUnitArr[3]);
  // } else {
  //   uint16 profileID = 0x0104;
  // }
  uint16 cID = AT_ChartoInt16(&cmdUnitArr[4]);
  // set a temp unit to store cmd
  AT_CmdUnit tempUnit;
  tempUnit.unitLen = 2;
  tempUnit.unit = cmdUnitArr[5].unit;
  uint8 fc = AT_ChartoInt8(&tempUnit);
  cmdUnitArr[5].unitLen -= 2;
  cmdUnitArr[5].unit += 2;
  tempUnit.unit = cmdUnitArr[5].unit;
  uint8 seqNum = AT_ChartoInt8(&tempUnit);
  cmdUnitArr[5].unitLen -= 2;
  cmdUnitArr[5].unit += 2;
  tempUnit.unit = cmdUnitArr[5].unit;
  uint8 cmd = AT_ChartoInt8(&tempUnit);
  cmdUnitArr[5].unitLen -= 2;
  cmdUnitArr[5].unit += 2;
  tempUnit.unit = cmdUnitArr[5].unit;

  dataLen = cmdUnitArr[5].unitLen;
  if (dataLen%2 != 0) {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }
  dataLen = dataLen / 2;
  buf = zcl_mem_alloc( dataLen );
  if ( buf != NULL )
  {
    uint8 i;

    // Load the buffer - serially
    pBuf = buf;
    // tempUnit.unitLen = 4;
    for (i = 0; i < dataLen; i++)
    {
      *pBuf++ = AT_ChartoInt8(&tempUnit);
      cmdUnitArr[5].unit += 2;
      // tempdata = AT_ChartoInt16(&tempUnit);
      // *pBuf++ = LO_UINT16(tempdata);
      // *pBuf++ = HI_UINT16(tempdata);
      // cmdUnitArr[5].unit += 4;
      tempUnit.unit = cmdUnitArr[5].unit;
    }

    // build destination address
    afAddrType_t dstAddr;
    dstAddr.endPoint = dstEP;
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    dstAddr.addr.shortAddr = addr;

    if(fc == ZCL_FRAME_TYPE_SPECIFIC_CMD)
    {
      status = zcl_SendCommand( srcEP, &dstAddr, cID, cmd, TRUE,
                             ZCL_FRAME_CLIENT_SERVER_DIR,
                             0, 0, seqNum,
                             dataLen, buf );
    } else {
      status = zcl_SendCommand( srcEP, &dstAddr, cID, cmd, FALSE,
                             ZCL_FRAME_CLIENT_SERVER_DIR,
                             0, 0, seqNum,
                             dataLen, buf );
    }


    zcl_mem_free( buf );
  }
  else
  {
    status = ZMemError;
  }

  if(status != afStatus_SUCCESS && status != ZMemError)
    AT_SEND_ERROR(status);
  else if (status == ZMemError) {
    AT_ZDO_ERROR(status);
  } else {
    AT_OK();
  }

}

/*******************************************************************************
 * @fn      AT_Cmd_RawZDO
 *
 * @brief   AT+RAWZDO - Construct A Raw ZDO Message And Send To Target
 *          Format: AT+RAWZDO:<NodeID>,<ClusterID>,<data>
 *                  <NodeID> - 16 bit hexadecimal number, network address of
 *                             a remote device.
 *                  <ClusterID> - ZDO Command ID, see ZigBeeSpecification [4] for
 *                             more information.
 *                  <data>   - a constructed ZCL command in hexadecimal format
 *                          (please check ZigBee Cluster Library for references)
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_RawZDO( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint16 dataLen;
  uint8 *buf;
  uint8 *pBuf;
  AT_CmdUnit cmdUnitArr[4];

  for ( i = 0; i < 4; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,,\r",cmdUnitArr);

  uint16 addr = AT_ChartoInt16(&cmdUnitArr[0]);
  uint16 cmd = AT_ChartoInt16(&cmdUnitArr[1]);
  uint8 str[20];
  osal_memcpy(str, cmdUnitArr[2].unit, cmdUnitArr[2].unitLen);
  dataLen = cmdUnitArr[2].unitLen / 2;
  buf = zcl_mem_alloc( dataLen );
  if ( buf != NULL )
  {
    // Load the buffer - serially
    pBuf = buf;
    AT_CmdUnit tempUnit;
    tempUnit.unit = cmdUnitArr[2].unit;
    tempUnit.unitLen = 2;
    for (i = 0; i < dataLen; i++)
    {
      *pBuf++ = AT_ChartoInt8(&tempUnit);
      cmdUnitArr[2].unit += 2;
      tempUnit.unit = cmdUnitArr[2].unit;
    }

    // build destination address
    zAddrType_t dstAddr;
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    dstAddr.addr.shortAddr = addr;

    status = ZDP_SendData( &ZDP_TransID, &dstAddr, cmd,
                           dataLen, buf, TRUE );
    zcl_mem_free( buf );
  }
  else
  {
    status = ZMemError;
  }

  if(status != afStatus_SUCCESS && status != ZMemError)
    AT_SEND_ERROR(status);
  else if (status == ZMemError) {
    AT_ZDO_ERROR(status);
  } else {
    AT_OK();
	AT_RESP("CMD:", 4);
	AT_RESP(cmdUnitArr[1].unit, cmdUnitArr[1].unitLen);
	AT_RESP(",", 1);
	AT_RESP("PAYLOAD:", 8);
	AT_RESP(str, dataLen*2);
	AT_NEW_LINE();
  }

}

/*******************************************************************************
 * @fn      AT_Cmd_InterPAN
 *
 * @brief   AT+INTERPAN - Send an Interpan Command
 *          Format: AT+INTERPAN:<AddressMode>,<DstAddress>,<DstPAN>,
 *                  <DstChannel>,<ProfileID>,<ClusterID>,<Payload>
 *                  <DstAddress> - 16 bit hexadecimal number if the user uses
 *                                 Node ID or Group ID.
 *                  <DstPAN> - 16 bit hexadecimal number representing
 *                             destination PAN ID
 *                  <ProfileID> - 16 bit hexadecimal number representing profile
 *                                ID. e.g. 0x0104 for Home automation. 0xC05E
 *                                for ZigBee Light Link.
 *                  <ClusterID> - 16 bit hex number representing Cluster ID.
 *                  <Payload> - Command payload, please input ASCII hex data.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_InterPAN( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint8 addrType;
  uint16 addr;
  uint16 dstPan;
  uint8 dstCh;
  uint16 cID;
  uint8 dataLen;
  uint8 *pBuf;
  AT_CmdUnit cmdUnitArr[7];

  for ( i = 0; i < 7; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,,,,,\r", cmdUnitArr);

  addrType = AT_ChartoInt8(&cmdUnitArr[0]);
  addr = AT_ChartoInt16(&cmdUnitArr[1]);
  dstPan = AT_ChartoInt16(&cmdUnitArr[2]);
  dstCh = AT_ChartoInt8(&cmdUnitArr[3]);
  cID = AT_ChartoInt16(&cmdUnitArr[5]);
  dataLen = cmdUnitArr[6].unitLen;
  pBuf = cmdUnitArr[6].unit;

  // Build address
  afAddrType_t dstAddr;
  dstAddr.endPoint = STUBAPS_INTER_PAN_EP;
  dstAddr.panId    = dstPan;
  if (addrType == 1) {
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  } else {
    dstAddr.addrMode = (afAddrMode_t)AddrGroup;
  }
  dstAddr.addr.shortAddr = addr;

  status = InterPAN_Send( &dstAddr, dstCh, cID, dataLen, pBuf);

  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_Scan
 *
 * @brief   AT+SCAN - Scan The Energy Of All Channels or Scan For Active PANs
 *          Format: AT+SCAN:<ScanType>
 *                  <ScanType> -  one digit boolean type. The user may input 0
 *                                to initiate energy scan. Alternatively the
 *                                user may input 1 to initiate an PAN scan
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_Scan( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoSCANCmd = FALSE;

  uint8 status;
  uint8 i;
  AT_CmdUnit cmdUnitArr[2];

  for ( i = 0; i < 2; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":\r",cmdUnitArr);

  uint8 type = AT_ChartoInt8(&cmdUnitArr[0]);
  if (type == 0) {
    NLME_ScanFields_t fields;
    fields.channels = MAX_CHANNELS_24GHZ;
    fields.duration = BEACON_ORDER_1_SECOND;
    fields.scanType = ZMAC_ED_SCAN;
    fields.scanApp  = NLME_ED_SCAN;

    //change the the call back function address of energe scan in NWK_layer.
    pZDNwkMgr_EDScanConfirmCB = AT_Cmd_ESCAN_CB;

    if ((status=NLME_NwkDiscReq2(&fields)) == ZSuccess) {
      //AT_OK();
      AT_DEBUG("\n\rPlease waitting...\n\r",sizeof("\n\rPlease waitting...\n\r")-1);
    } else {
      AT_ZDO_ERROR(status);
      NLME_NwkDiscTerm();
    }
  } else {
    zAddrType_t dstAddr;
    dstAddr.addr.shortAddr = NLME_GetShortAddr();
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;

    status = ZDP_MgmtNwkDiscReq( &dstAddr, MAX_CHANNELS_24GHZ, BEACON_ORDER_120_MSEC,
                                 0, 1 );
    if(status != afStatus_SUCCESS)
      AT_SEND_ERROR(status);
    else
      //AT_OK();
      AT_DEBUG("\n\rPlease waitting...\n\r",sizeof("\n\rPlease waitting...\n\r")-1);
  }

  //osal_start_timerEx( AT_UART_TaskID, AT_RESTORE_CMDDO_FLAG, 10000 );
}

/*******************************************************************************
 * @fn      AT_Cmd_FN
 *
 * @brief   AT+FN - Establish Network
 *          Format: AT+FN:<NWKType>,[<Ch>],[<Power>],[<PANID>]
 *                  <NWKType> - 1 digit Boolean type, the user can use 0 to form
 *                              a Centralized network, and use 1 to form a
 *                              distribute network
 *                  <Ch> - 2 digit decimal number which represents channel number.
 *                         Range from 11 to 26
 *                  <Power>
 *                  <PANID> - 16-bit decimal number which represents short PANID
 *                            of ZigBee network
 *          Use on: Coordinator and Router which are not part of a PAN
 *          Note: This command can only be executed if the local node is
 *                not part of a PAN already.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_FN( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoFNCmd = FALSE;

  uint8 i;
  // uint8 nt;
  // bool nwktype;
  uint32 channel = 1;
  uint16 PANID;
  AT_CmdUnit cmdUnitArr[5];

  osal_nv_read(ZCD_NV_BDBNODEISONANETWORK, 0,
               sizeof(bdbAttributes.bdbNodeIsOnANetwork),
               &bdbAttributes.bdbNodeIsOnANetwork);
  if (bdbAttributes.bdbNodeIsOnANetwork == TRUE) {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }

  for ( i = 0; i < 5; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,,,\r",cmdUnitArr);

  // nt = AT_ChartoInt8(&cmdUnitArr[0]);
  // if (nt == 1) {
  //   nwktype = true; // distribute network
  // } else {
  //   nwktype = false; // Centralized network
  // }
  if (cmdUnitArr[1].unitLen == 0) {
    channel = MAX_CHANNELS_24GHZ;
  } else {
    channel = ((uint32) 1) << (AT_ChartoInt8(&cmdUnitArr[1])/16*10+AT_ChartoInt8(&cmdUnitArr[1])%16);
  }
  // AT_ChartoIntx(&cmdUnitArr[3],ZDO_UseExtendedPANID, 64);
  if(cmdUnitArr[3].unitLen==0){
    PANID = 0xFFFF;
  } else {
    PANID = AT_ChartoInt16(&cmdUnitArr[3]);
  }

  // set BDB channel attribute
  bdb_setChannelAttribute( TRUE, channel );
  bdb_setChannelAttribute( FALSE, channel );

  zgConfigPANID = PANID;
  uint8 status = osal_nv_item_init( ZCD_NV_PANID, sizeof(zgConfigPANID), &zgConfigPANID );
  if ( status == ZSUCCESS ) {
    osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
  } else {
    AT_ERROR(AT_FORM_NWK_FAIL);
    return;
  }

  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_FINDING_BINDING);

  osal_start_timerEx( AT_UART_TaskID, AT_RESTORE_CMDDO_FLAG, 5000 );
  //AT_OK();

  /*// set channel and save it in Nv
  bdb_setChannel(channel);
  zgConfigPANID = PANID;
  if (ZG_BUILD_COORDINATOR_TYPE) {
    startMode = MODE_HARD;
  } else {
    startMode = MODE_JOIN;
  }

  // Initialize apps and start the network
  ZDApp_ChangeState( DEV_INIT );

  ZDO_StartDevice( (uint8)ZDO_Config_Node_Descriptor.LogicalType, startMode,
                   BEACON_ORDER_NO_BEACONS, BEACON_ORDER_NO_BEACONS );
  */
}

/*******************************************************************************
 * @fn      AT_Cmd_JN
 *
 * @brief   AT+JN - Join Network
 *          Format: AT+JN[:<channel>],[<PANID>],[<EPANID>]
 *                  <Ch> - 2 digit decimal number which represents channel number.
 *                         Range from 11 to 26
 *                  <PANID> - 16-bit decimal number which represents short PANID
 *                            of ZigBee network
 *                  <EPANID> - 64-bit hexadecimal number which represents
 *                             extended PAN ID of ZigBee network
 *          Use on: Joinable devices which are not part of a PAN
 *          Note: This command can only be executed if the local node is
 *                not part of a PAN already.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_JN( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoJNCmd = FALSE;   // set in process flag

  uint8 i;
  uint32 channel = 1;
  uint16 PANID;
  AT_CmdUnit cmdUnitArr[4];

  osal_nv_read(ZCD_NV_BDBNODEISONANETWORK, 0,
               sizeof(bdbAttributes.bdbNodeIsOnANetwork),
               &bdbAttributes.bdbNodeIsOnANetwork);
  if (bdbAttributes.bdbNodeIsOnANetwork == TRUE) {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }

  for ( i = 0; i < 4; i++ )
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  if (cmdUnitArr[0].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,,\r",cmdUnitArr);
  }


  if (cmdUnitArr[0].unitLen == 0) {
    channel = MAX_CHANNELS_24GHZ;
  } else {
    channel = ((uint32) 1) << (AT_ChartoInt8(&cmdUnitArr[0])/16*10+AT_ChartoInt8(&cmdUnitArr[0])%16);
  }
  if (cmdUnitArr[1].unitLen == 0) {
    PANID = 0xFFFF;
  } else {
    PANID = AT_ChartoInt16(&cmdUnitArr[1]);
  }
  if (cmdUnitArr[2].unitLen == 0) {
    // invalid address enable the device allow all the ExtPANid
    osal_memcpy( ZDO_UseExtendedPANID, "\0\0\0\0\0\0\0\0", 8);
  } else {
    AT_ChartoIntx(&cmdUnitArr[2], ZDO_UseExtendedPANID, 64);
  }

  // set BDB channel attribute
  bdb_setChannelAttribute( TRUE, channel );
  bdb_setChannelAttribute( FALSE, channel );

  zgConfigPANID = PANID;
  uint8 status = osal_nv_item_init( ZCD_NV_PANID, sizeof(zgConfigPANID), &zgConfigPANID );
  if ( status == ZSUCCESS ) {
    osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
  } else {
    AT_ERROR(AT_FORM_NWK_FAIL);
    return;
  }

  /*// set channel and save it in Nv
  bdb_setChannel(channel);
  zgConfigPANID = PANID;
  if (ZG_BUILD_JOINING_TYPE) {
    startMode = MODE_REJOIN;
  } else {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }*/

  //register the ZDO call back functio to receive the join confirm
  ZDO_RegisterForZdoCB(ZDO_JOIN_CNF_CBID, AT_ZDO_ProcessJOIN_CNF_CB);

  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING );

  osal_start_timerEx( AT_UART_TaskID, AT_RESTORE_CMDDO_FLAG, 5000 );
  /*// Initialize apps and start the network
  ZDApp_ChangeState( DEV_INIT );

  ZDO_StartDevice( (uint8)ZDO_Config_Node_Descriptor.LogicalType, startMode,
                   BEACON_ORDER_NO_BEACONS, BEACON_ORDER_NO_BEACONS );
  */
}

/*******************************************************************************
 * @fn      AT_Cmd_LN
 *
 * @brief   AT+LN - Make Local Device Leave PAN
 *          Format: AT+LN
 *          Use on: All Device
 *          Note: Use with care on a Coordinator. It will not be able to re-join
 *                the PAN
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_LN( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0],cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

  osal_nv_read(ZCD_NV_BDBNODEISONANETWORK, 0,
               sizeof(bdbAttributes.bdbNodeIsOnANetwork),
               &bdbAttributes.bdbNodeIsOnANetwork);
  if (bdbAttributes.bdbNodeIsOnANetwork == FALSE) {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }

  zAddrType_t dstAddr;
  dstAddr.addr.shortAddr = NLME_GetShortAddr();
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;

  status = ZDP_MgmtLeaveReq( &dstAddr, NLME_GetExtAddr(), FALSE, FALSE, TRUE );
  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_N
 *
 * @brief   AT+N - Display Network Information
 *          Format: AT+N
 *          Use on: All Device
 *          Response: +N=<devicetype>,<channel>,<power>,<PANID>,<EPANID>
 *                    or +N=NoPAN
 *                    followed by OK
 *
 *                   <devicetype> -  represents the node's functionality
 *                                   in the PAN (FFD,COO,ZED,SED,MED)
 *                   <channel> - the IEEE 802.15.4 radio channel (11-26)
 *                   <power> - the node's output power in dBm
 *                   <PANID> - the node's PAN ID
 *                   <EPANID> - the node's extended PAN ID.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_N( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 txPower;
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

  osal_nv_read(ZCD_NV_BDBNODEISONANETWORK, 0,
               sizeof(bdbAttributes.bdbNodeIsOnANetwork),
               &bdbAttributes.bdbNodeIsOnANetwork);
  if (bdbAttributes.bdbNodeIsOnANetwork == FALSE) {
    AT_NEW_LINE();
    AT_RESP("+N=NoPAN", 8);
    AT_NEW_LINE();
    AT_OK();
    return;
  }

  char str[20];
  AT_NEW_LINE();
  AT_RESP("+N=", 3);
  switch (ZDO_Config_Node_Descriptor.LogicalType) {
    case NODETYPE_COORDINATOR:
      AT_RESP("COO", 3);
    break;
    case NODETYPE_ROUTER:
      AT_RESP("ROUTER", 6);
    break;
    case NODETYPE_DEVICE:
      AT_RESP("ZED", 3);
    break;
  }
  AT_RESP(",", 1);
  printf("%02d\n", _NIB.nwkLogicalChannel);
  AT_RESP(",", 1);
#if defined MAC_RUNTIME_CC2591 || defined MAC_RUNTIME_CC2590 || \
  defined MAC_RUNTIME_CC2592
  const uint8 CODE *pTable = macRadioDefsTxPwrTables[macRadioDefsRefTableId >> 4];
#elif defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590 || \
  defined HAL_PA_LNA_CC2592
  const uint8 CODE *pTable = macRadioDefsTxPwrTables[0];
#else
  const uint8 CODE *pTable = macRadioDefsTxPwrBare;
#endif
  txPower = pMacPib->phyTransmitPower;
  /* if the selected dBm is out of range, use the closest available */
  if ((int8)txPower > (int8)pTable[MAC_RADIO_DEFS_TBL_TXPWR_FIRST_ENTRY])
  {
    /* greater than base value -- out of table range */
    txPower = pTable[MAC_RADIO_DEFS_TBL_TXPWR_FIRST_ENTRY];
  }
  else if ((int8)txPower < (int8)pTable[MAC_RADIO_DEFS_TBL_TXPWR_LAST_ENTRY])
  {
    /* smaller than the lowest power level -- out of table range */
    txPower = pTable[MAC_RADIO_DEFS_TBL_TXPWR_LAST_ENTRY];
  }
  if ((int8)txPower >= 0) {
    AT_RESP("+", 1);
    printf("%02d,", txPower);
  } else {
    AT_RESP("-", 1);
    printf("%02d,", ((int8)txPower)*-1);
  }
  AT_Int16toChar(_NIB.nwkPanId, (uint8 *)str);
  AT_RESP(str, 4);
  AT_RESP(",", 1);
  AT_GetIEEEAddrStr(_NIB.extendedPANID, (uint8 *)str);
  AT_RESP(str, 16);
  AT_NEW_LINE();
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_PJ
 *
 * @brief   AT+PJ - Permit joining
 *          Format: AT+PJ[:<sec>,<NodeID>]
 *          Use on: Coordinator or router
 *          Response: OK
 *                    or
 *                    ERROR:<errorcode>
 *
 *                   <sec> - 8 bit hexadecimal number which represents
 *                           the length of time in seconds during which
 *                           the ZigBee coordinator or router will
 *                           allow associations
 *                   <NodeID> - 16 bit hexadecimal number, network address of
 *                              a target device. If FFFC is used, the
 *                              constructed command will be sent as a broadcast
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_PJ( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[3];

  for( i = 0; i < 3; i++ )
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  if (cmdUnitArr[0].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);
    zAddrType_t addr = {
      {_NIB.nwkDevAddress},
      (afAddrMode_t)Addr16Bit
    };
    status = ZDP_MgmtPermitJoinReq(&addr, 60, true, 1);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);
    //build broadcast address
    zAddrType_t addr;
    uint16 address = AT_ChartoInt16(&cmdUnitArr[1]);
    if (address == NWK_BROADCAST_SHORTADDR_DEVZCZR) {
      addr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVZCZR;
      addr.addrMode = (afAddrMode_t)AddrBroadcast;
    } else {
      addr.addr.shortAddr = address;
      addr.addrMode = (afAddrMode_t)Addr16Bit;
    }
    uint8 duration = AT_ChartoInt8(&cmdUnitArr[0]);
    status = ZDP_MgmtPermitJoinReq(&addr, duration, true, 1);
  }

  if (status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_RJ
 *
 * @brief   AT+RJ - Rejoin the network
 *          Format: AT+RJ:b
 *                  b - If b is set to 0 join without the known network key
 *                      (unencrypted) and if b is set to 1 join encrypted
 *          Use on: not Coordinator
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_RJ( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  // uint8 status;
  // uint8 joinType;
  AT_CmdUnit cmdUnitArr[2];

  for( i = 0; i < 2; i++ )
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);

  // Set NWK task to stop
  nwk_setStateIdle( FALSE );

  // Use the new network paramters
  zgConfigPANID = _NIB.nwkPanId;
  zgDefaultChannelList = _NIB.channelList;
  osal_cpyExtAddr( ZDO_UseExtendedPANID, _NIB.extendedPANID );

  // set runtimeChannel to MAX_CHANNELS_24GHZ
  bdb_setChannel( MAX_CHANNELS_24GHZ );

  _NIB.nwkState = NWK_INIT;
  _tmpRejoinState = TRUE;
  devStartMode = MODE_REJOIN;

  //register the ZDO call back functio to receive the join confirm
  ZDO_RegisterForZdoCB(ZDO_JOIN_CNF_CBID, AT_ZDO_ProcessJOIN_CNF_CB);

  // Start the network joining process
  osal_set_event( ZDAppTaskID, ZDO_NETWORK_INIT );

  AT_NEW_LINE();
  AT_RESP("LOSTPAN", 7);
  AT_NEW_LINE();
}

/*******************************************************************************
 * @fn      AT_Cmd_KeyTab
 *
 * @brief   AT+KEYTAB - Print Local Key And Link Key Table
 *          Format: AT+KEYTAB
 *          Use on: All Device
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_KeyTab( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8  i;
  uint16 index;
  uint8  retValue;
  uint8  *pKey;
  nwkActiveKeyItems keyItems;
  // APSME_LinkKeyData_t *pApsLinkKey = NULL;
  // uint16 apsLinkKeyNvId;
  uint8 str[8];
  AT_CmdUnit cmdUnitArr[1];

  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  AT_NEW_LINE();
  // show Network Key
  SSP_ReadNwkActiveKey( &keyItems );
  AT_RESP("NWK:", 4);
  pKey = keyItems.active.key;
  for ( i = 0; i < SEC_KEY_LEN; i++ ) {
    AT_Int8toChar(pKey[i], str);
    AT_RESP(str, 2);
  }
  AT_RESP(",", 1);
  printf("%08X", keyItems.frameCounter);

  // show default TC_Link_Key
  retValue = ZDSecMgrReadKeyFromNv( ZCD_NV_TCLK_DEFAULT, pKey );
  if (retValue != ZSuccess) {
    AT_ZDO_ERROR(retValue);
	  return;
  }
  AT_NEXT_LINE();
  AT_RESP("LINKFF:", 7);
  for ( i = 0; i < SEC_KEY_LEN; i++ ) {
    AT_Int8toChar(pKey[i], str);
    AT_RESP(str, 2);
  }

  // show TC_Link_Key in the Key Table
  AddrMgrEntry_t addrEntry;
  if ( ZDSecMgrEntries != NULL )
  {
    // TODO: the numbers of entrys in defined by ZDSECMGR_ENTRY_MAX
    // here I set it to 3 [define in ZDSecMgr.c]
    for ( index = 0; index < 3 ; index++ )
    {
      addrEntry.user  = ADDRMGR_USER_SECURITY;
      addrEntry.index = ZDSecMgrEntries[index].ami;
      if (AddrMgrEntryGet( &addrEntry ) == TRUE) {
        // Fetch the key NV ID
        retValue = ZDSecMgrReadKeyFromNv(ZDSecMgrEntries[index].keyNvId, pKey);
        if (retValue != ZSuccess) {
          AT_ZDO_ERROR(retValue);
      	  return;
        }
        AT_NEXT_LINE();
        printf("LINK%02X:", index);
        for ( i = SEC_KEY_LEN-1; ((int8)i) >= 0; i-- ) {
          AT_Int8toChar(pKey[i], str);
          AT_RESP(str, 2);
        }
        AT_RESP(",", 1);
        AT_GetIEEEAddrStr(addrEntry.extAddr, (uint8 *)str);
        AT_RESP(str, 16);
      }
    }
  }
  // AddrMgrEntry_t addrEntry;
  // for ( addrEntry.index = 0; addrEntry.index < NWK_MAX_ADDRESSES; addrEntry.index++ ) {
  //   if (AddrMgrEntryGet( &addrEntry )) {
  //     pExtAddr = addrEntry.extAddr;
  //     // Fetch the key NV ID
  //     retValue = APSME_LinkKeyNVIdGet( pExtAddr, &apsLinkKeyNvId );
  //
  //     if (retValue == ZSuccess)
  //     {
  //       // retrieve key from NV
  //       if (osal_nv_read( apsLinkKeyNvId, 0,
  //                   sizeof(APSME_LinkKeyData_t), pApsLinkKey) == SUCCESS)
  //       {
  //         AT_NEXT_LINE();
  //         printf("LINK%02X:", addrEntry.index);
  //         pKey = pApsLinkKey->key;
  //         for ( i = 0; i < SEC_KEY_LEN; i++ ) {
  //           AT_Int8toChar(pKey[i], str);
  //           AT_RESP(str, 2);
  //         }
  //         AT_RESP(",", 1);
  //         printf("%04X%04X%04X%04X", pExtAddr[3], pExtAddr[2], pExtAddr[1], pExtAddr[0]);
  //       } else {
  //         retValue = AT_NWK_UNKNOWN_DEV;
  //         break;
  //       }
  //     }
  //   }
  // }
  AT_NEW_LINE();

  if (retValue != ZSuccess) {
    AT_ERROR(retValue);
	  return;
  } else {
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_AddKey
 *
 * @brief   AT+ADDKEY - Add a key entry to local key table (used on trust centre)
 *          Format: AT+ADDKEY:<EUI>[,<InstallCode>]
 *          Use on: Forming device to allow other devices to join
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_AddKey( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint8 removefn;
  uint8 InstallCode[INSTALL_CODE_LEN+INSTALL_CODE_CRC_LEN];
  uint16 shortAddr;
  uint8 pExtAddr[Z_EXTADDR_LEN];
  AT_CmdUnit cmdUnitArr[3];

  for( i = 0; i < 3; i++ )
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  if (cmdUnitArr[1].unitLen == 0) {
    AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);
  }

  AT_ChartoIntx(&cmdUnitArr[0], pExtAddr, 64);
  if (cmdUnitArr[1].unitLen == 0) {
    removefn = TRUE;
  } else {
    removefn = FALSE;
    AT_ChartoIntx(&cmdUnitArr[1], InstallCode, 128);
  }

  if (!removefn) {
  	if(AddrMgrNwkAddrLookup(pExtAddr, &shortAddr))
    {
      status = ZDSecMgrAddLinkKey( shortAddr, pExtAddr, InstallCode );
    } else {
      status = AT_NWK_UNKNOWN_DEV;
    }

    /*uint16 crc = bdb_GenerateInstallCodeCRC(InstallCode);
    InstallCode[INSTALL_CODE_LEN] = crc & 0xFF;
    InstallCode[INSTALL_CODE_LEN + 1] = crc >> 8;

    #if (ZG_BUILD_COORDINATOR_TYPE)
    if (ZG_DEVICE_COORDINATOR_TYPE)
    {
      status = bdb_addInstallCode(InstallCode, pExtAddr);
    }
    else
    {
      status = bdb_setActiveCentralizedLinkKey(FALSE, InstallCode);
    }
    #else
    status = bdb_setActiveCentralizedLinkKey(FALSE, InstallCode);
    #endif*/
  } else {
    if( ( ZG_BUILD_COORDINATOR_TYPE ) && ( ZG_DEVICE_COORDINATOR_TYPE ))
    {
      uint16 tempIndex;
      APSME_TCLKDevEntry_t TCLKDevEntry;
      uint8 found;

      tempIndex = APSME_SearchTCLinkKeyEntry(pExtAddr,&found, &TCLKDevEntry);

      if(found)
      {
        uint16 i;

        i = tempIndex - ZCD_NV_TCLK_TABLE_START;
        //Reset the frame counter associated to this device  TCLinkKeyFrmCntr
        TCLinkKeyFrmCntr[i].txFrmCntr = 0;
        TCLinkKeyFrmCntr[i].rxFrmCntr = 0;

        if(TCLKDevEntry.keyAttributes == ZG_PROVISIONAL_KEY)
        {
          APSME_EraseICEntry(&TCLKDevEntry.SeedShift_IcIndex);
        }

        osal_memset(&TCLKDevEntry,0,sizeof(APSME_TCLKDevEntry_t));
        osal_nv_write( ( tempIndex), 0, sizeof(APSME_TCLKDevEntry_t), &TCLKDevEntry );
        status = ZSuccess;
      }
    }
    else
    {
      status = ZDSecMgrDeviceRemoveByExtAddr( pExtAddr );
    }
  }

  if (status != ZSuccess) {
    AT_ZDO_ERROR(status);
  } else {
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_IDREQ
 *
 * @brief   +IDREQ - Request Node's NodeID (ZDO)
 *          Format: AT+IDREQ:<Address>[,XX]
 *          Response: OK or ERROR:<errorcode>
 *          Prompt: AddrResp:<errorcode>[,<NodeID>,<EUI64>] [nn. <NodeID>]
 *          Use on: All Devices
 *          Note: Where <Address> can be a node's EUI64, or address table entry
 *                and XX is an optional index number. When an index number is
 *                provided, an extended response is requested asking the remote
 *                device to list its associated devices (ie children).
 *                It then sends a broadcast to obtain the specified Device's
 *                NodeID and optionally also elements of its associated devices
 *                list.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_IDREQ( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[3];
  uint8 i;
  for(i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);

  uint8 ext[8];
  AddrMgrEntry_t entry;
  if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], ext, 64);
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet(&entry);
    osal_memcpy(ext, entry.extAddr, Z_EXTADDR_LEN);
  }
  if (cmdUnitArr[1].symbol == ',' ) {
    AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);
    uint8 status;
    status = ZDP_NwkAddrReq( ext, ZDP_ADDR_REQTYPE_EXTENDED,
                              AT_ChartoInt8(&cmdUnitArr[1]), TRUE );
    if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":\r",cmdUnitArr);
    uint8 status;
    status = ZDP_NwkAddrReq( ext, ZDP_ADDR_REQTYPE_SINGLE,
                          0xFF, TRUE );
    if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  }
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_EUIREQ
 *
 * @brief   +EUIREQ - Request Node's EUI64 (ZDO)
 *          Format: AT+EUIREQ:<Address>,<NodeID>[,XX]
 *          Response: OK or ERROR:<errorcode>
 *          Prompt: AddrResp:<errorcode>[,<NodeID>,<EUI64>]
 *          Use on: All Devices
 *          Note: Where <Address> is Node ID or address table entry
 *                of the node which is to be interrogated about the node with
 *                the Node ID specified in <NodeID>. XX is an optional index number.
 *                In case an index number is provided, an extended response is
 *                requested asking the remote device to list its associated
 *                devices (i.e. children).
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_EUIREQ( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoEUIREQcmd = FALSE;
  AT_CmdUnit cmdUnitArr[4];
  uint8 i;
  for(i = 0; i < 4; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);

  uint16 dstaddr;
  AddrMgrEntry_t entry;
  if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    dstaddr = AT_ChartoInt16(&cmdUnitArr[0]);
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstaddr = entry.nwkAddr;
  }
  if (cmdUnitArr[2].symbol == ',') {
    AT_PARSE_CMD_PATTERN_ERROR(":,,\r", cmdUnitArr);
    uint8 status;
    status = ZDP_IEEEAddrReq( dstaddr, AT_ChartoInt16(&cmdUnitArr[1]), ZDP_ADDR_REQTYPE_EXTENDED,
                            AT_ChartoInt8(&cmdUnitArr[2]), TRUE );
    if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);
    uint8 status;
    status = ZDP_IEEEAddrReq( dstaddr, AT_ChartoInt16(&cmdUnitArr[1]), ZDP_ADDR_REQTYPE_SINGLE,
                            0xFF, TRUE );
    if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  }
  AT_SEQ(ZDP_TransID);
  AT_SEQ_STROE(ZDP_TransID);
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_NODEDESC
 *
 * @brief   +NODEDESC - Request Node's Descriptor (ZDO)
 *          Format: AT+NODEDESC:<Address>,<NodeID>
 *                  Where <Address> is the EUI64, NodeID or Address table entry
 *                  of the node which is to be interrogated about the node with
 *                  the NodeID specified in <NodeID>.
 *          Use on: All Devices
 *          Note: Sends a unicast to obtain the specified device's node descriptor.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_NODEDESC( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoNODEDESEcmd = FALSE;
  uint8 status;
  AT_CmdUnit cmdUnitArr[3];
  uint8 i;
  for (i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  status = ZDP_NodeDescReq(dstAddr, AT_ChartoInt16(&cmdUnitArr[1]), TRUE);
  osal_mem_free(dstAddr);
  if (status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_POWERDESC
 *
 * @brief   +POWERDESC - Request Node's Power Descriptor (ZDO)
 *          Format: AT+POWERDESC:<Address>,<NodeID>
 *                  Where <Address> is the EUI64, NodeID or Address table entry
 *                  of the node which is to be interrogated about the node with
 *                  the NodeID specified in <NodeID>.
 *          Use on: All Devices
 *          Prompt: PowerDesc:<NodeID>,<errorcode>[,<PowerDescriptor>]
 *          Note: Sends a unicast to obtain the specified device's node descriptor.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_POWERDESC( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;
  uint8 i;
  AT_CmdUnit cmdUnitArr[3];
  for (i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  status = ZDP_PowerDescReq(dstAddr,AT_ChartoInt16(&cmdUnitArr[1]), TRUE);
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_ACTEPDESC
 *
 * @brief   +ACTEPDESC - Request Node's Active Endpoint List (ZDO)
 *          Format: AT+ACTEPDESC:<Address>,<NodeID>
 *                  Where <Address> is the EUI64, NodeID or Address table entry
 *                  of the node which is to be interrogated about the node with
 *                  the NodeID specified in <NodeID>.
 *          Use on: All Devices
 *          Prompt: ActEpDesc:<NodeID>,<errorcode>[,XX,...]
 *          Note: Sends a unicast to obtain the specified device's node descriptor.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ACTEPDESC( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;
  uint8 i;
  AT_CmdUnit cmdUnitArr[3];
  for (i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  status = ZDP_ActiveEPReq(dstAddr, AT_ChartoInt16(&cmdUnitArr[1]), TRUE);
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_SIMPLEDESC
 *
 * @brief   +SIMPLEDESC - Request Endpoint's Simple Descriptor (ZDO)
 *          Format: AT+SIMPLEDESC:<Address>,<NodeID>,<XX>
 *                  Where <Address> is the EUI64, NodeID or Address table entry
 *                  of the node which is to be interrogated about the node with
 *                  the NodeID specified in <NodeID> and XX is the number of the
 *                  endpoint, which simple descriptor is to be read. XX should
 *                  be hexadecimal number.
 *          Use on: All Devices
 *          Note: Sends a unicast to obtain the specified device's node descriptor.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_SIMPLEDESC( uint8 cmd_ptr, uint8* msg_ptr )
{
  notdoSIMPLEDESCcmd = FALSE;
  uint8 status;
  uint8 i;
  AT_CmdUnit cmdUnitArr[4];
  for (i = 0; i < 4; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8 XX = AT_ChartoInt8(&cmdUnitArr[2]);
  if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  status = ZDP_SimpleDescReq( dstAddr, AT_ChartoInt16(&cmdUnitArr[1]), XX, TRUE );
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_MATCHREQ
 *
 * @brief   +MATCHREQ - Find Nodes which Match a Specific Descriptor (ZDO)
 *          Format: AT+MATCHREQ:<ProfileID>,<NumInClusters>[,<InClusterList>],
 *                              <NumOutClusters>[,<OutClusterList>]
 *                  Where <ProfileID> Required profile ID of the device being
 *                  searched for followed by a specification of required input
 *                  and output clusters. If a remote node has a matching
 *                  ProfileID and matches at least one of the specified clusters
 *                  it will respond to this broadcast listing the matching endpoint(s).
 *          Use on: All Devices
 *          Note: <NumInClusters> and <NumOutClusters> must be 2 hexadecimal digits
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_MATCHREQ( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];
  uint8 i;
  uint8 status;
  uint16 ProfileID;
  uint8 NumInClusters;
  uint8 NumOutClusters;
  uint16 *InClusterList;
  uint16 *OutClusterList;

  // ProfileID
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR(":", cmdUnitArr);
  ProfileID= AT_ChartoInt16(&cmdUnitArr[0]);

  // NumInClusters
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
  NumInClusters = AT_ChartoInt8(&cmdUnitArr[0]);

  // InClusterList
  InClusterList = (uint16*)osal_mem_alloc(NumInClusters*2);
  if(InClusterList == NULL) {
    AT_ZDO_ERROR(ZMemError);
    return;
  }
  for (i = 0; i < NumInClusters; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
    AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
    InClusterList[i] = AT_ChartoInt16(&cmdUnitArr[0]);
  }

  // NumOutClusters
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
  NumOutClusters = AT_ChartoInt8(&cmdUnitArr[0]);

  //OutClusterList
  OutClusterList = (uint16*)osal_mem_alloc(NumOutClusters*2);
  if (OutClusterList == NULL) {
    osal_mem_free(InClusterList);
    AT_ZDO_ERROR(ZMemError);
    return;
  }
  for (i = 0; i < NumOutClusters; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
    AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
    OutClusterList[i] = AT_ChartoInt16(&cmdUnitArr[0]);
  }

  //check the end of the command
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  //build broadcast address
  zAddrType_t broad_addr={
    {NWK_BROADCAST_SHORTADDR_DEVZCZR},   //addr
    (afAddrMode_t)AddrBroadcast,         //addr mode
  };

  status = ZDP_MatchDescReq( &broad_addr, 0xFFFF, ProfileID,
                           NumInClusters, InClusterList,
                           NumOutClusters, OutClusterList, TRUE);
  osal_mem_free(OutClusterList);
  osal_mem_free(InClusterList);

  if(status != afStatus_SUCCESS)  AT_SEND_ERROR(status);
  else AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_ANNCE
 *
 * @brief   +ANNCE - Announce Local Device In The Network (ZDO)
 *          Format: AT+ANNCE
 *          Use on: All Devices
 *          Note: Send a ZigBee device announcement.
 *                Broadcast announcing the local node on the network.
 *          CapabilityFlags Bitmap values:
 *                CAPINFO_ALTPANCOORD           0x01
 *                CAPINFO_DEVICETYPE_FFD        0x02
 *                CAPINFO_DEVICETYPE_RFD        0x00
 *                CAPINFO_POWER_AC              0x04
 *                CAPINFO_RCVR_ON_IDLE          0x08
 *                CAPINFO_SECURITY_CAPABLE      0x40
 *                CAPINFO_ALLOC_ADDR            0x80
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ANNCE( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

  status = ZDP_DeviceAnnce( NLME_GetShortAddr(), NLME_GetExtAddr(), _NIB.CapabilityFlags, 0 );
  if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else AT_OK();
}

#if ( ZG_BUILD_RTR_TYPE )
/*******************************************************************************
 * @fn      AT_Cmd_PANNCE
 *
 * @brief   +PANNCE - Send A Parent Announce In The Network (ZDO)
 *          Format: AT+PANNCE
 *          Use on: FFD Devices
 *          Note: Send a ZigBee parent device announce command.
 *                The user can use Unicast and Broadcast announcing on the network.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_PANNCE( uint8 cmd_ptr, uint8* msg_ptr )
{
  // Make sure used by FFD devices
  if (!ZG_BUILD_RTR_TYPE) {
    AT_ERROR(AT_OPERATION_INVALID);
  }
  uint8 i;
  uint8 status;
  uint16 address;
  uint8 NumOfChildren;
  ZDO_ChildInfo_t* ChildList;
  AT_CmdUnit cmdUnitArr[1];

  // Address
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR(":", cmdUnitArr);
  address = AT_ChartoInt16(&cmdUnitArr[0]);
  //build broadcast address
  zAddrType_t dstAddr;
  if (address == NWK_BROADCAST_SHORTADDR_DEVZCZR) {
    dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVZCZR;
    dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  } else {
    dstAddr.addr.shortAddr = address;
    dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  }

  // NumOfChildren
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
  NumOfChildren = AT_ChartoInt8(&cmdUnitArr[0]);

  // EUIOfChild
  ChildList = (ZDO_ChildInfo_t*)osal_mem_alloc(NumOfChildren*sizeof(ZDO_ChildInfo_t));
  if(ChildList == NULL) {
    AT_ZDO_ERROR(ZMemError);
    return;
  }
  for (i = 0; i < NumOfChildren; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
    AT_PARSE_SINGLE_CMD_PATTERN_ERROR(",", cmdUnitArr);
    AT_ChartoIntx(&cmdUnitArr[0], (uint8*)(&(ChildList[i].extAddr)), 64);
  }

  status = ZDP_ParentAnnceReq( dstAddr, NumOfChildren, (uint8*)ChildList, TRUE );
  osal_mem_free(ChildList);

  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_NTABLE
 *
 * @brief   +NTABLE - Display Neighbour Table
 *          Format: AT+NTABLE:XX,<address>
 *                  XX - The start index of the remote LQI table
 *                  <address> - the remote node's EUI64 or NodeID or address
 *                              table entry
 *          Use on: FFD,COO Devices
 *          Note: This command requests the target node to respond by
 *                listing its neighbour table starting from the requested
 *                index. Can be used to find the identity of all ZigBee
 *                devices in the network.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_NTABLE( uint8 cmd_ptr, uint8* msg_ptr )
{
  // Make sure used by FFD devices
  if (!ZG_DEVICE_RTR_TYPE) {
    AT_ERROR(AT_OPERATION_INVALID);
    return;
  }
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[3];

  for ( i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8 startIndex = AT_ChartoInt8(&cmdUnitArr[0]);
  if (cmdUnitArr[1].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[1], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[1].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[1]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else if (cmdUnitArr[1].unitLen > 0) { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else {
    AT_ERROR(AT_LACK_PARA);
    return;
  }

  status = ZDP_MgmtLqiReq( dstAddr, startIndex, 0 );
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS) AT_SEND_ERROR(status);
  else AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_RTABLE
 *
 * @brief   AT+RTABLE:XX,<address>
 *          Format: AT+RTABLE:XX,<address>
 *                  XX - The start index of the remote LQI table
 *                  <address> - the remote node's EUI64 or NodeID
 *          Use on: FFD,COO Devices
 *          Note: This command requests the target node to respond by
 *                listing its routing table starting from the requested index.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_RTABLE( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[3];

  for (i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8 startIndex = AT_ChartoInt8(&cmdUnitArr[0]);
  if (cmdUnitArr[1].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[1], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[1].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[1]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else if (cmdUnitArr[1].unitLen > 0) { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    dstAddr->addr.shortAddr = entry.nwkAddr;
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else {
    AT_ERROR(AT_LACK_PARA);
    return;
  }

  status = ZDP_MgmtRtgReq( dstAddr, startIndex, 0 );
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}
#endif // ZG_BUILD_RTR_TYPE

#if defined ( REFLECTOR )
#if defined ( ZDO_MGMT_BIND_RESPONSE )
/*******************************************************************************
 * @fn      AT_Cmd_BTABLE
 *
 * @brief   AT+BTABLE:XX,<address>
 *          Format: AT+BTABLE:XX,<address>
 *                  XX - The start index of the remote LQI table
 *                  <address> - the remote node's EUI64 or NodeID
 *          Use on: All Devices
 *          Note: This command requests the target node to respond by
 *                listing its binding table starting from the requested index.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_BTABLE( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint8 index;
  AT_CmdUnit cmdUnitArr[3];

  for (i = 0; i < 3; i++)
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR(":,\r",cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8 startIndex = AT_ChartoInt8(&cmdUnitArr[0]);
  if (cmdUnitArr[1].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[1], dstAddr->addr.extAddr, 64);
    dstAddr->addrMode = (afAddrMode_t)Addr64Bit;
  } else if (cmdUnitArr[1].unitLen > 2) { // <Address> is NodeID
    dstAddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[1]);
    dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
  } else if (cmdUnitArr[1].unitLen > 0) { // <Address> is address table entry
    index = AT_ChartoInt8(&cmdUnitArr[0]);
    if (index != 0xFF) {
      entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
      // entry.user = ADDRMGR_USER_DEFAULT;
      AddrMgrEntryGet( &entry );
      dstAddr->addr.shortAddr = entry.nwkAddr;
      dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
    } else {
      dstAddr->addr.shortAddr = NLME_GetShortAddr();
      dstAddr->addrMode = (afAddrMode_t)Addr16Bit;
    }
  } else {
    AT_ERROR(AT_LACK_PARA);
    return;
  }

  status = ZDP_MgmtBindReq( dstAddr, startIndex, 0 );
  osal_mem_free(dstAddr);
  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}
#endif // ZDO_MGMT_BIND_RESPONSE

/*******************************************************************************
 * @fn      AT_Cmd_BSET
 *
 * @brief   +BSET - Set Local Binding Table Entry
 *          Format: AT+BSET:<type>,<LocalEP>,<ClusterID>,<DstAddress>[,<DstEP>]
 *                  <Type> - the type of binding:
 *                    1 = Unicast Binding with EUI64 and remote EP specified
 *                    2 = Many to one Binding with EUI64 and remote EP Specified
 *                    3 = Multicast Binding with Multicast ID Specified
 *                  <LocalEP> - the local endpoint
 *                  <ClusterID> - the t cluster ID, Address is either the EUI64
 *                                of the target device, or a multicast ID
 *                  <DstEP> - the remote endpoint which is not specified in case
 *                            of a multicast binding.
 *          Use on: All Devices
 *          Note: The new binding is created in the next available free binding
 *                table entry.
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_BSET( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 dstEP = 0;
  AT_CmdUnit cmdUnitArr[6];

  for ( i = 0; i < 6; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }

  uint8  type    = AT_ChartoInt8(&cmdUnitArr[0]);
  uint8  localEP = AT_ChartoInt8(&cmdUnitArr[1]);
  uint16 cID     = AT_ChartoInt16(&cmdUnitArr[2]);

  zAddrType_t dstAddr;
  if (type == 1 || type == 2) {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,\r", cmdUnitArr);
    dstEP = AT_ChartoInt8(&cmdUnitArr[4]);
    dstAddr.addrMode = Addr64Bit;
    AT_ChartoIntx(&cmdUnitArr[3], dstAddr.addr.extAddr, 64);
  } else if (type == 3) {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,\r", cmdUnitArr);
    dstEP = 0;
    dstAddr.addrMode = AddrGroup;
    dstAddr.addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[3]);
  } else {
    AT_ERROR(AT_INVALID_PARA);
	  return;
  }

  if ( pbindAddEntry ) {
    // Add the entry into the binding table
    if (!pbindAddEntry( localEP, &dstAddr, dstEP, 1, &cID )) {
      AT_ERROR(AT_FATAL_ERROR);
    } else {
      AT_OK();
      if ( pBindWriteNV ) {
        pBindWriteNV();
      }
    }
  } else {
    AT_ERROR(AT_FATAL_ERROR);
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_BCLR
 *
 * @brief   +BCLR - Clear local Binding Table Entry
 *          Format: AT+BCLR:XX
 *                  XX - The entry number of the binding table which is to be
 *                       cleared
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_BCLR( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  AT_CmdUnit cmdUnitArr[2];

  for ( i = 0; i < 2; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);

  uint8 x = AT_ChartoInt8(&cmdUnitArr[0]);
  if (bindRemoveEntry( &BindingTable[x] )) {
    AT_OK();
    if ( pBindWriteNV ) {
      pBindWriteNV();
    }
  } else {
    AT_ERROR(AT_FATAL_ERROR);
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_BIND
 *
 * @brief   +BIND - Create Binding on Remote Device (ZDO)
 *          Format: AT+BIND:<address>,<type>,<SrcAddress>,<SrcEP>,<ClusterID>,<DstAddress>[,<DstEP>]
 *
 *                  AT+BIND:<address>,1,<SrcAddress>,<SrcEP>,<ClusterID>,<DstAddress>
 *                  AT+BIND:<address>,2,<SrcAddress>,<SrcEP>,<ClusterID>,<DstAddress>[,<DstEP>]
 *
 *                  <address> - the target Node's EUI64, NodeID or Address Table Entry
 *                  <type> - the Address mode shown as blow
 *                    1 = Multicast Binding with Multicast ID Specified
 *                    2 = Unicast Binding with EUI64 and remote EP specified
 *                  <SrcAddress> - The EUI64 of the source
 *                  <SrcEP> - The source endpoint
 *                  <ClusterID> - The Cluster ID on the source device
 *                  <DstAddress> - The EUI64 or 16-bit multicast ID, depending
 *                                  on <type>
 *                  <DstEP> - Only in Mode 2
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_BIND( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[8];
  for ( i = 0; i < 8; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }

  zAddrType_t  *Saddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8  type    = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8  SourceAddr[Z_EXTADDR_LEN];
  uint8  srcEP   = AT_ChartoInt8(&cmdUnitArr[3]);
  uint16 cID     = AT_ChartoInt16(&cmdUnitArr[4]);
  zAddrType_t  dstAddr;
  uint8 dstEP;

  if (type == 2) {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,,,\r", cmdUnitArr);
    dstEP = AT_ChartoInt8(&cmdUnitArr[6]);
    dstAddr.addrMode = Addr64Bit;
    AT_ChartoIntx(&cmdUnitArr[5], dstAddr.addr.extAddr, 64);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,,\r", cmdUnitArr);
    dstEP = 0;
    dstAddr.addrMode = AddrGroup;
    dstAddr.addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[5]);
  }

  AddrMgrEntry_t entry;
  if (cmdUnitArr[0].unitLen == 0) {  // <address>
    Saddr->addr.shortAddr = NLME_GetShortAddr();
    Saddr->addrMode = Addr16Bit;
  } else if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], Saddr->addr.extAddr, 64);
    Saddr->addrMode = Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    Saddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    Saddr->addrMode = Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    Saddr->addr.shortAddr = entry.nwkAddr;
    Saddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  if (cmdUnitArr[2].unitLen == 0) {
    osal_memcpy(SourceAddr, NLME_GetExtAddr(), 8);
  } else {
    AT_ChartoIntx(&cmdUnitArr[2], SourceAddr, 64);
  }

  status = ZDP_BindReq( Saddr, SourceAddr, srcEP, cID, &dstAddr, dstEP, TRUE);
  osal_mem_free(Saddr);

  if (status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_UNBIND
 *
 * @brief   +UNBIND - Delete Binding on Remote Device (ZDO)
 *          Format: AT+UNBIND:<address>,<SrcAddress>,<SrcEP>,<ClusterID>,
 *                          <DstAddress>[,<DstEP>]
 *                  <address> - the target Node's EUI64, NodeID
 *                  <type> - the Address mode shown as blow
 *                    1 = Multicast Binding with Multicast ID Specified
 *                    2 = Unicast Binding with EUI64 and remote EP specified
 *                  <SrcAddress> - The EUI64 of the source
 *                  <SrcEP> - The source endpoint
 *                  <ClusterID> - The Cluster ID on the source device
 *                  <DstAddress> - The EUI64 or 16-bit multicast ID, depending
 *                                  on <type>
 *                  <DstEP> - Only in Mode 2
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_UNBIND( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[8];
  for ( i = 0; i < 8; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }

  zAddrType_t *Saddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  uint8  type    = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8  SourceAddr[Z_EXTADDR_LEN];
  uint8  srcEP   = AT_ChartoInt8(&cmdUnitArr[3]);
  uint16 cID     = AT_ChartoInt16(&cmdUnitArr[4]);
  zAddrType_t  dstAddr;
  uint8 dstEP;

  if (type == 2) {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,,,\r", cmdUnitArr);
    dstEP = AT_ChartoInt8(&cmdUnitArr[6]);
    dstAddr.addrMode = Addr64Bit;
    AT_ChartoIntx(&cmdUnitArr[5], dstAddr.addr.extAddr, 64);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,,\r", cmdUnitArr);
    dstEP = 0;
    dstAddr.addrMode = AddrGroup;
    dstAddr.addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[5]);
  }

  AddrMgrEntry_t entry;
  if (cmdUnitArr[0].unitLen == 0) {  // <address>
    Saddr->addr.shortAddr = NLME_GetShortAddr();
    Saddr->addrMode = Addr16Bit;
  } else if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], Saddr->addr.extAddr, 64);
    Saddr->addrMode = Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    Saddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    Saddr->addrMode = Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    Saddr->addr.shortAddr = entry.nwkAddr;
    Saddr->addrMode = (afAddrMode_t)Addr16Bit;
  }

  if (cmdUnitArr[2].unitLen == 0) {
    osal_memcpy(SourceAddr, NLME_GetExtAddr(), 8);
  } else {
    AT_ChartoIntx(&cmdUnitArr[2], SourceAddr, 64);
  }

  status = ZDP_UnbindReq( Saddr, SourceAddr, srcEP, cID, &dstAddr, dstEP, TRUE);
  osal_mem_free(Saddr);

  if (status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_EBIND
 *
 * @brief   +EBIND - End Device Bind
 *          Format: AT+EBIND:<EP>
 *                  <EP> - Local Endpoint which will initiate end device binding
 *          Use on: Local node
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_EBIND( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[2];

  for ( i = 0; i < 2; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  uint8 ep = AT_ChartoInt8(&cmdUnitArr[0]);
  zAddrType_t addr = {
    {0x0000},
    (afAddrMode_t) Addr16Bit
  };

  endPointDesc_t *epDesc = afFindEndPointDesc(ep);
  if (epDesc) {
    status = ZDP_EndDeviceBindReq(&addr, NLME_GetShortAddr(), ep,
                                  epDesc->simpleDesc->AppProfId,
                                  epDesc->simpleDesc->AppNumInClusters,
                                  epDesc->simpleDesc->pAppInClusterList,
                                  epDesc->simpleDesc->AppNumOutClusters,
                                  epDesc->simpleDesc->pAppOutClusterList,
                                  TRUE);
    if (status != afStatus_SUCCESS) {
      AT_SEND_ERROR(status);
    } else {
      EBindSeq = ZDP_TransID - 1;
      AT_OK();
    }
  } else {
    AT_ERROR(AT_FATAL_ERROR);
  }
}
#endif // REFLECTOR

/*******************************************************************************
 * @fn      AT_Cmd_DASSR
 *
 * @brief   +DASSR - Disassociate Remote Node from PAN
 *          Format: AT+DASSR:<address>
 *                  <address> - a node's EUI64, NodeID
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_DASSR( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[2];

  for ( i = 0; i < 2; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);

  AddrMgrEntry_t entry;
  zAddrType_t *Saddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  if (cmdUnitArr[0].unitLen == 0) {  // <address>
    Saddr->addr.shortAddr = NLME_GetShortAddr();
    Saddr->addrMode = Addr16Bit;
  } else if (cmdUnitArr[0].unitLen > 10) { // <Address> is EUI64
    AT_ChartoIntx(&cmdUnitArr[0], Saddr->addr.extAddr, 64);
    Saddr->addrMode = Addr64Bit;
  } else if (cmdUnitArr[0].unitLen > 2) { // <Address> is NodeID
    Saddr->addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
    Saddr->addrMode = Addr16Bit;
  } else { // <Address> is address table entry
    entry.index = AT_ChartoInt8(&cmdUnitArr[0]);
    // entry.user = ADDRMGR_USER_DEFAULT;
    AddrMgrEntryGet( &entry );
    Saddr->addr.shortAddr = entry.nwkAddr;
    Saddr->addrMode = (afAddrMode_t)Addr16Bit;
  }
  uint8 IEEEAddr[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  status = ZDP_MgmtLeaveReq(Saddr, IEEEAddr, FALSE, FALSE, TRUE);
  osal_mem_free(Saddr);

  if (status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    AT_SEQ(ZDP_TransID);
    AT_SEQ_STROE(ZDP_TransID);
    AT_OK();
  }
}

#if ( ZG_BUILD_COORDINATOR_TYPE )
/*******************************************************************************
 * @fn      AT_Cmd_KEYUPD
 *
 * @brief   +KEYUPD - Update the Network Key
 *          Format: AT+KEYUPD
 *          Use on: Trust Centre
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_KEYUPD( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint16 temp;
  uint8 status;
  AT_CmdUnit cmdUnitArr[1];

  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  uint8 key[SEC_KEY_LEN];
  keySeqNum++;

  for ( i = 0; i < SEC_KEY_LEN; i++) {
    temp = osal_rand();
    key[i++] = LO_UINT16(temp);
    key[i] = HI_UINT16(temp);
  }

  status = ZDSecMgrUpdateNwkKey(key, keySeqNum, NWK_BROADCAST_SHORTADDR_DEVALL);
  if (status == ZSuccess) {
    status = ZDSecMgrSwitchNwkKey(keySeqNum, NWK_BROADCAST_SHORTADDR_DEVALL);
  }
  // TODO: maybe we should send switch network key command for more times
  if (status == ZSuccess) {
    AT_RESP("NWKKEYUPDATED", 13);
    AT_OK();
  } else {
    AT_ZDO_ERROR(status);
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_CCHANGE
 *
 * @brief   +CCHANGE - Change the network's channel
 *          Format: AT+CCHANGE:<channel>
 *          Use on: Network Manager
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_CCHANGE( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[2];

  for ( i = 0; i < 2; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);

  gChannel = AT_ChartoInt8(&cmdUnitArr[0])/16*10+AT_ChartoInt8(&cmdUnitArr[0])%16;
  uint32 channelMask = (uint32)1 << gChannel;
  gNwkUpdateId = _NIB.nwkUpdateId + 1;

  // Build dstAddress
  zAddrType_t *dstAddr = (zAddrType_t *)osal_mem_alloc(sizeof(zAddrType_t));
  dstAddr->addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
  dstAddr->addrMode = (afAddrMode_t)AddrBroadcast;

  status = ZDP_MgmtNwkUpdateReq(dstAddr, channelMask, 0xFE, 0, gNwkUpdateId, 0);
  osal_mem_free(dstAddr);

  if(status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    //send three times ensure network is updated
	osal_start_timerEx( AT_UART_TaskID, AT_NWKUPDATE_EVENT, ZDNWKMGR_BCAST_DELIVERY_TIME+50 );
  }
}
#endif // ZG_BUILD_COORDINATOR_TYPE

/*******************************************************************************
 * @fn      AT_Cmd_RADIOCH
 *
 * @brief   +RADIOCH - Set or get local radio channel
 *          Format: AT+RADIOCH[:<channel>]
 *          Use on: All nodes
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_RADIOCH( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 channel;
  uint8 currChannel;
  uint8 rxOnIdle;
  AT_CmdUnit cmdUnitArr[2];

  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  if (cmdUnitArr[0].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);
    ZMacGetReq( ZMacChannel, &currChannel );
    AT_NEW_LINE();
    printf("channel: %02X", currChannel);
    AT_OK();
  } else {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[1], cmd_ptr, msg_ptr);
    AT_PARSE_CMD_PATTERN_ERROR(":\r", cmdUnitArr);
    channel = AT_ChartoInt8(&cmdUnitArr[0])/16*10+AT_ChartoInt8(&cmdUnitArr[0])%16;;
    ZMacGetReq( ZMacChannel, &currChannel );
    if ( currChannel != channel ) {
      // turn MAC receiver off
      rxOnIdle = false;
      ZMacSetReq( ZMacRxOnIdle, &rxOnIdle );

      // set the NIB channel
      ZMacSetReq( ZMacChannel, &channel );

      // turn MAC receiver back on
      rxOnIdle = true;
      ZMacSetReq( ZMacRxOnIdle, &rxOnIdle );
      _NIB.nwkLogicalChannel = channel;
      // Our Channel has been changed -- notify to save info into NV
      ZDApp_NwkStateUpdateCB();
    }
    AT_OK();
  }
}

/*******************************************************************************
 * @fn      AT_Cmd_ATABLE
 *
 * @brief   +ATABLE - Display Address Table
 *          Format: AT+ATABLE
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ATABLE( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];

  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r", cmdUnitArr);

  AddrMgrEntry_t entry;
  AT_NEW_LINE();
  printf("No. | Active |  ID  | EUI");
  for ( entry.index = 0; entry.index < NWK_MAX_ADDRESSES; entry.index++) {
    entry.user = ADDRMGR_USER_DEFAULT;
    if (!AddrMgrEntryGet( &entry )) {
      entry.user = ADDRMGR_USER_ASSOC;
      if (!AddrMgrEntryGet( &entry )) {
        entry.user = ADDRMGR_USER_ASSOC;
        if (!AddrMgrEntryGet( &entry )) {
          entry.user = ADDRMGR_USER_ASSOC;
          if (!AddrMgrEntryGet( &entry )) {
            entry.user = ADDRMGR_USER_ASSOC;
            if (!AddrMgrEntryGet( &entry ))
              continue;
          }
        }
      }
    }
    AT_NEXT_LINE();
    uint16* ext= (uint16*) entry.extAddr;
    printf("%02X. |  %02X    | %04X | %04X%04X%04X%04X",
           entry.index, entry.user, entry.nwkAddr,
           ext[3],ext[2],ext[1],ext[0]);
  }
  AT_NEW_LINE();
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_ASET
 *
 * @brief   +ASET - Set Address Table Entry, updata not add entry
 *          Format: AT+ASET:<NodeID>,<EUI64>
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ASET( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AddrMgrEntry_t entry;
  uint8 extAddr[Z_EXTADDR_LEN];
  AT_CmdUnit cmdUnitArr[3];

  for ( i = 0; i < 3; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);

  uint8 index; // = AT_ChartoInt8(&cmdUnitArr[0]);
  uint16 nodeId = AT_ChartoInt16(&cmdUnitArr[0]);
  AT_ChartoIntx(&cmdUnitArr[1], extAddr, 64);

  // add entry
  entry.user    = ADDRMGR_USER_DEFAULT;
  entry.nwkAddr = nodeId;
  AddrMgrExtAddrSet( entry.extAddr, extAddr );

  if ( AddrMgrEntryUpdate( &entry ) == TRUE )
  {
    // return successful results
    index   = entry.index;
    status = ZSuccess;
  }
  else
  {
    // return failed results
    index   = entry.index;
    status = ZNwkUnknownDevice;
  }

  if (status == ZSuccess) {
    AT_NEW_LINE();
    printf("EntryStoreIn: %02X", index);
    AT_OK();
  } else {
    AT_ZDO_ERROR(status);
  }
}

#ifdef ZCL_DISCOVER
/*******************************************************************************
 * @fn      AT_Cmd_ATRDISC
 *
 * @brief   +ATRDISC - Find Supported Defined Attributes On A Remote Device
 *          Format: AT+ATRDISC:<NodeID>,<EP>,<ClusterID>,<AttributeID>,
 *                  <MaxNumofAttr>[,<ProfileID>]
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_ATRDISC( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  uint16 nodeId;
  uint8 ep;
  uint16 cID;
  uint16 startAttrId;
  uint8 numofAttr;
  // uint16 profileID;
  AT_CmdUnit cmdUnitArr[7];

  for ( i = 0; i < 7; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  if (cmdUnitArr[5].symbol == '\r') {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,\r", cmdUnitArr);
  } else {
    AT_PARSE_CMD_PATTERN_ERROR(":,,,,,\r", cmdUnitArr);
    // profileID = AT_ChartoInt16(&cmdUnitArr[5]);
  }

  nodeId = AT_ChartoInt16(&cmdUnitArr[0]);
  ep = AT_ChartoInt8(&cmdUnitArr[1]);
  cID = AT_ChartoInt16(&cmdUnitArr[2]);
  startAttrId = AT_ChartoInt16(&cmdUnitArr[3]);
  numofAttr = AT_ChartoInt8(&cmdUnitArr[4])/16*10+AT_ChartoInt8(&cmdUnitArr[4])%16;

  //build destination address
  afAddrType_t dstAddr;
  dstAddr.endPoint = ep;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.addr.shortAddr = nodeId;

  zclDiscoverAttrsCmd_t discCmd;
  discCmd.startAttr  = startAttrId;
  discCmd.maxAttrIDs = numofAttr;

  status = zcl_SendDiscoverAttrsCmd( ZCL_CONTROLLER_ENDPOINT, &dstAddr, cID,
                            &discCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter() );

  if (status != afStatus_SUCCESS) {
    AT_SEND_ERROR(status);
  } else {
    AT_OK();
  }
}
#endif // ZCL_DISCOVER

#ifdef ZCL_READ
/*******************************************************************************
 * @fn      AT_Cmd_ReadAttr
 *
 * @brief   AT+READATTR - Read attribute data
 *                    AT+READATTR:<Addr>,<EP>,<SendMode>,<ClusterID>,<AttrID>,...,<AttrID>
 *                            <Addr> - local/remote device's address
 *                            <EP> - 8 bit hexadecimal number Endpoint
 *                            <SendMode> - transmission mode
 *                                          0 - unicast   1 - group addressing
 *                            <ClusterID> - uint16 number represents the cluster id
 *                            <AttrID> - uint16 number represents the attribute id
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_ReadAttr( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[20];
  uint8 i;

  uint8 parameterN = 1;
  char pattern[21];
  for(i = 0; i < 20; i++){
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    if(cmdUnitArr[i].symbol == ',') parameterN++;
  }

  #if AT_CMD_PATTERN_CHECK
  //built check pattern
  pattern[0] = ':';
  for(i = 1; i < 5; i++) pattern[i] = ',';
  for(i = 5; i < parameterN; i++) pattern[i] = ',';
  pattern[i] = '\r';
  pattern[i+1] = '\0';
  #endif

  //check pattern
  AT_PARSE_CMD_PATTERN_ERROR(pattern,cmdUnitArr);

  uint8 endpoint = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8 sendmode = AT_ChartoInt8(&cmdUnitArr[2]);

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.endPoint = endpoint;
  dstAddr.addrMode = (sendmode == 0 ? (afAddrMode_t)Addr16Bit : (afAddrMode_t) AddrGroup);
  if (cmdUnitArr[0].unitLen == 0) {
    dstAddr.addr.shortAddr = NLME_GetShortAddr();
  } else {
    dstAddr.addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
  }

  // build ZCL readCmd
  zclReadCmd_t* readCmd = (zclReadCmd_t*) osal_mem_alloc(sizeof(int)+2*(parameterN-4));
  readCmd->numAttr = parameterN-4;
  for(i = 0; i < parameterN-4; i++)
      readCmd->attrID[i] = AT_ChartoInt16(&cmdUnitArr[4+i]);

  //send zcl read
  uint8 status;
  status = zcl_SendRead( ZCL_CONTROLLER_ENDPOINT, &dstAddr,
                               AT_ChartoInt16(&cmdUnitArr[3]), readCmd,
                               ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter() );
  osal_mem_free(readCmd);
  if(status!=afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*******************************************************************************
 * @fn      AT_Cmd_WriteAttr
 *
 * @brief   AT+WRITEATTR - Write attribute data
 *                    AT+WRITEATTR:<Addr>,[<EP>],<S/C>,[Mcode],<ClusterID>,
 *                     <AttrID>,<DataType>,<Data>,...,<AttrID>,<DataType>,<Data>
 *                            <Addr> - local/remote device's address
 *                            <EP> - 8 bit hexadecimal number Endpoint
 *                            <ClusterID> - uint16 number represents the cluster id
 *                            <AttrID> - uint16 number represents the attribute id
 *                            <DataType> - 8 bit hexadecimal number that represents the type of the
 *                                        data accepted by this Attribute (please check HA specification)
 *                            <Data> - If attribute value has an integer type this field shall contain
 *                                        hexadecimal representation in big-endian format. If attribute
 *                                        value has a string type this field contains sequence of characters.
 *                    Note: This command can be used to write multiple attributes (up to 5 in a cluster)
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_WriteAttr( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[20];
  uint8 i;
  uint16 address;
  uint8 dataType;
  uint8 parameterN = 1;
  char pattern[21];
  for(i = 0; i < 20; i++){
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    if(cmdUnitArr[i].symbol == ',') parameterN++;
  }

  #if AT_CMD_PATTERN_CHECK
  //built check pattern
  pattern[0] = ':';
  for(i = 1; i < parameterN; i++) pattern[i] = ',';
  pattern[i] = '\r';
  pattern[i+1] = '\0';
  if((parameterN-5)%3 != 0) {
    AT_ERROR(AT_LACK_PARA);
    return;
  }
  #endif

  //check pattern
  AT_PARSE_CMD_PATTERN_ERROR( pattern, cmdUnitArr );

  if(cmdUnitArr[0].unitLen==0){
    address = NLME_GetShortAddr();
  }else{
    address = AT_ChartoInt16(&cmdUnitArr[0]);
  }
  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addr.shortAddr = address;
  if (cmdUnitArr[1].unitLen != 0) {
    dstAddr.endPoint = AT_ChartoInt8(&cmdUnitArr[1]);
    if ((address == NWK_BROADCAST_SHORTADDR_DEVALL)
          || (address == NWK_BROADCAST_SHORTADDR_DEVZCZR)
          || (address == NWK_BROADCAST_SHORTADDR_DEVRXON)) {
        dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    } else {
        dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    }
  } else {
    dstAddr.endPoint = ZCL_CONTROLLER_ENDPOINT;
    dstAddr.addrMode = (afAddrMode_t)AddrGroup;
  }

  uint8 numAttr = (parameterN-5)/3;

  //build ZCL writeCmd
  zclWriteCmd_t* writeCmd;
  writeCmd = (zclWriteCmd_t*)osal_mem_alloc( sizeof(zclWriteCmd_t) + sizeof(zclWriteRec_t)*numAttr );
  writeCmd->numAttr = numAttr;
  for ( i = 0; i < numAttr; i++) {
    writeCmd->attrList[i].attrID = AT_ChartoInt16(&cmdUnitArr[i*3+5]);
    dataType = AT_ChartoInt16(&cmdUnitArr[i*3+6]);
    writeCmd->attrList[i].dataType = dataType;
    if(dataType == ZCL_DATATYPE_CHAR_STR) {
      cmdUnitArr[i*3+7].unit--;
      *(cmdUnitArr[i*3+7].unit) = cmdUnitArr[i*3+7].unitLen;
      writeCmd->attrList[i].attrData = cmdUnitArr[i*3+7].unit;
    } else if (dataType == ZCL_DATATYPE_DATA16||
              dataType == ZCL_DATATYPE_UINT16||
              dataType == ZCL_DATATYPE_INT16){
       *(uint16*)cmdUnitArr[i*3+7].unit = AT_ChartoInt16(&cmdUnitArr[i*3+7]);
       writeCmd->attrList[i].attrData = cmdUnitArr[i*3+7].unit;
    } else {
       *(uint8*)cmdUnitArr[i*3+7].unit = AT_ChartoInt8(&cmdUnitArr[i*3+7]);
       writeCmd->attrList[i].attrData = cmdUnitArr[i*3+7].unit;
    }
  }

  //send zcl write
  uint8 status;
  status = zcl_SendWrite( ZCL_CONTROLLER_ENDPOINT, &dstAddr,
                               AT_ChartoInt16(&cmdUnitArr[4]), writeCmd,
                               ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter() );
  osal_mem_free ( writeCmd );
  if(status!=afStatus_SUCCESS) AT_SEND_ERROR(status);
  else AT_OK();
}
#endif // ZCL_WRITE

#ifdef ZCL_REPORT_CONFIGURING_DEVICE
/*******************************************************************************
 * @fn      AT_Cmd_READRCFG
 *
 * @brief   +READRCFG - Read Reportiong Configuration From Remote Node
 *          Format: AT+READRCFG:<Address>,[<EP>],<ClusterID>,
 *                  <Direction1>,<AttrID1>,-,<Direction10>,<AttrID10>
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_READRCFG( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[23];
  uint8 i;
  uint8 status;
  uint16 address;
  uint8 parameterN = 1;
  char pattern[24];
  for(i = 0; i < 23; i++){
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    if(cmdUnitArr[i].symbol == ',') parameterN++;
  }

  #if AT_CMD_PATTERN_CHECK
  //built check pattern
  pattern[0] = ':';
  for(i = 1; i < parameterN; i++) pattern[i] = ',';
  pattern[i] = '\r';
  pattern[i+1] = '\0';
  if((parameterN-3)%2 != 0) {
    AT_ERROR(AT_LACK_PARA);
    return;
  }
  #endif

  //check pattern
  AT_PARSE_CMD_PATTERN_ERROR( pattern, cmdUnitArr );

  address = AT_ChartoInt16(&cmdUnitArr[0]);

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addr.shortAddr = address;
  if (cmdUnitArr[1].unitLen != 0) {
    dstAddr.endPoint = AT_ChartoInt8(&cmdUnitArr[1]);
    if ((address == NWK_BROADCAST_SHORTADDR_DEVALL)
          || (address == NWK_BROADCAST_SHORTADDR_DEVZCZR)
          || (address == NWK_BROADCAST_SHORTADDR_DEVRXON)) {
        dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    } else {
        dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    }
  } else {
    dstAddr.endPoint = ZCL_CONTROLLER_ENDPOINT;
    dstAddr.addrMode = (afAddrMode_t)AddrGroup;
  }

  // build zclReadReportCfgCmd
  zclReadReportCfgCmd_t* readReportCfgCmd = (zclReadReportCfgCmd_t*) osal_mem_alloc(
                    sizeof(uint8)+sizeof(zclReadReportCfgRec_t)*(parameterN-3));
  readReportCfgCmd->numAttr = (parameterN-3)/2;
  for(i = 0; i < (parameterN-3)/2; i++) {
    readReportCfgCmd->attrList[i].direction = AT_ChartoInt8(&cmdUnitArr[3+i*2]);
    readReportCfgCmd->attrList[i].attrID = AT_ChartoInt16(&cmdUnitArr[3+i*2+1]);
  }

  // Send Read Reporting Configuration command
  status = zcl_SendReadReportCfgCmd( ZCL_CONTROLLER_ENDPOINT, &dstAddr,
                               AT_ChartoInt16(&cmdUnitArr[2]), readReportCfgCmd,
                               ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter() );

  osal_mem_free(readReportCfgCmd);
  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_CFGRPT
 *
 * @brief   +CFGRPT - Configure Attribute Reporting
 *          Format: AT+CFGRPT:<Address>,[<EP>],<ClusterID>,
 *                  <Direction>,<AttrID>,[<DataType>,<MinimumReportionInterval>,
 *                  <MaximumReporingInterval>,<ReportableChange>][Timeout]
 *
 *                  AT+CFGRPT:<Address>,[<EP>],<ClusterID>,0,<AttrID>,[<DataType>,<MinimumReportionInterval>,<MaximumReporingInterval>,<ReportableChange>]
 *                  AT+CFGRPT:<Address>,[<EP>],<ClusterID>,1,<AttrID>,[Timeout]
 *
 *          Use on: All Devices
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_CFGRPT( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 i;
  uint8 status;
  AT_CmdUnit cmdUnitArr[10];
  uint16 address;
  uint8 direction;
  zclCfgReportCmd_t* cfgReportCmd;

  for(i = 0; i < 5; i++){
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR( ":,,,,", cmdUnitArr );

  address = AT_ChartoInt16(&cmdUnitArr[0]);
  // build destination address
  afAddrType_t dstAddr;
  dstAddr.addr.shortAddr = address;
  if (cmdUnitArr[1].unitLen != 0) {
    dstAddr.endPoint = AT_ChartoInt8(&cmdUnitArr[1]);
    if ((address == NWK_BROADCAST_SHORTADDR_DEVALL)
          || (address == NWK_BROADCAST_SHORTADDR_DEVZCZR)
          || (address == NWK_BROADCAST_SHORTADDR_DEVRXON)) {
        dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    } else {
        dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    }
  } else {
    dstAddr.endPoint = ZCL_CONTROLLER_ENDPOINT;
    dstAddr.addrMode = (afAddrMode_t)AddrGroup;
  }

  direction = AT_ChartoInt8(&cmdUnitArr[3]);
  if (direction == 0) {
    for(i = 5; i < 10; i++){
      cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    }
    AT_PARSE_CMD_PATTERN_ERROR( ":,,,,,,,,\r", cmdUnitArr );

    // build zclCfgReportCmd
    cfgReportCmd = (zclCfgReportCmd_t*) osal_mem_alloc(
                                      sizeof(uint8)+sizeof(zclCfgReportRec_t));
    cfgReportCmd->numAttr = 1;
    cfgReportCmd->attrList[0].direction = direction;
    cfgReportCmd->attrList[0].attrID = AT_ChartoInt16(&cmdUnitArr[4]);
    cfgReportCmd->attrList[0].dataType = AT_ChartoInt8(&cmdUnitArr[5]);
    cfgReportCmd->attrList[0].minReportInt = AT_ChartoInt16(&cmdUnitArr[6]);
    cfgReportCmd->attrList[0].maxReportInt = AT_ChartoInt16(&cmdUnitArr[7]);
    if ( zclAnalogDataType( cfgReportCmd->attrList[0].dataType ) )
    {
      cfgReportCmd->attrList[0].reportableChange = AT_AttrData_ChartoInt(
                          cfgReportCmd->attrList[0].dataType, &cmdUnitArr[8] );
    }
  } else {
    for(i = 5; i < 7; i++){
      cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
    }
    AT_PARSE_CMD_PATTERN_ERROR( ":,,,,,\r", cmdUnitArr );

    // build zclCfgReportCmd
    cfgReportCmd = (zclCfgReportCmd_t*) osal_mem_alloc(
                                      sizeof(uint8)+sizeof(zclCfgReportRec_t));
    cfgReportCmd->numAttr = 1;
    cfgReportCmd->attrList[0].direction = direction;
    cfgReportCmd->attrList[0].timeoutPeriod = AT_ChartoInt16(&cmdUnitArr[5]);
  }

  // Send a Configure Reporting command
  status = zcl_SendConfigReportCmd( ZCL_CONTROLLER_ENDPOINT, &dstAddr,
                               AT_ChartoInt16(&cmdUnitArr[2]), cfgReportCmd,
                               ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter() );

  osal_mem_free(cfgReportCmd);
  if(status != afStatus_SUCCESS)
    AT_SEND_ERROR(status);
  else
    AT_OK();
}
#endif // ZCL_REPORT_CONFIGURING_DEVICE

/*******************************************************************************
 * @fn      AT_Cmd_Help
 *
 * @brief   AT+HELP - show all the AT commands
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ******************************************************************************/
void AT_Cmd_Help( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

  uint8 i;
  AT_NEW_LINE();
  AT_RESP("ATI", 3);
  for(i = 0; i < AT_CMD_HELP_DESC_OFFSET-3; i++)
    AT_RESP(".", 1);
  AT_RESP("Display Product Identification Information", sizeof("Display Product Identification Information")-1);
  AT_NEXT_LINE();

  AT_RESP("ATZ",3);
  for(i = 0; i < AT_CMD_HELP_DESC_OFFSET-3; i++)
    AT_RESP(".", 1);
  AT_RESP("Software Reset", sizeof("Software Reset")-1);
  AT_NEXT_LINE();

  AT_RESP("ATF", 3);
  for(i = 0; i < AT_CMD_HELP_DESC_OFFSET-3; i++)
    AT_RESP(".", 1);
  AT_RESP("Restore Local Device's Factory Defaults", sizeof("Restore Local Device's Factory Defaults")-1);

  for(i = 0; i < AT_CMD_SZ; i++) {
    // uint8 j;
    AT_NEXT_LINE();
    AT_RESP("AT+", 3);
    AT_RESP(AT_Cmd_Arr[i].AT_Cmd_str, strlen(AT_Cmd_Arr[i].AT_Cmd_str));
    //for(j = 0; j < AT_CMD_HELP_DESC_OFFSET-strlen(AT_Cmd_Arr[i].AT_Cmd_str)-3; j++)
    //  AT_RESP(".",1);
    AT_RESP(".................", AT_CMD_HELP_DESC_OFFSET-getLength((uint8 *)AT_Cmd_Arr[i].AT_Cmd_str, 1)-3);
    // AT_RESP(AT_Cmd_Arr[i].ATCmdDescription, strlen(AT_Cmd_Arr[i].ATCmdDescription));
    printf(AT_Cmd_Arr[i].ATCmdDescription);
  }
  AT_NEW_LINE();
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_GetAddr
 *
 * @brief   AT+GETADDR - show self short address and IEEE address
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_GetAddr( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[1];
  cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[0], cmd_ptr, msg_ptr);
  AT_PARSE_CMD_PATTERN_ERROR("\r",cmdUnitArr);

  char addrbuff[20] = {0};
  sprintf(addrbuff, "SelfshortAddr:%04X", NLME_GetShortAddr());
  AT_NEW_LINE();
  AT_RESP(addrbuff, strlen(addrbuff));
  AT_NEW_LINE();
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_EpCtrl
 *
 * @brief   AT+EPCTRL - Control the local endpoint
 *                      AT+EPCTRL:<Enable/Disable>,<EP>
 *                            <Enable/Disable> - 0 for Disable; 1 for Enable
 *                            <EP> - 8 bit hexadecimal number Endpoint
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_EpCtrl( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[3];
  uint8 i;
  for ( i = 0; i < 3; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);

  uint8 isEnable = AT_ChartoInt8(&cmdUnitArr[0]);
  uint8 ep = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8 str[2];
  if (isEnable != 0) isEnable = 1;

  if (isEnable == 0 || isEnable == 1) {
    if (isEnable) {
      AT_NEW_LINE();
      AT_RESP("ENABLED:", sizeof("ENABLED:")-1);
      AT_Int8toChar(ep, str);
      AT_RESP(str, 2);
      AT_NEW_LINE();
    } else {
      AT_NEW_LINE();
      AT_RESP("DISABLED:", sizeof("DISABLED:")-1);
      AT_Int8toChar(ep, str);
      AT_RESP(str, 2);
      AT_NEW_LINE();
    }
  } else {
    AT_NEW_LINE();
    AT_RESP("UNKNOWNEP ERROR", sizeof("UNKNOWNEP ERROR")-1);
    AT_NEW_LINE();
    return;
  }
  AT_OK();
}

/*******************************************************************************
 * @fn      AT_Cmd_REpCtrl
 *
 * @brief   AT+REPCTRL - Control the remote endpoint
 *                      AT+REPCTRL:<shortAddr>,<Enable/Disable>,<EP>
 *                            <shortAddr> - remote device's short address
 *                            <Enable/Disable> - 0 for Disable; 1 for Enable
 *                            <EP> - 8 bit hexadecimal number Endpoint
 *
 * @param   uint8 cmd_ptr  - the point show the place we start scan the command
 * @param   uint8* msg_ptr - the message pointer
 *
 * @return  None
 ***************************************************************************************************/
void AT_Cmd_REpCtrl( uint8 cmd_ptr, uint8* msg_ptr )
{
  uint8 status;

  AT_CmdUnit cmdUnitArr[4];
  uint8 i;
  for ( i = 0; i < 4; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,,\r", cmdUnitArr);

  uint8 isEnable = AT_ChartoInt8(&cmdUnitArr[1]);
  uint8 ep = AT_ChartoInt8(&cmdUnitArr[2]);
  if (isEnable != 0) isEnable = 1;

  // build destination address
  afAddrType_t dstAddr;
  dstAddr.endPoint = ATApp_ENDPOINT;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  if (cmdUnitArr[0].unitLen == 0) {
    dstAddr.addr.shortAddr = NLME_GetShortAddr();
  } else {
    dstAddr.addr.shortAddr = AT_ChartoInt16(&cmdUnitArr[0]);
  }

  // buile endPoint control command
  EPCtrlCmd_t epCtrlCmd;
  epCtrlCmd.ep = ep;
  epCtrlCmd.isEnable = isEnable;

  status = AT_SendEPCtrl(ATApp_ENDPOINT, &dstAddr, ATApp_GENERIC_CLUSTER, &epCtrlCmd);
  if(status!=afStatus_SUCCESS) AT_SEND_ERROR(status);
  else AT_OK();
}

void AT_Cmd_Test( uint8 cmd_ptr, uint8* msg_ptr )
{
  AT_CmdUnit cmdUnitArr[3];
  uint8 i;
  for ( i = 0; i < 3; i++) {
    cmd_ptr = AT_get_next_cmdUnit(&cmdUnitArr[i], cmd_ptr, msg_ptr);
  }
  AT_PARSE_CMD_PATTERN_ERROR(":,\r", cmdUnitArr);

  AT_NEW_LINE();
  int8 sign = (AT_ChartoInt8(&cmdUnitArr[0]) == 1) ? 1 : -1;
  uint8 data = AT_ChartoInt8(&cmdUnitArr[1])/16*10+AT_ChartoInt8(&cmdUnitArr[1])%16;
  AT_RESP("Result:", sizeof("Result:")-1);
  data = sign * data;
  if ((int8)data >= 0) {
    AT_RESP("+", 1);
    printf("%02d,", data);
  } else {
    AT_RESP("-", 1);
    printf("%02d,", ((int8)data)*-1);
  }
  AT_NEW_LINE();
  AT_OK();
}

/******************************************************************************
 ******************************************************************************/
