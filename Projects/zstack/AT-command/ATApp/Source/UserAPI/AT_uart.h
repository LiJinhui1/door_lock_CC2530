/*********************************************************************
  Filename:       AT_uart.h

  Author:         Yasin Zhang
*********************************************************************/

#ifndef AT_UART_H
#define AT_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Onboard.h"
#include "OSAL.h"
#include "MT.h"
#include "AT_uart0.h"

/*****************************************
                 config
****************************************/
/*********************
 *  compile options
 */
#define AT_FCS_VERIFY         FALSE
#define AT_DEBUG_INF_SHOW     FALSE
#define AT_CMD_PATTERN_CHECK  TRUE
#define AT_UART_BACKSPACE     TRUE
#define AT_SHOW_STATE_CHANGE  TRUE

/*********************
 *  UART configuration
 */
#define AT_UART_PORT        HAL_UART_PORT_0
//#define AT_UART_BR        HAL_UART_BR_9600
//#define AT_UART_BR        HAL_UART_BR_19200
//#define AT_UART_BR        HAL_UART_BR_38400
//#define AT_UART_BR        HAL_UART_BR_57600
#define AT_UART_BR          HAL_UART_BR_115200

#define AT_UART_RX_BUFF_MAX     100
#define AT_UART_TX_BUFF_MAX     100
#define AT_CMD_BUFF_MAX         100

/*********************
 *  error message
 */
#define AT_NO_ERROR          0x00      // Everything OK - Success
#define AT_FATAL_ERROR       0x01      // Fatal Error
#define AT_UNKNOWN_CMD       0x02      // Unknown command
#define AT_LACK_CMD          0x03      // lack command
#define AT_INVALID_S         0x04      // Invalid S-Register
#define AT_INVALID_PARA      0x05      // Invalid parameter
#define AT_LACK_OPERATOR     0x22
#define AT_LACK_PARA         0x24
#define AT_FORM_NWK_FAIL     0x25      // Cannot form network
#define AT_NO_NETWORK        0x27      // Network not found
#define AT_OPERATION_INVALID 0x70      // Invalid operation
#define AT_CANNOT_JOIN       0x94      // Cannot join network
#define AT_NWK_PRESENT       0xC5      // NWK already present
#define AT_NWK_TAB_FULL      0xC7      // NWK table full
#define AT_NWK_UNKNOWN_DEV   0xC8      // NWK unknown device

/*********************
 *  command constants
 */
#define AT_CMD_HELP_DESC_OFFSET   17

/********************************************************
 *               define UART message sent
*******************************************************/
#define AT_RESP(str, len)     AT_Uart0_Send((uint8 *)str, len)
#define AT_OK()               AT_RESP("\r\nOK\r\n", sizeof("\r\nOK\r\n")-1)
#define AT_SEQ(x)             printf("\r\nSEQ:%02X\r\n", x-1)
#define AT_NEXT_LINE()        AT_RESP("\r\n", 2)
#define AT_NEW_LINE()         AT_RESP("\r\n", 2)
#define AT_ERROR(x)           AT_UARTWriteErrMsg(x,1)
#define AT_SEND_ERROR(x)      AT_UARTWriteErrMsg(x,2)
#define AT_ZDO_ERROR(x)       AT_UARTWriteErrMsg(x,3)

#if AT_DEBUG_INF_SHOW
#define AT_DEBUG(str, len)    AT_RESP((uint8 *) str, len)
#else
#define AT_DEBUG(str, len)
#endif

#if AT_CMD_PATTERN_CHECK
#define AT_PARSE_CMD_PATTERN_ERROR(x,y)      \
  {uint8 err;                                \
  err = AT_Pattern_Check(x,y);               \
    if (err != 0) { AT_ERROR(err); return;} }
#else
#define AT_PARSE_CMD_PATTERN_ERROR(x,y)
#endif

#if AT_CMD_PATTERN_CHECK
#define AT_PARSE_SINGLE_CMD_PATTERN_ERROR(x,y)      \
  { if (x[0] != y->symbol) {AT_ERROR(AT_INVALID_PARA); return;}}
#else
#define AT_PARSE_CMD_PATTERN_ERROR(x,y)
#endif

/*********************************************************************
 * DATA STRUCTURE
 */
typedef struct {
  uint8 symbol;
  uint8 unitLen;
  uint8* unit;
} AT_CmdUnit;

typedef void (*AT_CmdFn_t)(uint8 cmd_ptr, uint8* msg_ptr);
typedef struct {
  char* AT_Cmd_str;
  AT_CmdFn_t AT_CmdFn;
  char* ATCmdDescription;
} AT_Cmd_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern bool notdoFNCmd;
extern bool notdoJNCmd;
extern bool notdoSCANCmd;
extern bool notdoEUIREQcmd;
extern bool notdoSIMPLEDESCcmd;
extern bool notdoNODEDESEcmd;

extern uint8 keySeqNum;
#if AT_SHOW_STATE_CHANGE
extern const uint8* devStates_str[];
#endif

/*********************************************************************
 * FUNCTIONS
 */
extern uint8 AT_UART_Init( byte task_id );   // Task Initialization for the UART Application
extern void AT_HandleCMD( uint8 *msg_ptr ); // Command Handler
extern uint16 AT_HalUARTWrite( uint8 port, uint8 *buf, uint16 len );
extern void AT_UARTWriteErrMsg( uint8 errCode, uint8 fn );

/*****************************
 * Tool functions
 */
void AT_GetIEEEAddrStr( uint8* pIeeeAddr, uint8* pStr );
void AT_Int8toChar( uint8 pbyte, uint8* str );
void AT_Int16toChar( uint16 pWord, uint8* str );
void AT_Int32toChar( uint32 pDWord, uint8* str );
void AT_IntxtoChar( uint8* data, uint8 *pStr, uint8 x );
uint8 AT_ChartoInt8( AT_CmdUnit *cmdUnit );
uint16 AT_ChartoInt16( AT_CmdUnit *cmdUnit );
uint32 AT_ChartoInt32( AT_CmdUnit *cmdUnit );
void AT_ChartoIntx( AT_CmdUnit *cmdUnit,uint8 *pHex, uint8 x );

/*********************************
 *  AT Command process funcitons
 */
void AT_Cmd_ATI( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_ATZ( uint8 cmd_ptr, uint8* msg );
void AT_Cmd_ATF( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_BLoad( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_Time( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_RawZCL( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_RawZDO( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_InterPAN( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_Scan( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_FN( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_JN( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_LN( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_N( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_PJ( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_RJ( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_KeyTab( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_AddKey( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_IDREQ( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_EUIREQ( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_NODEDESC( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_POWERDESC( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_ACTEPDESC( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_SIMPLEDESC( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_MATCHREQ( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_ANNCE( uint8 cmd_ptr, uint8* msg_ptr );
#if ( ZG_BUILD_RTR_TYPE )
void AT_Cmd_PANNCE( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_NTABLE( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_RTABLE( uint8 cmd_ptr, uint8* msg_ptr );
#endif // ZG_BUILD_RTR_TYPE
#if defined ( REFLECTOR )
#if defined ( ZDO_MGMT_BIND_RESPONSE )
void AT_Cmd_BTABLE( uint8 cmd_ptr, uint8* msg_ptr );
#endif // ZDO_MGMT_BIND_RESPONSE
void AT_Cmd_BSET( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_BCLR( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_BIND( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_UNBIND( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_EBIND( uint8 cmd_ptr, uint8* msg_ptr );
#endif // REFLECTOR
void AT_Cmd_DASSR( uint8 cmd_ptr, uint8* msg_ptr );
#if ( ZG_BUILD_COORDINATOR_TYPE )
void AT_Cmd_KEYUPD( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_CCHANGE( uint8 cmd_ptr, uint8* msg_ptr );
#endif
void AT_Cmd_RADIOCH( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_ATABLE( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_ASET( uint8 cmd_ptr, uint8* msg_ptr );
#ifdef ZCL_DISCOVER
void AT_Cmd_ATRDISC( uint8 cmd_ptr, uint8* msg_ptr );
#endif
#ifdef ZCL_REPORT_CONFIGURING_DEVICE
void AT_Cmd_READRCFG( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_CFGRPT( uint8 cmd_ptr, uint8* msg_ptr );
#endif

void AT_Cmd_Help( uint8 cmd_ptr, uint8* msg );
void AT_Cmd_GetAddr( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_EpCtrl( uint8 cmd_ptr, uint8* msg_ptr );
void AT_Cmd_REpCtrl( uint8 cmd_ptr, uint8* msg_ptr );
#ifdef ZCL_READ
void AT_Cmd_ReadAttr( uint8 cmd_ptr, uint8* msg_ptr );
#endif
#ifdef ZCL_WRITE
void AT_Cmd_WriteAttr( uint8 cmd_ptr, uint8* msg_ptr );
#endif
void AT_Cmd_Test( uint8 cmd_ptr, uint8* msg_ptr );

#ifdef __cplusplus
}
#endif

#endif /* AT_UART_H */
