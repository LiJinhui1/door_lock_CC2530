/*********************************************************************
  Filename:       AT_doorlock.h
  Author:         Yang Wang
*********************************************************************/
#ifndef AT_DOORLOCK_H
#define AT_DOORLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_defs.h"
#include "hal_mcu.h"
#include "zcl_closures.h"

#define DOORLOCK_RSP_IGN        0xFF // byte to be ignored
#define DOORLOCK_RSP_SOF        0x5A // start-of-frame byte

#define DOORLOCK_STATE_SOP1     0x00 // Start-of-packet//0xFF
#define DOORLOCK_STATE_SOP2     0x01 // Start-of-packet//0x5A
#define DOORLOCK_STATE_LEN      0x02
#define DOORLOCK_STATE_DATA     0x03
#define DOORLOCK_STATE_SUMCHK   0x04

#define BYTE_SYNC               0x5A

#define PIN_LEN_NULL            0
#define PIN_LEN_MIN             6
#define PIN_LEN_MAX             12

// bit mask for Status of command sent to doorlock
#define CHECK_PIN_A_U_T         BV(0) // 0: check only administrator/temp, 1: check administrator/user/temp
#define CHECK_SET_TIME          BV(1) // 0: do not check and set time, 1: check and set time
#define CHECK_PIN_Y_N           BV(2) // 0: check pin, 1: do not check pin

#define UNLOCK_WITHOUT_TIMEOUT  0x00 // The DoorLock will keep unlocked
#define UNLOCK_WITH_TIMEOUT     0x01 // The DoorLock will unlock itself, but relock after timeout (s)

#define DOORLOCK_RELOCK_TIMEOUT 5000//ms

#define LEN_DOORLOCK_LOCK                       23
#define LEN_DOORLOCK_UNLOCK                     27
#define LEN_DOORLOCK_LOG_RECORD                 0x11
#define LEN_DOORLOCK_SET_TEMPORARY_PIN          40
#define LEN_DOORLOCK_MODIFY_PIN                 0x28

// specific command supported by manufacturer
#define CMD_DOORLOCK_UNLOCK                     0x01
#define CMD_DOORLOCK_LOCK                       0x02
#define CMD_DOORLOCK_SET_TEMPORARY_PIN          0x04
#define CMD_DOORLOCK_MODIFY_PIN                 0x05
#define CMD_DOORLOCK_GET_LOG_RECORD             0x09
#define CMD_DOORLOCK_EVT_UPLOAD                 0x0A
#define CMD_DOORLOCK_KEY_SHORT_PRESS            0xC0
#define CMD_DOORLOCK_RESET_TO_FACT_NEW          0xC3
#define CMD_DOORLOCK_WAKE_UP                    0xCE
#define CMD_DOORLOCK_MATCH                      0xCF
#define CMD_DOORLOCK_RESET_TO_FACT_NEW_ALL      0xE0

// specific event supported by manufacturer
#define EVT_DOORLOCK_UPLOAD_FORCE_PRY_LOCK      0x01
#define EVT_DOORLOCK_UPLOAD_ENTER_LOCKED_STATE  0x02
#define EVT_DOORLOCK_UPLOAD_LOW_POWER           0x03
#define EVT_DOORLOCK_UPLOAD_UNLOCK              0x04
#define EVT_DOORLOCK_UPLOAD_ADD_USER            0x08
#define EVT_DOORLOCK_UPLOAD_DELETE_USER         0x09
#define EVT_DOORLOCK_UPLOAD_DOORBELL            0x0D
#define EVT_DOORLOCK_UPLOAD_EXIT_LOCKED_STATE   0x15

// specific unlock type supported by manufacturer
#define WITH_FINGERPRINT                        BV(0)
#define WITH_KEYPAD_CODE                        BV(1)
#define WITH_RFID_CARD                          BV(2)
#define WITH_REMOTE_CONTROL                     BV(3)
#define WITH_MANUAL_KEY                         BV(5)
#define WITH_TEMP_PIN                           BV(6)
#define WITH_REMOTE_APP                         BV(7)

#define UNLOCK_SUCCESS                          0x01
#define UNLOCK_FAILURE                          0x02

#define SET_TEMPORARY_PIN_SUCCESS               0x01
#define SET_TEMPORARY_PIN_FAILED                0x02
#define SET_TEMPORARY_PIN_SAME_PIN              0x07

#define MODIFY_PIN_SUCCESS                      0x01
#define MODIFY_PIN_FAILED                       0x02
#define MODIFY_PIN_SAME_PIN                     0x07  

#define EVENT_FLAG_OPERATION                    0x01
#define EVENT_FLAG_PROGRAMMING                  0x02
#define EVENT_FLAG_UNKNOWN                      0xFF

void AT_DoorLock_Init(void);

ZStatus_t AT_DoorLock_Unlock( zclDoorLock_t *pInCmd );
ZStatus_t AT_DoorLock_Lock( zclDoorLock_t *pInCmd );
ZStatus_t AT_DoorLock_SetTemporaryPin_Req( zclDoorLockSetTemporaryPin_t *pCmd );

ZStatus_t AT_DoorLock_Handle_Rsp(uint8 *buf, uint8 len);

#ifdef __cplusplus
}
#endif

#endif /* AT_DOORLOCK_H */
