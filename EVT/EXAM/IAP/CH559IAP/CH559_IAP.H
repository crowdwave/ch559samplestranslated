#ifndef __CH559_IAP_H__
#define __CH559_IAP_H__
#define Uart_Sync_Head1 0x57
#define Uart_Sync_Head2 0xab
/*The following is the definition of IAP download command*/
#define CMD_IAP_PROM 0x80 // ISP programming command
#define CMD_IAP_ERASE 0x81 // ISP erase command
#define CMD_IAP_VERIFY 0x82 // ISP verification command
#define CMD_IAP_END 0x83 // ISP end flag
/************************************************* *******************************/
/*
  Command format description:
 1, CMD_IAP_END (IAP end flag):
    The command format is as follows:
        CMD_IAP_END + 1 byte data length + 2 byte status (reserved).
    Return data:
        no
 2, CMD_IAP_VERIFY (IAP verification command):
     The command format is as follows:
         CMD_IAP_VERIFY + 1 byte data length + 2 byte data address + subsequent data.
    Return data:
        Correct: 0x0000
        Error: 0x0001
 3, CMD_IAP_PROM (IAP programming command):
     The command format is as follows:
         CMD_IAP_PROM + 1 byte data length + 2 bytes data address + subsequent data. Note that the number of data sent by this command must be an integer multiple of 4 bytes
     Return data:
        Correct: 0x0000
        Error: 0x0001
 4, CMD_IAP_ERASE (IAP erase command):
     The command format is as follows:
        CMD_IAP_ERASE+1 byte data length + 2 byte start address (usually 0x0000), this command only needs to be sent once
    Return data:
        Correct: 0x0000
        Error: 0x0001
*/
#define IAP_LEN 64
/* IAP download program structure */
typedef union _IAP_CMD
{
    struct
    {
        UINT8 cmd; /* Command code 0x81 */
        UINT8 len; /* The length of subsequent data */
        UINT8 addr[2]; /* Erase address */
    } erase; /* erase command */
    struct
    {
        UINT8 cmd; /* Command code 0x83 */
        UINT8 len; /* The length of subsequent data */
        UINT8 status[2]; /* Two-byte status, reserved */
    } end; /* end command */
    struct
    {
        UINT8 cmd; /* Command code 0x82 */
        UINT8 len; /* The length of subsequent data */
        UINT8 addr[2]; /* Check address */
        UINT8 buf[IAP_LEN-4]; /* Check data */
    } verify; /* Verify command */
    struct
    {
        UINT8 cmd; /* Command code 0x80 */
        UINT8 len; /* The length of subsequent data */
        UINT8 addr[2]; /* address */
        UINT8 buf[IAP_LEN-4]; /* Follow-up data */
    } program; /* programming command */
    struct
    {
        UINT8 buf[IAP_LEN]; /* Receive data packet*/
    } other;
} iap_cmd1;
#endif
