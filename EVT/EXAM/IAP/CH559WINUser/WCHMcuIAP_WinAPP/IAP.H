/*
#ifndef __WCHMCU_IAP_H__
#define __WCHMCU_IAP_H__
*/
//IAP communication command and structure definition are the same as the MCU definition of the lower computer

#define Uart_Sync_Head1 0x57
#define Uart_Sync_Head2 0xab

/*The following is the definition of ISP download commands*/
#define CMD_IAP_PROM 0x80 //ISP programming command
#define CMD_IAP_ERASE 0x81 //ISP erase command
#define CMD_IAP_VERIFY 0x82 //ISP verification command
#define CMD_IAP_END 0x83 //ISP end flag

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

#define UINT8 UCHAR
#define IAP_LEN 64

#define mMAX_CMDBUF_LENGTH 60 //Maximum length of IAP command packet data buffer
#define mMAX_CMDPKT_LENGTH (mIAP_COMMAND_HEAD + mMAX_CMDBUF_LENGTH) //Maximum bytes of IAP command packet


/*ISP download program structure*/
typedef union _IAP_CMD {
struct {
UINT8 cmd;
UINT8 len;
UINT8 addr[2];
} erase; /*Erase command*/
struct {
UINT8 cmd;
UINT8 len;
UINT8 status[2];
} end; /*End command*/
struct {
UINT8 cmd;
UINT8 len; //Length of subsequent data
UINT8 addr[2]; //Check address
        UINT8 buf[IAP_LEN-4]; /* Check data */
} verify; /*Verify command*/
struct {
UINT8 cmd;
UINT8 len; //Length of subsequent data
UINT8 addr[2]; //Address
        UINT8 buf[IAP_LEN-4]; /* Follow-up data */
}program; /*program command*/
struct {
        UINT8 buf[IAP_LEN]; /* Receive data packet*/
} other; /*Receive command*/
}IAPCMDS;

#define IAP_UART_BAUDRATE 57600

//Download the IMCU user program through the USB interface and the IAP firmware of the MCU
DWORD WINAPI IAPFlashDownloadThread(LPVOID lpParameter);
