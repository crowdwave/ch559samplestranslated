// ************************************************ *******************
//Website: http://wch.cn
//Email: tech@wch.cn
//Author: WCH 2015/10/13
// ************************************************ *******************

#ifndef __UINTERFACE_H__
#define __UINTERFACE_H__


//#define DISK_BASE_BUF_LEN 512 /* The default disk data buffer size is 512 bytes (you can choose 2048 or even 4096 to support some large-sector U disks). If it is 0, it is forbidden to define the buffer in this file and Specified by the application in pDISK_BASE_BUF*/
#define FOR_ROOT_UDISK_ONLY 1//U disk file operation only for DP/DM port (using subroutine library CH559UFI/X), U disk operation under HUB is not supported

// Each subroutine returns status code
#define ERR_SUCCESS 0x00 // operation is successful
#define ERR_USB_CONNECT 0x15 /* A USB device connection event has been detected and has been connected */
#define ERR_USB_DISCON 0x16 /* A USB device disconnection event has been detected and has been disconnected */
#define ERR_USB_BUF_OVER 0x17 /* The data transmitted by USB is wrong or there is too much data and buffer overflow */
#define ERR_USB_DISK_ERR 0x1F /* USB storage operation failed, it may be that the USB storage is not supported during initialization, and the disk may be damaged or disconnected during read and write operations */
#define ERR_USB_TRANSFER 0x20 /* NAK/STALL and other error codes are in 0x20~0x2F */
#define ERR_USB_UNSUPPORT 0xFB
#define ERR_USB_UNKNOWN 0xFE
#define WAIT_USB_TOUT_200US 200 // Waiting for USB interrupt timeout time 200uS@Fsys=12MHz
#define SetUsbSpeed( x)
UINT8X UsbDevEndp0Size; /* Maximum packet size of endpoint 0 of the USB device */


//USB device related information table, CH559 supports up to 2 devices
#define ROOT_DEV_DISCONNECT 0
#define ROOT_DEV_CONNECTED 1
#define ROOT_DEV_FAILED 2
#define ROOT_DEV_SUCCESS 3
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
bit FoundNewDev;
#pragma NOAREGS
void mDelayuS( UINT16 n ); // Delay in uS
void mDelaymS( UINT16 n ); // Delay in mS
void DisableRootHubPort( void ); // Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
UINT8 AnalyzeRootHub( void ); // Analyze the port status and process the ROOT-HUB port device plug-in event
// Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for detected disconnection
void SetHostUsbAddr( UINT8 addr ); // Set the USB device address currently operated by the USB host
void ResetRootHubPort( void ); // After the device is detected, reset the bus to prepare for the enumeration of the device, and set it to full speed by default
UINT8 EnableRootHubPort( void ); // Enable the port, and the corresponding bUH_PORT_EN is set to 1 to turn on the port. The device disconnection may cause the return failure
UINT8 WaitUSB_Interrupt( void ); // Wait for USB interrupt
// CH559 transfer transaction, enter the destination endpoint address/PID token, synchronization flag, the total NAK retry time in units of 20uS (0 means no retry, 0xFFFF unlimited retry), return 0 success, timeout/error retry
UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout ); // endp_pid: the upper 4 bits are the token_pid token, the lower 4 bits are the endpoint address
UINT8 HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen ); // Perform control transfer, 8-byte request code is in pSetupReq, DataBuf is an optional transmit and receive buffer
// If you need to receive and send data, then DataBuf needs to point to a valid buffer for storing subsequent data, and the total length of the actual successfully sent and received is returned and saved in the byte variable pointed to by ReqLen
void CopySetupReqPkg( PUINT8C pReqPkt ); // copy control transmission request packet
UINT8 CtrlGetDeviceDescr( void ); // Get the device descriptor and return it in TxBuffer
UINT8 CtrlGetConfigDescr( void ); // Get the configuration descriptor and return it in TxBuffer
UINT8 CtrlSetUsbAddress( UINT8 addr ); // Set USB device address
UINT8 CtrlSetUsbConfig( UINT8 cfg ); // Set USB device configuration
UINT8 CtrlClearEndpStall( UINT8 endp ); // Clear endpoint STALL
UINT8 InitRootDevice( void ); // Initialize the USB device
/* Initialize the serial port for printf and getkey input and output */
void mInitSTDIO( void );
void InitUSB_Host( void ); // Initialize the USB host
#endif
