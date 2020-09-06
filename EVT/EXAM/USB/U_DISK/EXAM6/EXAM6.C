/********************************** (C) COPYRIGHT *********** ********************
* File Name :EXAM6.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description:
 CH559 C language U disk file byte read and write sample program,
 Demonstrate to find the /C51/CH559HFT.C file, if found, read the file content from the sector;
 Demonstrate the creation of the /LIUNEWFILE.TXT file, write the specified character string in the circular sector, and modify the file attributes;
 Demonstrate deleting the /OLD.TXT file;
 Support: FAT12/FAT16/FAT32
************************************************** *****************************/

#include <CH559.H>
#include <stdio.h>
#include <string.h>
#include "..\..\..\DEBUG.H"
#include "..\..\..\DEBUG.C"

#define CH559vSectorSize ((UINT16)CH559vSectorSizeH<<8) //The actual sector size, the default is 512 bytes
#define DISK_BASE_BUF_LEN 1024 /* The default disk data buffer size is 512 bytes (2048 or 4096 can be selected to support some large-sector U disks), 0 means that the buffer is forbidden to be defined in this file and used by the application The program is specified in pDISK_BASE_BUF*/
#define FOR_ROOT_UDISK_ONLY 1//U disk file operation only for DP/DM port (using subroutine library CH559UFI/X), U disk operation under HUB is not supported
//You also need to add LIB library files
//#define NO_DEFAULT_ACCESS_SECTOR 1 /* The default disk sector read and write subroutine is forbidden, and replace it with a self-written program below */
//#define NO_DEFAULT_DISK_CONNECT 1 /* The default check disk connection subroutine is forbidden, replace it with a self-written program below */
//#define NO_DEFAULT_FILE_ENUMER 1 /* The default file name enumeration callback program is prohibited, and replace it with a self-written program below */

#include "..\..\..\USB_LIB\CH559UFI.H"
#include "..\..\..\USB_LIB\CH559UFI.C"

#pragma NOAREGS


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

// Get device descriptor
UINT8C SetupGetDevDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };
// Get configuration descriptor
UINT8C SetupGetCfgDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };
// Set the USB address
UINT8C SetupSetUsbAddr[] = {USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Set USB configuration
UINT8C SetupSetUsbConfig[] = {USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Clear endpoint STALL
UINT8C SetupClrEndpStall[] = {USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

UINT8X UsbDevEndp0Size; /* The maximum packet size of endpoint 0 of the USB device */

//USB device related information table, CH559 supports up to 2 devices
#define ROOT_DEV_DISCONNECT 0
#define ROOT_DEV_CONNECTED 1
#define ROOT_DEV_FAILED 2
#define ROOT_DEV_SUCCESS 3

UINT8X RxBuffer[ MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[ MAX_PACKET_SIZE] _at_ 0x0040; // OUT, must even address
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
bit FoundNewDev;
bit RootHubId; // The currently operating root-hub port number: 0=HUB0,1=HUB1
UINT8X MY_FAT_BUF[ DISK_BASE_BUF_LEN] _at_ 0x0480; /* pDISK_FAT_BUF points to the disk data buffer of external RAM, the length of the buffer is the length of at least one sector */
UINT8X MY_DATA_BUF[ DISK_BASE_BUF_LEN] _at_ 0x0880; /* MY_DATA_BUF points to the disk data buffer of external RAM, the length of the buffer is at least one sector length, used for user data cache*/
//sbit LED_RUN = P1^0;
//sbit LED_CFG = P1^1;
//sbit LED_TMP = P1^2;

struct _RootHubDev{
UINT8 DeviceStatus; // Device status, 0-no device, 1-device but not initialized, 2-device but initialization enumeration failed, 3-device and initialization enumeration succeeded
UINT8 DeviceAddress; // The assigned USB address of the device
UINT8 DeviceSpeed; // 0 is low speed, non-zero is full speed
UINT8 DeviceType; // Device type
UINT8 GpVar; // General variable
} xdata RootHubDev[2];

#pragma NOAREGS

void mDelayuS( UINT16 n ); // Delay in uS
void mDelaymS( UINT16 n ); // Delay in mS
void DisableRootHubPort( UINT8 RootHubIndex ); // Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
UINT8 AnalyzeRootHub( void ); // Analyze the port status and process the ROOT-HUB port device plug-in event
// Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for detected disconnection
void SetHostUsbAddr( UINT8 addr ); // Set the USB device address currently operated by the USB host
void ResetRootHubPort( UINT8 RootHubIndex ); // After the device is detected, reset the bus to prepare for enumerating the device, and set it to default to full speed
UINT8 EnableRootHubPort( UINT8 RootHubIndex ); // Enable the port, and the corresponding bUH_PORT_EN is set to 1 to enable the port, and the device disconnection may cause the return failure
void SelectHubPort( UINT8 RootHubIndex); // Select the ROOT-HUB port specified by the operation
UINT8 WaitUSB_Interrupt( void ); // Wait for USB interrupt
// CH559 transfer transaction, enter the destination endpoint address/PID token, synchronization flag, the total NAK retry time in units of 20uS (0 means no retry, 0xFFFF unlimited retry), return 0 success, timeout/error retry
UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout ); // endp_pid: the upper 4 bits are the token_pid token, the lower 4 bits are the endpoint address
UINT8 HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen ); // Perform control transfer, 8-byte request code is in pSetupReq, DataBuf is an optional transmit and receive buffer
// If you need to receive and send data, then DataBuf needs to point to a valid buffer for storing subsequent data, and the total length of the actual successfully sent and received is returned and saved in the byte variable pointed to by ReqLen
void CopySetupReqPkg( PUINT8C pReqPkt ); // Copy control transmission request packet
UINT8 CtrlGetDeviceDescr( void ); // Get the device descriptor and return it in TxBuffer
UINT8 CtrlGetConfigDescr( void ); // Get the configuration descriptor and return it in TxBuffer
UINT8 CtrlSetUsbAddress( UINT8 addr ); // Set USB device address
UINT8 CtrlSetUsbConfig( UINT8 cfg ); // Set USB device configuration
UINT8 CtrlClearEndpStall( UINT8 endp ); // Clear endpoint STALL
UINT8 InitRootDevice( UINT8 RootHubIndex ); // Initialize the USB device
UINT8 EnumAllRootDevice( void ); // Enumerate all USB devices of ROOT-HUB port
void mInitSTDIO( void ); //Initialize the serial port for printf and getkey input and output
void InitUSB_Host( void ); // Initialize the USB host
void mStopIfError( UINT8 iError ); //Check the operation status, if there is an error, display the error code and stop
/************************************************* ******************************
* Function Name: DisableRootHubPort(UINT8 RootHubIndex)
* Description: Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
* Input: UINT8 RootHubIndex HUB port
* Output: None
* Return: None
************************************************** *****************************/
void DisableRootHubPort( UINT8 RootHubIndex)
{
RootHubDev[ RootHubIndex ].DeviceStatus = ROOT_DEV_DISCONNECT;
RootHubDev[ RootHubIndex ].DeviceAddress = 0x00;
if (RootHubIndex == 1) UHUB1_CTRL = 0x00; // Clear the control data related to HUB1, actually does not need to be cleared
else UHUB0_CTRL = 0x00; // Clear the control data related to HUB0, actually does not need to be cleared
}
/************************************************* ******************************
* Function Name: AnalyzeRootHub(void)
* Description: Analyze port status and handle port device plug-in events
                   Handle the plug-in event of the port. If the device is unplugged, call the DisableRootHubPort() function in the function to close the port, insert the event, and set the status bit of the corresponding port
* Input: None
* Output: None
* Return: Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for disconnection detected
************************************************** *****************************/
UINT8 AnalyzeRootHub( void)
{
UINT8 s = ERR_SUCCESS;
if (USB_HUB_ST & bUHS_H0_ATTACH) {// device exists
if (RootHubDev[0].DeviceStatus == ROOT_DEV_DISCONNECT // Detected device insertion
|| (UHUB0_CTRL & bUH_PORT_EN) == 0x00) {// Device insertion is detected, but it is not yet allowed, indicating that it has just been inserted
DisableRootHubPort( 0 ); // Close the port
RootHubDev[0].DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
RootHubDev[0].DeviceStatus = ROOT_DEV_CONNECTED; //Set connection flag
printf( "HUB 0 dev in\n" );
s = ERR_USB_CONNECT;
}
}
else if (RootHubDev[0].DeviceStatus >= ROOT_DEV_CONNECTED) {//The device is unplugged
DisableRootHubPort( 0 ); // Close the port
printf( "HUB 0 dev out\n" );
if (s == ERR_SUCCESS) s = ERR_USB_DISCON;
}
if (USB_HUB_ST & bUHS_H1_ATTACH) {// device exists
if (RootHubDev[1].DeviceStatus == ROOT_DEV_DISCONNECT // Detected device insertion
|| (UHUB1_CTRL & bUH_PORT_EN) == 0x00) {// Device insertion is detected, but it is not yet allowed, indicating that it has just been inserted
DisableRootHubPort( 1 ); // Close the port
RootHubDev[1].DeviceSpeed ​​= USB_HUB_ST & bUHS_HM_LEVEL? 0: 1;
RootHubDev[1].DeviceStatus = ROOT_DEV_CONNECTED; //Set connection flag
printf( "HUB 1 dev in\n" );
s = ERR_USB_CONNECT;
}
}
else if (RootHubDev[1].DeviceStatus >= ROOT_DEV_CONNECTED) {//The device is unplugged
DisableRootHubPort( 1 ); // Close the port
printf( "HUB 1 dev out\n" );
if (s == ERR_SUCCESS) s = ERR_USB_DISCON;
}
return( s );
}
/************************************************* ******************************
* Function Name: SetHostUsbAddr
* Description: Set the USB device address currently operated by the USB host
* Input: UINT8 addr
* Output: None
* Return: None
************************************************** *****************************/
void SetHostUsbAddr( UINT8 addr)
{
    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | addr & 0x7F;
}
/************************************************* ******************************
* Function Name: ResetRootHubPort
* Description: After the device is detected, reset the bus to prepare for enumerating the device, and set it to default to full speed
* Input: UINT8 RootHubIndex
* Output: None
* Return: None
************************************************** *****************************/
void ResetRootHubPort( UINT8 RootHubIndex)
{
UsbDevEndp0Size = DEFAULT_ENDP0_SIZE; //Maximum packet size of endpoint 0 of the USB device
SetHostUsbAddr( 0x00 );
SetUsbSpeed( 1 ); // The default is full speed
if (RootHubIndex == 1) {
UHUB1_CTRL = UHUB1_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
mDelaymS( 15 ); // reset time 10mS to 20mS
UHUB1_CTRL = UHUB1_CTRL & ~ bUH_BUS_RESET;// End reset
}
else {
UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
mDelaymS( 15 ); // reset time 10mS to 20mS
UHUB0_CTRL = UHUB0_CTRL & ~ bUH_BUS_RESET; // end reset
}
mDelayuS( 250 );
UIF_DETECT = 0; // Clear interrupt flag
}
/************************************************* ******************************
* Function Name: EnableRootHubPort
* Description: Enable the port, the corresponding bUH_PORT_EN is set to 1 to enable the port, the device may be disconnected, which may cause the return failure
* Input: UINT8 RootHubIndex
* Output: None
* Return: Return ERR_SUCCESS to detect a new connection, return ERR_USB_DISCON to indicate no connection
************************************************** *****************************/
UINT8 EnableRootHubPort( UINT8 RootHubIndex)
{
if (RootHubDev[ RootHubIndex ].DeviceStatus <ROOT_DEV_CONNECTED) RootHubDev[ RootHubIndex ].DeviceStatus = ROOT_DEV_CONNECTED;
if (RootHubIndex == 1) {
if (USB_HUB_ST & bUHS_H1_ATTACH) {// There is equipment
if ((UHUB1_CTRL & bUH_PORT_EN) == 0x00) {// not yet enabled
RootHubDev[1].DeviceSpeed ​​= USB_HUB_ST & bUHS_HM_LEVEL? 0: 1;
if (RootHubDev[1].DeviceSpeed ​​== 0) UHUB1_CTRL |= bUH_LOW_SPEED; // low speed
}
UHUB1_CTRL |= bUH_PORT_EN; //Enable HUB port
return( ERR_SUCCESS );
}
}
else {
if (USB_HUB_ST & bUHS_H0_ATTACH) {// There is equipment
if ((UHUB0_CTRL & bUH_PORT_EN) == 0x00) {// not yet enabled
RootHubDev[0].DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
if (RootHubDev[0].DeviceSpeed ​​== 0) UHUB0_CTRL |= bUH_LOW_SPEED; // low speed
}
UHUB0_CTRL |= bUH_PORT_EN; //Enable HUB port
return( ERR_SUCCESS );
}
}
return( ERR_USB_DISCON );
}
/************************************************* ******************************
* Function Name: SelectHubPort
* Description: Select the ROOT-HUB port specified by the operation
* Input: UINT8 RootHubIndex
* Output: None
* Return: Return ERR_SUCCESS to detect a new connection, return ERR_USB_DISCON to indicate no connection
************************************************** *****************************/
void SelectHubPort( UINT8 RootHubIndex)
{// Select the ROOT-HUB port specified by the operation
SetHostUsbAddr( RootHubDev[RootHubIndex].DeviceAddress ); // Set the USB device address currently operated by the USB host
SetUsbSpeed( RootHubDev[RootHubIndex].DeviceSpeed ​​); // Set the current USB speed
RootHubId = RootHubIndex? 1: 0;
}
/************************************************* ******************************
* Function Name: WaitUSB_Interrupt
* Description: Wait for USB interrupt
* Input: None
* Output: None
* Return: return ERR_SUCCESS data received or sent successfully
                   ERR_USB_UNKNOWN Data receiving or sending failed
************************************************** *****************************/
UINT8 WaitUSB_Interrupt( void)
{
    UINT16 i;
    for (i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -)
    {
        ;
    }
    return( UIF_TRANSFER? ERR_SUCCESS: ERR_USB_UNKNOWN );
}
/************************************************* ******************************
* Function Name: USBHostTransact
* Description: CH559 transmission transaction, enter the destination endpoint address/PID token, synchronization flag, the total NAK retry time in units of 20uS (0 means no retry, 0xFFFF unlimited retry), return 0 success, timeout/error retry test
                   This subroutine focuses on easy to understand, and in actual applications, in order to provide running speed, the code of this subroutine should be optimized
* Input: UINT8 endp_pid token and address endp_pid: the upper 4 bits are the token_pid token, the lower 4 bits are the endpoint address
                   UINT8 tog synchronization flag
                   UINT16 timeout timeout
* Output: None
* Return: ERR_USB_UNKNOWN timed out, the hardware may be abnormal
                   ERR_USB_DISCON Device disconnect
                   ERR_USB_CONNECT Device connection
                   ERR_SUCCESS transfer completed
************************************************** *****************************/
UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout)
{
// UINT8 TransRetry;
#define TransRetry UEP0_T_LEN // Save memory
    UINT8 s, r;
    UINT16 i;
    UH_RX_CTRL = UH_TX_CTRL = tog;
    TransRetry = 0;
    do
    {
// LED_TMP = 0;
        UH_EP_PID = endp_pid; // Specify the token PID and destination endpoint number
        UIF_TRANSFER = 0; // Allow transmission
// s = WaitUSB_Interrupt( );
        for (i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -)
        {
            ;
        }
        UH_EP_PID = 0x00; // Stop USB transmission
// LED_TMP = 1;
// if (s != ERR_SUCCESS) return( s ); // interrupt timeout, it may be a hardware abnormality
        if (UIF_TRANSFER == 0)
        {
            return( ERR_USB_UNKNOWN );
        }
        if (UIF_DETECT) // USB device plug-in event
        {
// mDelayuS( 200 ); // Wait for the transfer to complete
            UIF_DETECT = 0; // Clear interrupt flag
            s = AnalyzeRootHub( ); // analyze ROOT-HUB status
            if (s == ERR_USB_CONNECT)
            {
                FoundNewDev = 1;
            }
            if (CH559DiskStatus == DISK_DISCONNECT)
            {
                return( ERR_USB_DISCON ); // USB device disconnect event
            }
            if (CH559DiskStatus == DISK_CONNECT)
            {
                return( ERR_USB_CONNECT ); // USB device connection event
            }
// if ((USB_HUB_ST & bUHS_H0_ATTACH) == 0x00) return( ERR_USB_DISCON );// USB device disconnect event
            mDelayuS( 200 ); // Wait for the transmission to complete
        }
        if (UIF_TRANSFER) // transfer completed
        {
            if (U_TOG_OK)
            {
                return( ERR_SUCCESS );
            }
#ifdef DEBUG_NOW
            printf("endp_pid=%02X\n",(UINT16)endp_pid);
            printf("USB_INT_FG=%02X\n",(UINT16)USB_INT_FG);
            printf("USB_INT_ST=%02X\n",(UINT16)USB_INT_ST);
            printf("USB_MIS_ST=%02X\n",(UINT16)USB_MIS_ST);
            printf("USB_RX_LEN=%02X\n",(UINT16)USB_RX_LEN);
            printf("UH_TX_LEN=%02X\n",(UINT16)UH_TX_LEN);
            printf("UH_RX_CTRL=%02X\n",(UINT16)UH_RX_CTRL);
            printf("UH_TX_CTRL=%02X\n",(UINT16)UH_TX_CTRL);
            printf("UHUB0_CTRL=%02X\n",(UINT16)UHUB0_CTRL);
            printf("UHUB1_CTRL=%02X\n",(UINT16)UHUB1_CTRL);
#endif
            r = USB_INT_ST & MASK_UIS_H_RES; // USB device response status
            if (r == USB_PID_STALL)
            {
                return( r | ERR_USB_TRANSFER );
            }
            if (r == USB_PID_NAK)
            {
                if (timeout == 0)
                {
                    return( r | ERR_USB_TRANSFER );
                }
                if (timeout <0xFFFF)
                {
                    timeout --;
                }
                - TransRetry;
            }
            else switch (endp_pid >> 4)
                {
                case USB_PID_SETUP:
                case USB_PID_OUT:
// if (U_TOG_OK) return( ERR_SUCCESS );
// if (r == USB_PID_ACK) return( ERR_SUCCESS );
// if (r == USB_PID_STALL || r == USB_PID_NAK) return( r | ERR_USB_TRANSFER );
                    if (r)
                    {
                        return( r | ERR_USB_TRANSFER ); // Not a timeout/error, unexpected response
                    }
                    break; // Retry after timeout
                case USB_PID_IN:
// if (U_TOG_OK) return( ERR_SUCCESS );
// if (tog? r == USB_PID_DATA1: r == USB_PID_DATA0) return( ERR_SUCCESS );
// if (r == USB_PID_STALL || r == USB_PID_NAK) return( r | ERR_USB_TRANSFER );
                    if (r == USB_PID_DATA0 && r == USB_PID_DATA1 )// If out of sync, you need to discard and try again
                        {} // Retry without synchronization
                    else if (r)
                    {
                        return( r | ERR_USB_TRANSFER ); // Not a timeout/error, unexpected response
                    }
                    break; // Retry after timeout
                default:
                    return( ERR_USB_UNKNOWN ); // impossible situation
                    break;
                }
        }
        else // other interrupts, things that shouldn't happen
        {
            USB_INT_FG = 0xFF; // Clear interrupt flag
        }
        mDelayuS( 15 );
    }
    while (++ TransRetry <3 );
    return( ERR_USB_TRANSFER ); // Response timeout
}
/************************************************* ******************************
* Function Name: HostCtrlTransfer
* Description: Perform control transmission, the 8-byte request code is in pSetupReq, and DataBuf is an optional transceiver buffer
* Input: PUINT8X DataBuf If you need to receive and send data, then DataBuf needs to point to a valid buffer for storing subsequent data
                   The total length of PUINT8I RetLen actually successfully transmitted and received is stored in the byte variable pointed to by RetLen
* Output: None
* Return: ERR_USB_BUF_OVER IN state phase error
                   ERR_SUCCESS Data exchange is successful
                   Other error status
************************************************** *****************************/
UINT8 HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen)
{
    UINT8 s, RemLen, RxLen, RxCnt, TxCnt;
    PUINT8X xdata pBuf;
    PUINT8I xdata pLen;
    pBuf = DataBuf;
    pLen = RetLen;
    mDelayuS( 200 );
    if (pLen)
    {
        *pLen = 0; // The total length of actual successful sending and receiving
    }
    UH_TX_LEN = sizeof( USB_SETUP_REQ );
    s = USBHostTransact( USB_PID_SETUP << 4 | 0x00, 0x00, 200000/20 ); // SETUP phase, 200mS timeout
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG;// Default DATA1
    UH_TX_LEN = 0x01; // There is no data by default, so the status stage is IN
    RemLen = pSetupReq -> wLengthH? 0xFF: pSetupReq -> wLengthL;
    if (RemLen && pBuf) // Need to send and receive data
    {
        if (pSetupReq -> bRequestType & USB_REQ_TYP_IN) // accept
        {
            while (RemLen)
            {
                mDelayuS( 200 );
                s = USBHostTransact( USB_PID_IN << 4 | 0x00, UH_RX_CTRL, 200000/20 );// IN data
                if (s != ERR_SUCCESS)
                {
                    return( s );
                }
                RxLen = USB_RX_LEN <RemLen? USB_RX_LEN: RemLen;
                RemLen -= RxLen;
                if (pLen)
                {
                    *pLen += RxLen; // The total length of actual successful sending and receiving
                }
// memcpy( pBuf, RxBuffer, RxLen );
// pBuf += RxLen;
                for (RxCnt = 0; RxCnt != RxLen; RxCnt ++)
                {
                    *pBuf = RxBuffer[ RxCnt ];
                    pBuf ++;
                }
                if (USB_RX_LEN == 0 || (USB_RX_LEN & (UsbDevEndp0Size-1)))
                {
                    break; // short package
                }
            }
            UH_TX_LEN = 0x00; // The status phase is OUT
        }
        else // send
        {
            while (RemLen)
            {
                mDelayuS( 200 );
                UH_TX_LEN = RemLen >= UsbDevEndp0Size? UsbDevEndp0Size: RemLen;
// memcpy( TxBuffer, pBuf, UH_TX_LEN );
// pBuf += UH_TX_LEN;
                for (TxCnt = 0; TxCnt != UH_TX_LEN; TxCnt ++)
                {
                    TxBuffer[ TxCnt] = *pBuf;
                    pBuf ++;
                }
                s = USBHostTransact( USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, 200000/20 );// OUT data
                if (s != ERR_SUCCESS)
                {
                    return( s );
                }
                RemLen -= UH_TX_LEN;
                if (pLen)
                {
                    *pLen += UH_TX_LEN; // The total length of actual successful sending and receiving
                }
            }
// UH_TX_LEN = 0x01; // Status phase is IN
        }
    }
    mDelayuS( 200 );
    s = USBHostTransact( (UH_TX_LEN? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), bUH_R_TOG | bUH_T_TOG, 200000/20 ); // STATUS stage
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (UH_TX_LEN == 0)
    {
        return( ERR_SUCCESS ); // status OUT
    }
    if (USB_RX_LEN == 0)
    {
        return( ERR_SUCCESS ); // Status IN, check the IN status and return data length
    }
    return( ERR_USB_BUF_OVER ); // Error in IN state stage
}
/************************************************* ******************************
* Function Name: CopySetupReqPkg
* Description: Copy control transmission request packet
* Input: PUINT8C pReqPkt control request packet address
* Output: None
* Return: None
************************************************** *****************************/
void CopySetupReqPkg( PUINT8C pReqPkt)
{
    UINT8 i;
    for (i = 0; i != sizeof( USB_SETUP_REQ ); i ++)
    {
        ((PUINT8X)pSetupReq)[ i] = *pReqPkt;
        pReqPkt ++;
    }
}
/************************************************* ******************************
* Function Name: CtrlGetDeviceDescr
* Description: Get the device descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: ERR_USB_BUF_OVER Descriptor length error
                   ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlGetDeviceDescr( void)
{
    UINT8 s;
    UINT8D len;
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;
    CopySetupReqPkg( SetupGetDevDescr );
    s = HostCtrlTransfer( TxBuffer, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    UsbDevEndp0Size = ((PXUSB_DEV_DESCR)TxBuffer) -> bMaxPacketSize0;// The maximum packet length of endpoint 0, which is simplified processing. Normally, you should get the first 8 bytes and immediately update UsbDevEndp0Size before continuing
    if (len <((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Descriptor length error
    }
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: CtrlGetConfigDescr
* Description: Get the configuration descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: ERR_USB_BUF_OVER Descriptor length error
                   ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlGetConfigDescr( void)
{
    UINT8 s;
    UINT8D len;
    CopySetupReqPkg( SetupGetCfgDescr );
    s = HostCtrlTransfer( TxBuffer, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetCfgDescr) -> wLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Return length error
    }
    len = ((PXUSB_CFG_DESCR)TxBuffer) -> wTotalLengthL;
    if (len> MAX_PACKET_SIZE)
    {
        return( ERR_USB_BUF_OVER ); // Return length error
    }
    CopySetupReqPkg( SetupGetCfgDescr );
    pSetupReq -> wLengthL = len; // Total length of the complete configuration descriptor
    s = HostCtrlTransfer( TxBuffer, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetCfgDescr) -> wLengthL || len <((PXUSB_CFG_DESCR)TxBuffer) -> wTotalLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Descriptor length error
    }
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: CtrlSetUsbAddress
* Description: Set the USB device address
* Input: UINT8 addr device address
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlSetUsbAddress( UINT8 addr)
{
    UINT8 s;
    CopySetupReqPkg( SetupSetUsbAddr );
    pSetupReq -> wValueL = addr; // USB device address
    s = HostCtrlTransfer( NULL, NULL ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    SetHostUsbAddr( addr ); // Set the USB device address currently operated by the USB host
    mDelaymS( 10 ); // Wait for the USB device to complete the operation
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: CtrlSetUsbConfig
* Description: Set USB device configuration
* Input: UINT8 cfg configuration value
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlSetUsbConfig( UINT8 cfg) // Set USB device configuration
{
    CopySetupReqPkg( SetupSetUsbConfig );
    pSetupReq -> wValueL = cfg; // USB device configuration
    return( HostCtrlTransfer( NULL, NULL) ); // Perform control transfer
}
/************************************************* ******************************
* Function Name: CtrlClearEndpStall
* Description: Clear endpoint STALL
* Input: UINT8 endp endpoint address
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlClearEndpStall( UINT8 endp)
{
    CopySetupReqPkg( SetupClrEndpStall ); // Clear the error of the endpoint
    pSetupReq -> wIndexL = endp; // endpoint address
    return( HostCtrlTransfer( NULL, NULL) ); // Perform control transfer
}
/************************************************* ******************************
* Function Name: AnalyzeHidIntEndp
* Description: Analyze the address of the HID interrupt endpoint from the descriptor
* Input: PUINT8X buf buffer address of data to be analyzed
* Output: None
* Return: interrupt endpoint address
************************************************** *****************************/
UINT8 AnalyzeHidIntEndp( PUINT8X buf)
{
    UINT8 i, s, l;
    s = 0;
    for (i = 0; i <((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l )// Search interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    {
        if (((PXUSB_ENDP_DESCR)(buf+i)) -> bDescriptorType == USB_DESCR_TYP_ENDP// is the endpoint descriptor
                && (((PXUSB_ENDP_DESCR)(buf+i)) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_INTER // is the interrupt endpoint
                && (((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_DIR_MASK) )// is the IN endpoint
        {
            s = ((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;// The address of the interrupt endpoint
            break; // You can save wMaxPacketSize and bInterval as needed
        }
        l = ((PXUSB_ENDP_DESCR)(buf+i)) -> bLength; // current descriptor length, skip
        if (l> 16)
        {
            break;
        }
    }
    return( s );
}
/************************************************* ******************************
* Function Name: InitRootDevice
* Description: Initialize the USB device
* Input: UINT8 RootHubIndex built-in HUB port number 0/1
* Output: None
* Return:
************************************************** *****************************/
UINT8 InitRootDevice( UINT8 RootHubIndex)
{
    UINT8 i, s, cfg, dv_cls, if_cls;
    printf( "Reset host port\n" );
    ResetRootHubPort( RootHubIndex ); // After detecting the device, reset the USB bus of the corresponding port
    for (i = 0, s = 0; i <100; i ++) // wait for the USB device to reset and reconnect, 100mS timeout
    {
        mDelaymS( 1 );
        if (EnableRootHubPort( RootHubIndex) == ERR_SUCCESS) // Enable port
        {
            i = 0;
            s ++; // Time to wait for the stability of the USB device connection
            if (s> 20)
            {
                break; // has been connected stably for 20mS
            }
        }
    }
    if (i) // device is not connected after reset
    {
        DisableRootHubPort( RootHubIndex );
        printf( "Disable host port because of disconnect\n" );
        return( ERR_USB_DISCON );
    }
SelectHubPort( RootHubIndex);
    printf( "GetDevDescr: ");
    s = CtrlGetDeviceDescr( ); // Get device descriptor
    if (s == ERR_SUCCESS)
    {
        for (i = 0; i <((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL; i ++)
        {
            printf( "x%02X ", (UINT16)( TxBuffer[i]) );
        }
        printf( "\n" ); // display the descriptor
        dv_cls = ((PXUSB_DEV_DESCR)TxBuffer) -> bDeviceClass; // device class code
s = CtrlSetUsbAddress( RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL );//Set the USB device address, plus RootHubIndex can ensure that 2 HUB ports are assigned different addresses
        if (s == ERR_SUCCESS)
        {
RootHubDev[RootHubIndex].DeviceAddress = RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL;// Save the USB address
            printf( "GetCfgDescr: ");
            s = CtrlGetConfigDescr( ); // Get configuration descriptor
            if (s == ERR_SUCCESS)
            {
                cfg = ((PXUSB_CFG_DESCR)TxBuffer) -> bConfigurationValue;
                for (i = 0; i <((PXUSB_CFG_DESCR)TxBuffer) -> wTotalLengthL; i ++)
                {
                    printf( "x%02X ", (UINT16)( TxBuffer[i]) );
                }
                printf("\n");
                /* Analyze the configuration descriptor, get the endpoint data/each endpoint address/each endpoint size, etc., update the variables endp_addr and endp_size, etc. */
                if_cls = ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceClass;// Interface class code
                if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE) // It is a USB storage device, basically confirm it is a U disk
                {
// s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
// if (s == ERR_SUCCESS) {
RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
RootHubDev[RootHubIndex].DeviceType = USB_DEV_CLASS_STORAGE;
printf( "USB-Disk Ready\n" );
SetUsbSpeed( 1 ); // The default is full speed
return( ERR_SUCCESS );
//}
                }
             }
         }
}
    printf( "InitRootDev Err = %02X\n", (UINT16)s );
    CH559DiskStatus = DISK_CONNECT;
    SetUsbSpeed( 1 ); // The default is full speed
    return( s );
}

/************************************************* ******************************
* Function Name: EnumAllRootDevice
* Description: Enumerate all USB devices of ROOT-HUB port
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
UINT8 EnumAllRootDevice( void)
{
UINT8 s, RootHubIndex;
printf( "EnumAllRootDev\n" );
for (RootHubIndex = 0; RootHubIndex != 2; RootHubIndex ++) {
if (RootHubDev[RootHubIndex].DeviceStatus == ROOT_DEV_CONNECTED) {// The device has just been inserted and has not been initialized
s = InitRootDevice( RootHubIndex ); // Initialize/enumerate the USB device of the specified HUB port
if (s != ERR_SUCCESS) return( s );
}
}
return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: SearchTypeDevice
* Description: Search for the port number where the specified type of device is located on ROOT-HUB, and the output port number is 0xFFFF, but it is not found
                   The upper 8 bits of the output are the ROOT-HUB port number, the lower 8 bits are the port number of the external HUB, and the lower 8 bits are 0, the device is directly on the ROOT-HUB port
                   Of course, you can also search according to the PID of the USB manufacturer's VID product (record the VID and PID of each device in advance), and specify the search serial number
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
UINT16 SearchTypeDevice( UINT8 type)
{
UINT8 RootHubIndex;
for (RootHubIndex = 0; RootHubIndex != 2; RootHubIndex ++) {// The current search can avoid the problem that the device is unplugged in the middle and some information is not updated in time
if (RootHubDev[RootHubIndex].DeviceType == type && RootHubDev[RootHubIndex].DeviceStatus >= ROOT_DEV_SUCCESS) return( (UINT16)RootHubIndex << 8 ); // The type matches and the enumeration is successful, on the ROOT-HUB port
}
return( 0xFFFF );
}
/************************************************* ******************************
* Function Name: InitUSB_Host
* Description: Initialize the USB host
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitUSB_Host( void) // initialize the USB host
{
    IE_USB = 0;
// LED_CFG = 1;
// LED_RUN = 0;
    USB_CTRL = bUC_HOST_MODE; // Set the mode first
    USB_DEV_AD = 0x00;
    UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN;
    UH_RX_DMA = RxBuffer;
    UH_TX_DMA = TxBuffer;
    UH_RX_CTRL = 0x00;
    UH_TX_CTRL = 0x00;
    USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB host and DMA, and automatically pause before the interrupt flag is cleared
// UHUB0_CTRL = 0x00;
// UHUB1_CTRL = 0x00;
    UH_SETUP = bUH_SOF_EN;
    USB_INT_FG = 0xFF; // Clear interrupt flag
    DisableRootHubPort(0); // Clear
    DisableRootHubPort(1); // Clear
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
// IE_USB = 1; // query method
}
/************************************************* ******************************
* Function Name: mStopIfError
* Description: Check the operating status, if there is an error, display the error code and stop
* Input: UINT8 iError
* Output: None
* Return: None
************************************************** *****************************/
void mStopIfError( UINT8 iError)
{
    if (iError == ERR_SUCCESS)
    {
        return; // operation is successful
    }
    printf( "Error: %02X\n", (UINT16)iError ); // display error
    /* After encountering an error, you should analyze the error code and CH559DiskStatus status. For example, call CH559DiskReady to query whether the current U disk is connected. If the U disk is disconnected, then wait for the U disk to be plugged in and then operate.
       Suggested processing steps after error:
       1. Call CH559DiskReady once, and continue the operation if successful, such as Open, Read/Write, etc.
       2. If CH559DiskReady is unsuccessful, then forcibly start the operation from the beginning (waiting for U disk connection, CH559DiskReady, etc.) */
    while (1)
    {
// LED_TMP=0; // LED flashes
// mDelaymS( 100 );
// LED_TMP=1;
// mDelaymS( 100 );
    }
}
void main()
{
    UINT8 s,i,SecCount;
    UINT16 tmp,loc;
    UINT8 tmpbuf[32];
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
    mInitSTDIO( ); //In order to let the computer monitor the demonstration process through the serial port
    printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
    InitUSB_Host( );
    CH559LibInit( ); //Initialize CH559 library to support U disk files
    pDISK_FAT_BUF = & MY_FAT_BUF[0]; //Disk FAT data buffer pointing to external RAM, can be combined with pDISK_BASE_BUF to save RAM
    /* If you want to improve the file access speed, you can re-point pDISK_FAT_BUF to another independently allocated buffer with the same size as pDISK_BASE_BUF after calling CH559LibInit in the main program */
    FoundNewDev = 0;
    printf( "Wait Device In\n" );
    while (1)
    {
        s = ERR_SUCCESS;
        if (UIF_DETECT) //Process if there is a USB host detection interrupt
        {
            UIF_DETECT = 0; //Clear interrupt flag
            s = AnalyzeRootHub( ); //Analyze ROOT-HUB status
            if (s == ERR_USB_CONNECT)
            {
                FoundNewDev = 1;
            }
        }
        if (FoundNewDev) // There is a new USB device inserted
        {
            FoundNewDev = 0;
            mDelaymS( 200 ); // Since the USB device has just been inserted and is not yet stable, wait for hundreds of milliseconds for the USB device to eliminate the plugging jitter
s = EnumAllRootDevice( ); // Enumerate USB devices of all ROOT-HUB ports
            loc = SearchTypeDevice( USB_DEV_CLASS_STORAGE ); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
if (loc != 0xFFFF) {// found
printf( "Start UDISK_demo @CH559UFI library\n" );
// U disk operation process: USB bus reset, U disk connection, get device descriptor and set USB address, optional get configuration descriptor, then arrive here, CH559 subroutine library will continue to complete the follow-up work
i = (UINT8)( loc >> 8 );
SelectHubPort(i); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
                CH559DiskStatus = DISK_USB_ADDR;
                for (i = 0; i != 10; i ++)
                {
                    printf( "Wait DiskReady\n" );
                    s = CH559DiskReady( );
                    if (s == ERR_SUCCESS)
                    {
                        break;
                    }
                    mDelaymS( 50 );
                }
                if (CH559DiskStatus >= DISK_MOUNTED) //disk is ready
                {
#if DISK_BASE_BUF_LEN
                    //if (DISK_BASE_BUF_LEN <CH559vSectorSize) {/* Check whether the disk data buffer is large enough, CH559vSectorSize is the actual sector size of the U disk */
                    // printf( "Too large sector size\n" );
                    // goto UnknownUsbDevice;
                    //}
#endif
                    /* Query the physical capacity of the disk */
                    /* printf( "DiskSize\n" );
                            i = CH559DiskQuery( );
                            mStopIfError( i );
                            printf( "TotalSize = %u MB \n", (unsigned int)( mCmdParam.Query.mTotalSector * CH559vSectorSizeH / 2) ); Displays the capacity in MB
                    */
                    /* Read the original file */
                    printf( "Open\n" );
                    strcpy( mCmdParam.Open.mPathName, "/C51/CH559HFT.C" );//File name, the file is in the C51 subdirectory
                    s = CH559FileOpen( ); //Open the file
                    if (s == ERR_MISS_DIR || s == ERR_MISS_FILE )//No file found
                    {
                        printf( "File not found\n" );
                    }
                    else // find the file or error
                    {
                        printf( "Query\n" );
                        i = CH559FileQuery( ); //Query the information of the current file
                        mStopIfError( i );
                        printf( "Read\n" );
                        CH559vFileSize = CH559vFileSize+(sizeof( MY_DATA_BUF )-1); //The length of the original file
                        SecCount = CH559vFileSize/ sizeof( MY_DATA_BUF) ;//Calculate the number of sectors of the file, because reading and writing are based on sectors, first add CH559vSectorSize-1 to read the part of the file tail that is less than 1 sector
                        printf( "Size=%ld, Sec=%d\n", CH559vFileSize, (UINT16)SecCount );
                        while(SecCount--)
                        {
                          mCmdParam.Read.mSectorCount = sizeof( MY_DATA_BUF )/512; //Read all data, if there are more than 2 sectors, only 2 sectors will be read
                          mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0];//Point to the starting address of the file data buffer
                          i = CH559FileRead( ); //Read data from file
                          mStopIfError( i );
                          if(SecCount == 0) break;
/* for(tmp=0; tmp<sizeof( MY_DATA_BUF ); tmp++)
                          {
                            printf("%02X ",(UINT16)MY_DATA_BUF[tmp]);
                          }
                          printf("\n");
*/
}
                        tmp = (CH559vFileSize-(sizeof( MY_DATA_BUF )-1))%sizeof( MY_DATA_BUF );
                        if((tmp == 0)&&(CH559vFileSize != 0)) tmp = sizeof( MY_DATA_BUF );
                        CH559vFileSize = CH559vFileSize-(sizeof( MY_DATA_BUF )-1); //Restore the length of the original file
/*
                        for(i=0; i<tmp; i++)
                        {
                          printf("%02X ",(UINT16)MY_DATA_BUF[i]);
                        }
                        printf("\n");
*/
                        /*
                         If the file is relatively large and cannot be read at one time, you can call CH559FileRead to continue reading, and the file pointer will automatically move backward
                         while (1) {
                           c = 4; read 4 sectors at a time, the larger the buffer definition, the more sectors read at one time
                           mCmdParam.Read.mSectorCount = c; Specify the number of sectors read
                           mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0]; points to the starting address of the file data buffer
                           CH559FileRead(); The file pointer automatically moves back after reading
                           Data processing
                           if (mCmdParam.Read.mSectorCount <c) break; If the number of sectors actually read is smaller, the file has ended
                         }
                         If you want to start reading and writing from the specified position, you can move the file pointer
                          mCmdParam.Locate.mSectorOffset = 3; Skip the first 3 sectors of the file to start reading and writing
                          i = CH559FileLocate( );
                          mCmdParam.Read.mSectorCount = 10;
                          mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0]; points to the starting address of the file data buffer
                          CH559FileRead(); Directly read the data starting from the (CH559vSectorSizeH*256*3) byte of the file, the first 3 sectors are skipped
                          If you want to add new data to the end of the original file, you can move the file pointer
                          i = CH559FileOpen( );
                          mCmdParam.Locate.mSectorOffset = 0xffffffff; Move to the end of the file, in units of sectors, if the original file is 3 bytes, add it from CH559vSectorSizeH bytes
                          i = CH559FileLocate( );
                          mCmdParam.Write.mSectorCount = 10;
                          mCmdParam.Write.mDataBuffer = &MY_DATA_BUF[0];
                          CH559FileWrite(); Add data after the original file
                          Use CH559FileRead to define the starting address of the data buffer by yourself
                          mCmdParam.Read.mSectorCount = 2;
                          mCmdParam.Read.mDataBuffer = 0x50; Put the read data into the buffer starting at 50H, you need to specify the starting address of the buffer
                          CH559FileRead(); Read 2 sectors from the file to the specified buffer
                          Use CH559FileWrite to define the starting address of the data buffer by yourself
                          mCmdParam.Wiite.mSectorCount = 2;
                          mCmdParam.Write.mDataBuffer = 0x50; Write the data in the buffer starting at 50H
                          CH559FileWrite(); Write the data in the specified buffer to 2 sectors to the file
                        */
                        printf( "Close\n" );
                        i = CH559FileClose( ); //Close the file
                        mStopIfError( i );
                    }
                    printf( "Create\n" );
                    strcpy( mCmdParam.Create.mPathName, "/NEWFILE.TXT" );//New file name, in the root directory, Chinese file name
                    s = CH559FileCreate( ); //Create a new file and open it, if the file already exists, delete it and then create a new one */
                    mStopIfError( s );
                    printf( "Write\n" );
                    strcpy( tmpbuf, "0000ABCDEFGHIJKLMNOPQRSTUVWXYZ\xd\xa" );//Ready to write file data
                    for(i=0; i<(DISK_BASE_BUF_LEN/sizeof(tmpbuf)); i++)
                    {
                        tmp=i*sizeof(tmpbuf);
                        strcpy(&MY_DATA_BUF[tmp],tmpbuf);
                    }
                    for(tmp=0; tmp<sizeof(MY_DATA_BUF); tmp++)
                    {
                        printf("%02X",(UINT16)MY_DATA_BUF[tmp]);
                    }
                    printf("\n");
                    for(s=0; s<10; s++)
                    {
                        mCmdParam.Write.mSectorCount = 2; //Write data of all sectors
                        mCmdParam.Write.mDataBuffer = &MY_DATA_BUF[0]; //Point to the starting address of the file data buffer
                        i = CH559FileWrite( ); //Write data to file
                        mStopIfError( i );
                        printf("Successfully written %02X times\n",(UINT16)s);
                    }
                    /* printf( "Modify\n" );
                       mCmdParam.Modify.mFileAttr = 0xff; Input parameter: the new file attribute, if it is 0FFH, it will not be modified
                       mCmdParam.Modify.mFileTime = 0xffff; Input parameter: the new file time, if it is 0FFFFH, it will not be modified, and the default time generated by the new file will be used
                       mCmdParam.Modify.mFileDate = MAKE_FILE_DATE( 2015, 5, 18 ); Input parameters: New file date: 2015.05.18
                       mCmdParam.Modify.mFileSize = 0xffffffff; Input parameters: the new file length, in bytes, the file should be automatically updated when the file is closed by the library, so it is not modified here
                       i = CH559FileModify( ); modify the current file information, modification date
                       mStopIfError( i );
                    */
                    printf( "Close\n" );
                    mCmdParam.Close.mUpdateLen = 1; //Automatically calculate the file length, write the file in bytes, it is recommended to let the library close the file to automatically update the file length
                    i = CH559FileClose( );
                    mStopIfError( i );
                    /* Delete a file */
                    /*printf( "Erase\n" );
                      strcpy( mCmdParam.Create.mPathName, "/OLD.TXT" ); The name of the file to be deleted, in the root directory
                      i = CH559FileErase( ); delete file and close
                      if (i != ERR_SUCCESS) printf( "Error File not exist: %02X\n", (UINT16)i ); display error
                    */
                    printf( "U disk demo complete\n" );
                }
                else
                {
                    printf( "U disk is not ready ERR =%02X\n", (UINT16)s );
                }
            }
            else
            {
                printf("Failed to initialize the USB flash drive, please unplug the USB flash drive and try again\n");
            }
        }
        mDelaymS( 100 ); // simulate the MCU to do other things
        SetUsbSpeed( 1 ); // The default is full speed
    }
}
