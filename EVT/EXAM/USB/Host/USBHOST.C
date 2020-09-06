/********************************** (C) COPYRIGHT *********** ********************
* File Name: USBHOST.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description:
 USB host example for CH559, start USB device under DP/DM port
 USB host application example, initialize and enumerate the USB devices connected to the DP/DM port, can operate U disk or operate USB mouse, does not include HID command processing
 If you operate the U disk, you need to add the CH559UFX.LIB/CH559UFI.LIB file system library file
************************************************** *****************************/
#define FOR_ROOT_UDISK_ONLY 1 // Only used for U disk file operation of DP/DM port (use subroutine library CH559UFI/X), otherwise this program will simply demonstrate HID operation
#include <CH559.H>
#include <stdio.h>
#include <string.h>
#include "../../USB_LIB/CH559UFI.H"
#include "../../USB_LIB/CH559UFI.C"

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
#if !FOR_ROOT_UDISK_ONLY
#define DEV_TYPE_KEYBOARD (USB_DEV_CLASS_HID | 0x20)
#define DEV_TYPE_MOUSE (USB_DEV_CLASS_HID | 0x30)
struct _RootHubDev
{
    UINT8 DeviceStatus; // Device status, 0-no device, 1-device but not initialized, 2-device but initialization enumeration failed, 3-device and initialization enumeration successful
// UINT8 DeviceAddress; // The assigned USB address of the device
    UINT8 DeviceSpeed; // 0 is low speed, non-zero is full speed
    UINT8 DeviceType; // Device type
// union {
// struct MOUSE {
// UINT8 MouseInterruptEndp; // Mouse Interrupt Endpoint Number
// UINT8 MouseIntEndpTog; // The synchronization flag of the mouse interrupt endpoint
// UINT8 MouseIntEndpSize; // The length of the mouse interrupt endpoint
//}
// struct PRINT {
//}
//}
//..... struct _Endp_Attr Endp_Attr[4];//The attributes of the endpoint, supports up to 4 endpoints
    UINT8 GpVar; // General variable
} xdata ThisUsbDev;
#endif
#define WAIT_USB_TOUT_200US 200 // Waiting for USB interrupt timeout time 200uS@Fsys=12MHz
/*
Convention: USB device address allocation rules (refer to USB_DEVICE_ADDR)
Address value Device location
0x02 USB device or external HUB under built-in Root-HUB0
0x03 USB device or external HUB under built-in Root-HUB1
0x1x USB device under port x of the external HUB under built-in Root-HUB0, x is 1~n
0x2x USB device under port x of the external HUB under built-in Root-HUB1, x is 1~n
*/
UINT8X RxBuffer[ MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[ MAX_PACKET_SIZE] _at_ 0x0040; // OUT, must even address
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
bit FoundNewDev;
#pragma NOAREGS
void mDelayuS( UINT16 n ); // Delay in uS
void mDelaymS( UINT16 n ); // Delay in mS
void DisableRootHubPort( void ); // Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
UINT8 AnalyzeRootHub( void ); // Analyze the port status and process the ROOT-HUB port device plug-in event
// Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for detected disconnection
void SetHostUsbAddr( UINT8 addr ); // Set the USB device address currently operated by the USB host
#if FOR_ROOT_UDISK_ONLY
#define SetUsbSpeed( x)
#else
void SetUsbSpeed( UINT8 FullSpeed ​​); // Set the current USB speed
#endif
void ResetRootHubPort( void ); // After the device is detected, reset the bus to prepare for the enumeration of the device, and set it to full speed by default
UINT8 EnableRootHubPort( void ); // Enable the port, and the corresponding bUH_PORT_EN is set to 1 to open the port, and the device may be disconnected, which may cause the return to fail
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
#if !FOR_ROOT_UDISK_ONLY
UINT8 AnalyzeHidIntEndp( PUINT8X buf ); // Analyze the address of the HID interrupt endpoint from the descriptor
#endif
UINT8 InitRootDevice( void ); // Initialize the USB device
/* Initialize the serial port for printf and getkey input and output */
void mInitSTDIO( void );
void InitUSB_Host( void ); // Initialize the USB host
//#define FREQ_SYS 12000000 // System main frequency is 12MHz
/************************************************* ******************************
* Function Name: mDelayus(UNIT16 n)
* Description: us delay function
* Input: UNIT16 n
* Output: None
* Return: None
************************************************** *****************************/
void mDelayuS( UINT16 n) // Delay in uS
{
    while (n) // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
    {
        ++ SAFE_MOD; // 2 Fsys cycles, for higher Fsys, add operation here
#ifdef FREQ_SYS
#if FREQ_SYS >= 14000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 16000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 18000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 20000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 22000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 24000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 26000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 28000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 30000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 32000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 34000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 36000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 38000000
        ++ SAFE_MOD;
#endif
#if FREQ_SYS >= 40000000
        ++ SAFE_MOD;
#endif
#endif
        - n;
    }
}
/************************************************* ******************************
* Function Name: mDelayms(UNIT16 n)
* Description: ms delay function
* Input: UNIT16 n
* Output: None
* Return: None
************************************************** *****************************/
void mDelaymS( UINT16 n) // Delay in mS
{
    while (n)
    {
        mDelayuS( 1000 );
        - n;
    }
}
/************************************************* ******************************
* Function Name: DisableRootHubPort(void)
* Description: Close the HUB port
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void DisableRootHubPort( void) // Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
{
    CH559DiskStatus = DISK_DISCONNECT;
#ifndef FOR_ROOT_UDISK_ONLY
    ThisUsbDev.DeviceStatus = ROOT_DEV_DISCONNECT;
// ThisUsbDev.DeviceAddress = 0x00;
#endif
    UHUB0_CTRL = 0x00; // Clear the control data related to HUB0, actually does not need to be cleared
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
    UINT8 s;
    s = ERR_SUCCESS;
    if (USB_HUB_ST & bUHS_H0_ATTACH) // device exists
    {
#if FOR_ROOT_UDISK_ONLY
        if (CH559DiskStatus == DISK_DISCONNECT
#else
        if (ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT // Detected device insertion
#endif
                || (UHUB0_CTRL & bUH_PORT_EN) == 0x00) // Device insertion is detected, but it has not been allowed, indicating that it has just been inserted
        {
            DisableRootHubPort( ); // Close the port
#if FOR_ROOT_UDISK_ONLY
            CH559DiskStatus = DISK_CONNECT;
#else
// ThisUsbDev.DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
            ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED; //Set the connection flag
#endif
            printf( "USB dev in\n" );
            s = ERR_USB_CONNECT;
        }
    }
#if FOR_ROOT_UDISK_ONLY
    else if (CH559DiskStatus >= DISK_CONNECT)
    {
#else
    else if (ThisUsbDev.DeviceStatus >= ROOT_DEV_CONNECTED) //The device is unplugged
    {
#endif
        DisableRootHubPort( ); // Close the port
        printf( "USB dev out\n" );
        if (s == ERR_SUCCESS)
        {
            s = ERR_USB_DISCON;
        }
    }
// UIF_DETECT = 0; // Clear interrupt flag
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
* Function Name: SetUsbSpeed
* Description: Set the current USB speed
* Input: UINT8 FullSpeed
* Output: None
* Return: None
************************************************** *****************************/
#if !FOR_ROOT_UDISK_ONLY
void SetUsbSpeed( UINT8 FullSpeed) //Non-U disk operation
{
    if (FullSpeed) // full speed
    {
        USB_CTRL &= ~ bUC_LOW_SPEED; // Full speed
        UH_SETUP &= ~ bUH_PRE_PID_EN; // prohibit PRE PID
    }
    else
    {
        USB_CTRL |= bUC_LOW_SPEED; // Low speed
    }
}
#endif
/************************************************* ******************************
* Function Name: ResetRootHubPort
* Description: After the device is detected, reset the bus to prepare for enumerating the device, and set it to default to full speed
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void ResetRootHubPort( void)
{
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE; /* The maximum packet size of endpoint 0 of the USB device */
    SetHostUsbAddr( 0x00 );
    SetUsbSpeed( 1 ); // The default is full speed
    UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET; // The default is full speed, start reset
    mDelaymS( 15 ); // reset time 10mS to 20mS
    UHUB0_CTRL = UHUB0_CTRL & ~ bUH_BUS_RESET; // end reset
    mDelayuS( 250 );
    UIF_DETECT = 0; // Clear interrupt flag
}
/************************************************* ******************************
* Function Name: EnableRootHubPort
* Description: Enable the port, the corresponding bUH_PORT_EN is set to 1 to enable the port, the device may be disconnected, which may cause the return failure
* Input: None
* Output: None
* Return: Return ERR_SUCCESS to detect a new connection, return ERR_USB_DISCON to indicate no connection
************************************************** *****************************/
UINT8 EnableRootHubPort( void)
{
#if FOR_ROOT_UDISK_ONLY
    if (CH559DiskStatus <DISK_CONNECT)
    {
        CH559DiskStatus = DISK_CONNECT;
    }
#else
    if (ThisUsbDev.DeviceStatus <ROOT_DEV_CONNECTED)
    {
        ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;
    }
#endif
    if (USB_HUB_ST & bUHS_H0_ATTACH) // There is a device
    {
#if !FOR_ROOT_UDISK_ONLY
        if ((UHUB0_CTRL & bUH_PORT_EN) == 0x00) // not yet enabled
        {
            ThisUsbDev.DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
            if (ThisUsbDev.DeviceSpeed ​​== 0)
            {
                UHUB0_CTRL |= bUH_LOW_SPEED; // Low speed
            }
        }
#endif
        UHUB0_CTRL |= bUH_PORT_EN; //Enable HUB port
        return( ERR_SUCCESS );
    }
    return( ERR_USB_DISCON );
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
        UH_EP_PID = endp_pid; // Specify the token PID and destination endpoint number
        UIF_TRANSFER = 0; // Allow transmission
// s = WaitUSB_Interrupt( );
        for (i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -)
        {
            ;
        }
        UH_EP_PID = 0x00; // Stop USB transmission
// if (s != ERR_SUCCESS) return( s ); // interrupt timeout, it may be a hardware exception
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
#if FOR_ROOT_UDISK_ONLY
            if (CH559DiskStatus == DISK_DISCONNECT)
            {
                return( ERR_USB_DISCON ); // USB device disconnect event
            }
            if (CH559DiskStatus == DISK_CONNECT)
            {
                return( ERR_USB_CONNECT ); // USB device connection event
            }
#else
            if (ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT)
            {
                return( ERR_USB_DISCON ); // USB device disconnect event
            }
            if (ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED)
            {
                return( ERR_USB_CONNECT ); // USB device connection event
            }
#endif
// if ((USB_HUB_ST & bUHS_H0_ATTACH) == 0x00) return( ERR_USB_DISCON ); // USB device disconnect event
            mDelayuS( 200 ); // Wait for the transfer to complete
        }
        if (UIF_TRANSFER) // transfer completed
        {
            if (U_TOG_OK)
            {
                return( ERR_SUCCESS );
            }
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
                    if (r == USB_PID_DATA0 && r == USB_PID_DATA1) // If out of sync, you need to discard and try again
                    {
                    } // Retry without synchronization
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
            USB_INT_FG = 0xFF; /* Clear interrupt flag */
        }
        mDelayuS( 15 );
    }
    while (++ TransRetry <3 );
    return( ERR_USB_TRANSFER ); // response timeout
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
    UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG; // Default DATA1
    UH_TX_LEN = 0x01; // There is no data by default, so the status stage is IN
    RemLen = pSetupReq -> wLengthH? 0xFF: pSetupReq -> wLengthL;
    if (RemLen && pBuf) // Need to send and receive data
    {
        if (pSetupReq -> bRequestType & USB_REQ_TYP_IN) // accept
        {
            while (RemLen)
            {
                mDelayuS( 200 );
                s = USBHostTransact( USB_PID_IN << 4 | 0x00, UH_RX_CTRL, 200000/20 ); // IN data
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
    UsbDevEndp0Size = ((PXUSB_DEV_DESCR)TxBuffer) -> bMaxPacketSize0; // The maximum packet length of endpoint 0. This is simplified processing. Normally, you should get the first 8 bytes and update UsbDevEndp0Size immediately before continuing
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
    pSetupReq -> wLengthL = len; // The total length of the complete configuration descriptor
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
UINT8 CtrlSetUsbConfig( UINT8 cfg)
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
    return( HostCtrlTransfer( NULL, NULL) ); /* Perform control transfer */
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
    for (i = 0; i <((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) // search interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    {
        if (((PXUSB_ENDP_DESCR)(buf+i)) -> bDescriptorType == USB_DESCR_TYP_ENDP // is the endpoint descriptor
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
* Input: None
* Output: None
* Return:
************************************************** *****************************/
UINT8 InitRootDevice( void)
{
    UINT8 i, s, cfg, dv_cls, if_cls;
    printf( "Reset host port\n" );
    ResetRootHubPort( ); // After detecting the device, reset the USB bus of the corresponding port
    for (i = 0, s = 0; i <100; i ++) // wait for the USB device to reset and reconnect, 100mS timeout
    {
        mDelaymS( 1 );
        if (EnableRootHubPort() == ERR_SUCCESS) // enable port
        {
            i = 0;
            s ++; // Time to wait for the stability of the USB device connection
            if (s> 15)
            {
                break; // has been connected stably for 15mS
            }
        }
    }
    if (i) // device is not connected after reset
    {
        DisableRootHubPort( );
        printf( "Disable host port because of disconnect\n" );
        return( ERR_USB_DISCON );
    }
    SetUsbSpeed( ThisUsbDev.DeviceSpeed ​​); // Set the current USB speed
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
        s = CtrlSetUsbAddress( ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL ); // Set USB device address
        if (s == ERR_SUCCESS)
        {
// ThisUsbDev.DeviceAddress = ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL;// Save the USB address
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
                if_cls = ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceClass; // interface class code
                if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE) // It is a USB storage device, basically confirm it is a U disk
                {
#if FOR_ROOT_UDISK_ONLY
                    CH559DiskStatus = DISK_USB_ADDR;
                    return( ERR_SUCCESS );
                }
                else
                {
                    return( ERR_USB_UNSUPPORT );
                }
#else
                    s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                    if (s == ERR_SUCCESS)
                    {
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        ThisUsbDev.DeviceType = USB_DEV_CLASS_STORAGE;
                        printf( "USB-Disk Ready\n" );
                        SetUsbSpeed( 1 ); // The default is full speed
                        return( ERR_SUCCESS );
                    }
                }
                else if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_PRINTER && ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceSubClass == 0x01) // It is a printer device
                {
                    s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                    if (s == ERR_SUCCESS)
                    {
                        //The endpoint information needs to be saved for the main program to perform USB transfer
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        ThisUsbDev.DeviceType = USB_DEV_CLASS_PRINTER;
                        printf( "USB-Print Ready\n" );
                        SetUsbSpeed( 1 ); // The default is full speed
                        return( ERR_SUCCESS );
                    }
                }
                else if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_HID && ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceSubClass <= 0x01) // It is a HID device, keyboard/mouse, etc.
                {
                    s = AnalyzeHidIntEndp( TxBuffer ); // Analyze the address of the HID interrupt endpoint from the descriptor
                    ThisUsbDev.GpVar = s & USB_ENDP_ADDR_MASK; // Save the address of the interrupt endpoint, bit 7 is used for the synchronization flag, cleared to 0
                    if_cls = ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceProtocol;
                    s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                    if (s == ERR_SUCCESS)
                    {
// Set_Idle( );
                        //The endpoint information needs to be saved for the main program to perform USB transfer
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        if (if_cls == 1)
                        {
                            ThisUsbDev.DeviceType = DEV_TYPE_KEYBOARD;
                            //Further initialization, such as device keyboard indicator LED, etc.
                            printf( "USB-Keyboard Ready\n" );
                            SetUsbSpeed( 1 ); // The default is full speed
                            return( ERR_SUCCESS );
                        }
                        else if (if_cls == 2)
                        {
                            ThisUsbDev.DeviceType = DEV_TYPE_MOUSE;
                            //In order to query the mouse status in the future, the descriptor should be analyzed to obtain the address, length and other information of the interrupt port
                            printf( "USB-Mouse Ready\n" );
                            SetUsbSpeed( 1 ); // The default is full speed
                            return( ERR_SUCCESS );
                        }
                        s = ERR_USB_UNSUPPORT;
                    }
                }
                else // can be further analyzed
                {
                    s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                    if (s == ERR_SUCCESS)
                    {
                        //The endpoint information needs to be saved for the main program to perform USB transfer
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        ThisUsbDev.DeviceStatus = dv_cls? Dv_cls: if_cls;
                        SetUsbSpeed( 1 ); // The default is full speed
                        return( ERR_SUCCESS ); /* The unknown device was initialized successfully */
                    }
                }
#endif
            }
        }
    }
    printf( "InitRootDev Err = %02X\n", (UINT16)s );
#if FOR_ROOT_UDISK_ONLY
    CH559DiskStatus = DISK_CONNECT;
#else
    ThisUsbDev.DeviceStatus = ROOT_DEV_FAILED;
#endif
    SetUsbSpeed( 1 ); // The default is full speed
    return( s );
}

/************************************************* ******************************
* Function Name: mInitSTDIO
* Description: Initialize the serial port for printf and getkey input and output
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void mInitSTDIO( void)
{
// SCON = 0x50;
    SM0 = 0;
    SM1 = 1;
    SM2 = 0;
    REN = 1;
    PCON |= SMOD;
    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1; // 0X20
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK; // *12
    TH1 = 0-13;//0xF3; /* 12MHz crystal oscillator, 4800bps*12=57600bps */
    TR1 = 1;
    TI = 1;
}
/************************************************* ******************************
* Function Name: InitUSB_Host
* Description: Initialize the USB host
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitUSB_Host( void)
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
    DisableRootHubPort( ); // Clear
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
// IE_USB = 1; // query method
}
UINT8X buf[ 100 ];
/************************************************* ******************************
* Function Name: UDISK_demo
* Description: U disk operation process: USB bus reset, U disk connection, get device descriptor and set USB address, optional get configuration descriptor, then arrive here, CH559 subroutine library will continue to complete the follow-up work
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
UINT8 UDISK_demo( void)
{
    UINT8 i, s;
    printf( "Start UDISK_demo @CH559UFI library\n" );
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
    if (CH559DiskStatus >= DISK_MOUNTED) /* U Disk is ready */
    {
        printf( "FileCreate\n" );
        strcpy( mCmdParam.Open.mPathName, "/MY_ADC.TXT" ); /* File name, the file is in the root directory */
        s = CH559FileCreate( ); /* Create a new file and open it, if the file already exists, delete it and then create a new one */
        if (s == ERR_SUCCESS)
        {
            printf( "ByteWrite\n" );
            i = sprintf( buf, "This time ADC data=%d, time count=%u\xd\xa", ADC_FIFO, (TH1<<8)|TL1 );/* ADC data demo */
            mCmdParam.ByteWrite.mByteCount = i; /* Specify the number of bytes written this time */
            mCmdParam.ByteWrite.mByteBuffer = buf; /* point to buffer */
            s = CH559ByteWrite( ); /* Write data to the file in bytes */
            if (s == ERR_SUCCESS)
            {
                /* By default, if the number of bytes mCmdParam.ByteWrite.mByteCount is not 0, then CH559ByteWrite is only responsible for writing data without modifying the file length.
                   If you do not write data for a long time, you should update the file length to prevent the previously written data from not matching the file length after a sudden power failure.
                   If you need to modify/update the file length immediately after writing the data, you can set the byte count mCmdParam.ByteWrite.mByteCount to 0 and then call CH559ByteWrite to forcefully update the file length.
                   If it is determined that there will be no sudden power failure or data will be continuously written soon later, there is no need to update the file length, which can increase the speed and reduce the loss of the U disk (the internal memory of the U disk has a limited life and should not be rewritten frequently) */
                /* mCmdParam.ByteWrite.mByteCount = 0; If you specify to write 0 bytes, it is used to refresh the length of the file
                                CH559ByteWrite( ); Write data to the file in bytes. Because it is 0 byte write, it is only used to update the length of the file. When data is written in stages, this method can be used to update the file length */
                printf( "FileClose with updating size\n" );
                mCmdParam.Close.mUpdateLen = 1; /* The file length is automatically calculated and the file is written in bytes. It is recommended that the library close the file to automatically update the file length */
                s = CH559FileClose( ); /* Close the file */
                if (s == ERR_SUCCESS)
                {
                    printf( "File Create..Write..Close OK\n" );
                }
                else
                {
                    printf( "ErrorClose=%02X\n", (UINT16)s );
                }
            }
            else
            {
                printf( "FileClose\n" );
                printf( "ErrorWrite=%02X\n", (UINT16)s );
                mCmdParam.Close.mUpdateLen = 0; /* Do not automatically calculate the file length */
                CH559FileClose( ); /* Close the file */
            }
        }
        else
        {
            printf( "ErrorCreate=%02X\n", (UINT16)s );
        }
    }
    else
    {
        printf( "Disk not ready=%02X\n", (UINT16)s );
    }
    printf( "Finished UDISK_demo\n" );
    return( s );
}
main()
{
    UINT8 i, s;
#if !FOR_ROOT_UDISK_ONLY
    UINT8 len, endp;
#endif
// SAFE_MOD = 0x55;
// SAFE_MOD = 0xAA;
// CLOCK_CFG |= bOSC_EN_XT;
    mInitSTDIO( ); /* In order to let the computer monitor the demonstration process through the serial port */
    printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
    InitUSB_Host( );
    CH559LibInit( ); /* Initialize CH559 library to support U disk files */
    FoundNewDev = 0;
    printf( "Wait Device In\n" );
    while (1)
    {
        s = ERR_SUCCESS;
        if (UIF_DETECT) // Process if there is a USB host detection interrupt
        {
            UIF_DETECT = 0; // Clear interrupt flag
            s = AnalyzeRootHub( ); // analyze ROOT-HUB status
            if (s == ERR_USB_CONNECT)
            {
                FoundNewDev = 1;
            }
        }
// if (s == ERR_USB_CONNECT) {
//}
        if (FoundNewDev || s == ERR_USB_CONNECT) // There is a new USB device inserted
        {
            FoundNewDev = 0;
            mDelaymS( 200 ); // Since the USB device has just been inserted and is not yet stable, wait for hundreds of milliseconds for the USB device to eliminate plugging jitter
            s = InitRootDevice( ); // initialize the USB device
            if (s != ERR_SUCCESS)
            {
                return( s );
            }
        }
// other work for each device
        mDelaymS( 100 ); // simulate the MCU to do other things
        if (RI == 0)
        {
            continue;
        }
        i = getkey( );
        printf( "%c", (UINT8)i );
#if FOR_ROOT_UDISK_ONLY
        if (i =='D' && CH559DiskStatus >= DISK_USB_ADDR)
        {
#else
        if (i =='D' && ThisUsbDev.DeviceType == USB_DEV_CLASS_STORAGE && ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS )// Simulate a subjective request and operate on a USB device
        {
#endif
            printf( "Access USB-disk\n" );
// SetHostUsbAddr( ThisUsbDev.DeviceAddress ); // Set the USB device address currently operated by the USB host
            SetUsbSpeed( ThisUsbDev.DeviceSpeed ​​); // Set the current USB speed
            // Operate the U disk, call CH559UFI or HostCtrlTransfer, USBHostTransact, etc.
            s = UDISK_demo( );
            if (s)
            {
                printf( "ErrCode=%02X\n", (UINT16)s );
            }
            SetUsbSpeed( 1 ); // The default is full speed
        }
#if !FOR_ROOT_UDISK_ONLY
        else if (i =='M' && ThisUsbDev.DeviceType == DEV_TYPE_MOUSE && ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS )// Simulate subjective requirements, need to operate the mouse
        {
            printf( "Query Mouse\n" );
// SetHostUsbAddr( ThisUsbDev.DeviceAddress ); // Set the USB device address currently operated by the USB host
            SetUsbSpeed( ThisUsbDev.DeviceSpeed ​​); // Set the current USB speed
// Operate the mouse
// s = GetDeviceDescr( ); // Get device descriptor
// if (s == USB_INT_SUCCESS) {
// for (s = 0; s <((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL; s ++) printf( "x%02X ", (UINT16)( TxBuffer[s]) );
// printf( "\n" ); // display the descriptor
//}
            endp = ThisUsbDev.GpVar; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag
            if (endp & USB_ENDP_ADDR_MASK) // Endpoint is valid
            {
                s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80? bUH_R_TOG | bUH_T_TOG: 0, 0 ); // CH559 transmits transaction, gets data, NAK does not retry
                if (s == ERR_SUCCESS)
                {
                    endp ^= 0x80; // Synchronization flag flip
                    ThisUsbDev.GpVar = endp; // Save the synchronization flag
                    len = USB_RX_LEN; // received data length
                    if (len)
                    {
                        printf("Mouse data: ");
                        for (i = 0; i <len; i ++)
                        {
                            printf("x%02X ",(UINT16)(RxBuffer[i]) );
                        }
                        printf("\n");
                    }
                }
                else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                {
                    printf("Mouse error %02x\n",(UINT16)s); // It may be disconnected
                }
            }
            else
            {
                printf("Mouse no interrupt endpoint\n");
            }
            SetUsbSpeed( 1 ); // The default is full speed
        }
        else
        {
        }
#endif
    }
}
