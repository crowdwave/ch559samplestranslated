/********************************** (C) COPYRIGHT *********** ********************
* File Name: USBH_KM.C
* Author: WCH
* Version: V1.5
* Date: 2018/08/01
* Description:
 USB host example for CH559, start USB device under DP/DM and HP/HM port
 USB host application example, initialize and enumerate the devices connected to the USB port, support up to 2 USB devices at the same time, support level 1 external HUB,
 Can operate USB keyboard, mouse and HUB, including HID command processing
 U disk operation is not supported, if you need to operate U disk, please refer to other examples
************************************************** *****************************/
#include <CH559.H>
#include <stdio.h>
#include <string.h>
#pragma NOAREGS
// Each subroutine returns status code
#define ERR_SUCCESS 0x00 // operation is successful
#define ERR_USB_CONNECT 0x15 /* A USB device connection event has been detected and has been connected */
#define ERR_USB_DISCON 0x16 /* A USB device disconnection event has been detected and has been disconnected */
#define ERR_USB_BUF_OVER 0x17 /* The data transmitted by USB is wrong or there is too much data and buffer overflow */
#define ERR_USB_DISK_ERR 0x1F /* USB storage operation failed, it may be that the USB storage is not supported during initialization, and the disk may be damaged or disconnected during read and write operations */
#define ERR_USB_TRANSFER 0x20 /* NAK/STALL and other error codes are in 0x20~0x2F */
#define ERR_USB_OVER_IF 0x30 /* The number of device interfaces exceeds 4 */
#define ERR_USB_UNSUPPORT 0xFB /*USB device not supported*/
#define ERR_USB_UNKNOWN 0xFE /*Device operation error*/
/*Get device descriptor*/
UINT8C SetupGetDevDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };
/*Get configuration descriptor*/
UINT8C SetupGetCfgDescr[] = {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };
/*Set the USB address*/
UINT8C SetupSetUsbAddr[] = {USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*Set USB configuration*/
UINT8C SetupSetUsbConfig[] = {USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*Set USB IDLE*/
UINT8C SetupSetUsbIDLE[] = {0x21, HID_SET_IDLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*Set the USB interface configuration*/
UINT8C SetupSetUsbInterface[] = {USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*Clear endpoint STALL*/
UINT8C SetupClrEndpStall[] = {USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*Get HID device report descriptor*/
UINT8C SetupGetHIDDevReport[] = {0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x41, 0x00 };
/* set usb device report*/
UINT8C SetupSetReport[] = {USB_DESCR_TYP_HID,0x09,0x00,0x02,0x00,0x00,0x01,0x00};
UINT8X UsbDevEndp0Size; /* The maximum packet size of endpoint 0 of the USB device */
/*USB device related information table, CH559 supports up to 2 devices*/
#define ROOT_DEV_DISCONNECT 0
#define ROOT_DEV_CONNECTED 1
#define ROOT_DEV_FAILED 2
#define ROOT_DEV_SUCCESS 3
#define DEV_TYPE_KEYBOARD (USB_DEV_CLASS_HID | 0x20)
#define DEV_TYPE_MOUSE (USB_DEV_CLASS_HID | 0x30)
UINT8 ReportData = 0;
#define HUB_INTERFACE_COUNT 4
#define GLOB_USAGE_PAGE_DEF 0X05 // Unsigned integer specifying the current Usage Page.
#define GLOB_USAGE_PAGE_DESKTOP 0X01 // generic desktop
#define LOCAL_USAGE_DEF 0X09 // Usage index for an item usage; represents a suggested usage for the item or collection.
#define LOCAL_USAGE_MOUSE 0X02 // MOUSE
#define LOCAL_USAGE_KEY 0X06 // KEY

#define DEBUG 1

struct _RootHubDev
{
    UINT8 DeviceStatus; // Device status, 0-no device, 1-device but not initialized, 2-device but initialization enumeration failed, 3-device and initialization enumeration succeeded
    UINT8 DeviceAddress; // The assigned USB address of the device
    UINT8 DeviceSpeed; // 0 is low speed, non-zero is full speed
    UINT8 DeviceType; // Device type
    UINT8 InfCnt; // Device interface number
    struct interface{
        UINT16 DescrLen; // hid descripor len
        UINT8 InAddr; // in endpoint address
        UINT8 OutAddr; // in endpoint address
    UINT8 DeviceType; // device type
        UINT8 WaitUSB_IN_Interval;
    }Interface[HUB_INTERFACE_COUNT];
} xdata RootHubDev[2];
UINT8X RxBuffer[ MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[ MAX_PACKET_SIZE*3] _at_ 0x0040; // OUT, must even address
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
bit RootHubId; // The currently operating root-hub port number: 0=HUB0,1=HUB1
bit FoundNewDev;

UINT16X T0Count[2][HUB_INTERFACE_COUNT]; //T0 count in ms
#pragma NOAREGS
#define WAIT_USB_TOUT_200US 200 // Waiting for USB interrupt timeout time 200uS@Fsys=12MHz
#define WAIT_USB_IN_COUNT 0x2EE0 // Waiting for USB interrupt timeout time 200uS@Fsys=12MHz

void mDelayuS( UINT16 n ); // Delay in uS
void mDelaymS( UINT16 n ); // Delay in mS
void DisableRootHubPort( UINT8 RootHubIndex ); // Close the specified ROOT-HUB port, in fact, the hardware has been automatically closed, here is just to clear some structure states
UINT8 AnalyzeRootHub( void ); // Analyze the ROOT-HUB status and handle the ROOT-HUB port device plug-in event
// Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for detected disconnection
void SetHostUsbAddr( UINT8 addr ); // Set the USB device address currently operated by the USB host
void SetUsbSpeed( UINT8 FullSpeed ​​); // Set the current USB speed
void ResetRootHubPort( UINT8 RootHubIndex ); // After the device is detected, reset the bus of the corresponding port to prepare for enumerating the device, and set it to default to full speed
UINT8 EnableRootHubPort( UINT8 RootHubIndex ); // Enable the ROOT-HUB port, and the corresponding bUH_PORT_EN is set to 1 to enable the port, and the device disconnection may cause the return failure
void SelectHubPort( UINT8 RootHubIndex, UINT8 HubPortIndex );// HubPortIndex=0 selects the designated ROOT-HUB port for operation, otherwise, selects the designated port of the external HUB that operates the designated ROOT-HUB port
UINT8 WaitUSB_Interrupt( void ); // Wait for USB interrupt
// CH559 transfer transaction, enter the destination endpoint address/PID token, synchronization flag, the total NAK retry time in units of 20uS (0 means no retry, 0xFFFF unlimited retry), return 0 success, timeout/error retry
UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout ); // endp_pid: the upper 4 bits are the token_pid token, the lower 4 bits are the endpoint address
UINT8 HostCtrlTransfer( PUINT8 DataBuf, PUINT8 RetLen ); // Perform control transfer, 8-byte request code is in pSetupReq, and DataBuf is an optional transmit and receive buffer
// If you need to receive and send data, then DataBuf needs to point to a valid buffer for storing subsequent data, and the total length of the actual successfully sent and received is returned and saved in the byte variable pointed to by ReqLen
void CopySetupReqPkg( PUINT8C pReqPkt ); // Copy control transmission request packet
UINT8 CtrlGetDeviceDescr( void ); // Get the device descriptor and return it in TxBuffer
UINT8 CtrlGetConfigDescr( void ); // Get the configuration descriptor and return it in TxBuffer
UINT8 CtrlSetUsbAddress( UINT8 addr ); // Set USB device address
UINT8 CtrlSetUsbConfig( UINT8 cfg ); // Set USB device configuration
UINT8 CtrlSetUsbIntercace( UINT8 cfg ); // Set USB device interface
UINT8 CtrlClearEndpStall( UINT8 endp ); // Clear endpoint STALL
UINT8 AnalyzeHidIntEndp( PUINT8X buf ); // Analyze the address of the HID interrupt endpoint from the descriptor
UINT8 InitRootDevice( UINT8 RootHubIndex ); // Initialize the USB device of the designated ROOT-HUB port
UINT16 SearchTypeDevice( UINT8 type ); // Search for the port number where the device of the specified type is located on each port of ROOT-HUB and external HUB. If the output port number is 0xFFFF, it is not found
// The upper 8 bits of the output are the ROOT-HUB port number, the lower 8 bits are the port number of the external HUB, and the lower 8 bits are 0, the device is directly on the ROOT-HUB port
void mInitSTDIO( void ); // Initialize the serial port for printf and getkey input and output
void InitUSB_Host( void ); // Initialize the USB host
//#define FREQ_SYS 12000000 // System frequency is 12MHz

#define T0_START 1
#define T0_STOP 0

#define mTimer0ClkFsys() (T2MOD |= bTMR_CLK | bT0_CLK) //Timer, clock=Fsys
//CH559 Timer0 start (SS=1)/end (SS=0)
#define mTimer0RunCTL( SS) (TR0 = SS? T0_START: T0_STOP)
/************************************************* ******************************
* Function Name: mTimer0ModSetup(UINT8 mode)
* Description: CH559 timer counter 0 mode 0 setting
* Input: UINT8 mode, Timer0 mode selection
                   0: Mode 0, 13-bit timer, the upper 3 bits of TL0 are invalid
                   1: Mode 1, 16-bit timer
                   2: Mode 2, 8-bit automatic reload timer
                   3: Mode 3, two 8-bit timers
* Output: None
* Return: None
************************************************** *****************************/
void mTimer0ModSetup(UINT8 mode)
{
    TMOD &= 0xf0;
    TMOD |= mode;
}

/************************************************* ******************************
* Function Name: mTimer0SetData(UINT16 dat)
* Description: CH559Timer0 TH0 and TL0 assignment
* Input: UINT16 dat; timer assignment
* Output: None
* Return: None
************************************************** *****************************/
void mTimer0SetData(UINT16 dat)
{
    UINT16 tmp;
    tmp = 65536-dat;
    TL0 = tmp & 0xff;
    TH0 = (tmp>>8) & 0xff;
}

/************************************************* ******************************
* Function Name: mTimer0Interrupt()
* Description: CH559 timer counter 0 timer counter interrupt processing function
************************************************** *****************************/
void mTimer0Interrupt( void) //interrupt INT_NO_TMR0 using 1 //timer0 interrupt service routine, use register set 1
{//In mode 3, TH0 uses the interrupt resource of Timer1
    UINT8 i;
    if(TF0)
    {
      CAP1 = ~CAP1;
      mTimer0SetData(WAIT_USB_IN_COUNT); //Non-auto reload mode needs to re-assign TH0 and TL0
      for(i=0;i<HUB_INTERFACE_COUNT;i++)
      {
        T0Count[0][i]++;
        T0Count[1][i]++;
      }
      TF0 = 0;
    }
}

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
* Function Name: DisableRootHubPort(UINT8 RootHubIndex)
* Description: Close the HUB port
* Input: UINT8 RootHubIndex specifies the ROOT_HUB port
* Output: None
* Return: None
************************************************** *****************************/
void DisableRootHubPort( UINT8 RootHubIndex) // Close the specified ROOT-HUB port, in fact the hardware has been automatically closed, here is just to clear some structure states
{
    RootHubDev[ RootHubIndex ].DeviceStatus = ROOT_DEV_DISCONNECT;
    RootHubDev[ RootHubIndex ].DeviceAddress = 0x00;
    if (RootHubIndex == 1)
    {
        UHUB1_CTRL = 0x00; // Clear the control data related to HUB1, actually does not need to be cleared
    }
    else
    {
        UHUB0_CTRL = 0x00; // Clear the control data related to HUB0, actually does not need to be cleared
    }
}
/************************************************* ******************************
* Function Name: AnalyzeRootHub(void)
* Description: Analyze the ROOT-HUB status and handle the ROOT-HUB port device plug-in event
                   If the device is unplugged, call the DisableRootHubPort() function in the function to close the port, insert the event, and set the status bit of the corresponding port
* Input: None
* Output: None
* Return: Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for disconnection detected
************************************************** *****************************/
UINT8 AnalyzeRootHub( void)
{
    UINT8 s;
    s = ERR_SUCCESS;
    if (USB_HUB_ST & bUHS_H0_ATTACH) // device exists, HUB0
    {
        if (RootHubDev[0].DeviceStatus == ROOT_DEV_DISCONNECT // Detected device insertion
                || (UHUB0_CTRL & bUH_PORT_EN) == 0x00) // Device insertion is detected, but it has not been allowed, indicating that it has just been inserted
        {
            DisableRootHubPort( 0 ); // Close the port
// RootHubDev[0].DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
            RootHubDev[0].DeviceStatus = ROOT_DEV_CONNECTED; //Set connection flag
            printf( "HUB 0 dev in\n" );
            s = ERR_USB_CONNECT;
        }
    }
    else if (RootHubDev[0].DeviceStatus >= ROOT_DEV_CONNECTED )//The device is unplugged
    {
        DisableRootHubPort( 0 ); // Close the port
        printf( "HUB 0 dev out\n" );
        //UH_SETUP &= ~bUH_SOF_EN;
        if (s == ERR_SUCCESS)
        {
            s = ERR_USB_DISCON; //Set device disconnect flag
        }
    }
    if (USB_HUB_ST & bUHS_H1_ATTACH) // device exists, HUB1
    {
        if (RootHubDev[1].DeviceStatus == ROOT_DEV_DISCONNECT // Detected device insertion
                || (UHUB1_CTRL & bUH_PORT_EN) == 0x00) // Device insertion is detected, but it has not been allowed, indicating that it has just been inserted
        {
            DisableRootHubPort( 1 ); // Close the port
// RootHubDev[1].DeviceSpeed ​​= USB_HUB_ST & bUHS_HM_LEVEL? 0: 1;
            RootHubDev[1].DeviceStatus = ROOT_DEV_CONNECTED; //Set connection flag
            printf( "HUB 1 dev in\n" );
            s = ERR_USB_CONNECT;
        }
    }
    else if (RootHubDev[1].DeviceStatus >= ROOT_DEV_CONNECTED )//The device is unplugged
    {
        DisableRootHubPort( 1 ); // Close the port
        //UH_SETUP &= ~bUH_SOF_EN;
        printf( "HUB 1 dev out\n" );
        if (s == ERR_SUCCESS)
        {
            s = ERR_USB_DISCON;
        }
    }
// UIF_DETECT = 0; // Clear connection interruption flag
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
void SetUsbSpeed( UINT8 FullSpeed)
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
/************************************************* ******************************
* Function Name: ResetRootHubPort( UINT8 RootHubIndex)
* Description: After the device is detected, reset the bus to prepare for enumerating the device, and set it to default to full speed
* Input: UINT8 RootHubIndex designated port
* Output: None
* Return: None
************************************************** *****************************/
void ResetRootHubPort( UINT8 RootHubIndex)
{
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE; //Maximum packet size of endpoint 0 of the USB device
    SetHostUsbAddr( 0x00 );
    SetUsbSpeed( 1 ); // The default is full speed
    if (RootHubIndex == 1)
    {
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
        mDelaymS( 25 ); // reset time 10mS to 20mS
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_BUS_RESET; // end reset
    }
    else
    {
        UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
        mDelaymS( 25 ); // reset time 10mS to 20mS
        UHUB0_CTRL = UHUB0_CTRL & ~ bUH_BUS_RESET; // end reset
    }
    mDelayuS( 250 );
    UIF_DETECT = 0; // Clear interrupt flag
}
/************************************************* ******************************
* Function Name: EnableRootHubPort( UINT8 RootHubIndex)
* Description: Enable the ROOT-HUB port, and the corresponding bUH_PORT_EN is set to 1 to enable the port. The device disconnection may cause the return failure
* Input: UINT8 RootHubIndex designated port
* Output: None
* Return: Return ERR_SUCCESS to detect a new connection, return ERR_USB_DISCON to indicate no connection
************************************************** *****************************/
UINT8 EnableRootHubPort( UINT8 RootHubIndex)
{
    if (RootHubDev[ RootHubIndex ].DeviceStatus <ROOT_DEV_CONNECTED)
    {
        RootHubDev[ RootHubIndex ].DeviceStatus = ROOT_DEV_CONNECTED;
    }
    if (RootHubIndex == 1)
    {
        if (USB_HUB_ST & bUHS_H1_ATTACH) // HUB1 has equipment
        {
            if ((UHUB1_CTRL & bUH_PORT_EN) == 0x00) // not yet enabled
            {
                RootHubDev[1].DeviceSpeed ​​= USB_HUB_ST & bUHS_HM_LEVEL? 0: 1;
                if (RootHubDev[1].DeviceSpeed ​​== 0)
                {
                    UHUB1_CTRL |= bUH_LOW_SPEED; // Low speed
                }
            }
            UHUB1_CTRL |= bUH_PORT_EN; //Enable HUB1 port
            return( ERR_SUCCESS );
        }
    }
    else
    {
        if (USB_HUB_ST & bUHS_H0_ATTACH) // HUB0 has equipment
        {
            if ((UHUB0_CTRL & bUH_PORT_EN) == 0x00) // not yet enabled
            {
                RootHubDev[0].DeviceSpeed ​​= USB_HUB_ST & bUHS_DM_LEVEL? 0: 1;
                if (RootHubDev[0].DeviceSpeed ​​== 0)
                {
                    UHUB0_CTRL |= bUH_LOW_SPEED; // Low speed
                }
            }
            UHUB0_CTRL |= bUH_PORT_EN; //Enable HUB0 port
            return( ERR_SUCCESS );
        }
    }
    return( ERR_USB_DISCON );
}
/************************************************* ******************************
* Function Name: SelectHubPort( UINT8 RootHubIndex, UINT8 HubPortIndex)
* Description: Select the HUB port to be operated
* Input: UINT8 RootHubIndex HubPortIndex=0 to select the designated ROOT-HUB port for operation, otherwise select the designated port of the external HUB for the designated ROOT-HUB port
                   UINT8 HubPortIndex Select the designated port of the external HUB that operates the designated ROOT-HUB port
* Output: None
* Return: None
************************************************** *****************************/
void SelectHubPort( UINT8 RootHubIndex, UINT8 HubPortIndex)
{
    UINT8 i;
    i = HubPortIndex;
    SetHostUsbAddr( RootHubDev[RootHubIndex].DeviceAddress );// Set the USB device address currently operated by the USB host
    SetUsbSpeed( RootHubDev[RootHubIndex].DeviceSpeed ​​); // Set the current USB speed
    RootHubId = RootHubIndex? 1: 0;
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
    UINT8 TransRetry;
    UINT8 r;
    UINT16 i;
    UH_RX_CTRL = tog;
    UH_TX_CTRL =tog;
    TransRetry = 0;
    do
    {
        UH_EP_PID = endp_pid; // Specify the token PID and destination endpoint number
        UIF_TRANSFER = 0; // Allow transmission
        for (i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -)
        {
            ;
        }
        UH_EP_PID = 0x00; // Stop USB transmission
        if (UIF_TRANSFER == 0)
        {
            return( ERR_USB_UNKNOWN );
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
                    if (U_TOG_OK)
                    {
                        return( ERR_SUCCESS );
                    }
                    if (r == USB_PID_ACK)
                    {
                        return( ERR_SUCCESS );
                    }
                    if (r == USB_PID_STALL || r == USB_PID_NAK)
                    {
                        return( r | ERR_USB_TRANSFER );
                    }
                    if (r)
                    {
                        return( r | ERR_USB_TRANSFER ); // Not a timeout/error, unexpected response
                    }
                    break; // Retry after timeout
                case USB_PID_IN:
                    if (U_TOG_OK)
                    {
                        return( ERR_SUCCESS );
                    }
                    if (tog? r == USB_PID_DATA1: r == USB_PID_DATA0)
                    {
                        return( ERR_SUCCESS );
                    }
                    if (r == USB_PID_STALL || r == USB_PID_NAK)
                    {
                        return( r | ERR_USB_TRANSFER );
                    }
                    if (r == USB_PID_DATA0 && r == USB_PID_DATA1 )// If out of sync, you need to discard and try again
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
            USB_INT_FG = 0xFF; //Clear interrupt flag
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
                   The total length of PUINT8 RetLen actually successfully sent and received is stored in the byte variable pointed to by RetLen
* Output: None
* Return: ERR_USB_BUF_OVER IN state phase error
                   ERR_SUCCESS Data exchange is successful
                   Other error status
************************************************** *****************************/
UINT8 HostCtrlTransfer( PUINT8 DataBuf, PUINT8 RetLen)
{
    UINT16 RemLen = 0;
    UINT8 s, RxLen, RxCnt, TxCnt;
    PUINT8 pBuf;
    PUINT8 xdata pLen;
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
    RemLen = (pSetupReq -> wLengthH << 8)|( pSetupReq -> wLengthL);
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
void CopySetupReqPkg( PUINT8C pReqPkt) // copy control transmission request packet
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
    UINT8 len;
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
* Function Name: CtrlSetUsbIDLE
* Description: Set USB device configuration
* Input: UINT8 cfg configuration value
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlSetUsbIDLE( UINT8 idle)
{
    CopySetupReqPkg( SetupSetUsbIDLE );
    pSetupReq -> wIndexL = idle; // USB device configuration
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
* Function Name: CtrlGetHIDDeviceReport
* Description: Get HID report descriptor
* Input: UINT8 Len report descriptor length
                   UINT8 IfNum interface number
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlGetHIDDeviceReport(UINT16 Len,UINT8 IfNum)
{
    CopySetupReqPkg( SetupGetHIDDevReport ); // Get interface report
    ((PUINT8X)pSetupReq)[ 6] = Len + 0x40; //Length +0x40
    ((PUINT8X)pSetupReq)[ 7] = (Len + 0x40)>>8; //
    ((PUINT8X)pSetupReq)[ 4] = IfNum; //
return( HostCtrlTransfer( TxBuffer, NULL) ); // Perform control transfer
}
/************************************************* ******************************
* Function Name: KM_HostSetReport
* Description: Set USB device configuration
* Input: UINT8 cfg configuration value
* Output: None
* Return:
************************************************** *****************************/
void KM_HostSetReport( UINT8 cfg) // Set USB device configuration
{
CopySetupReqPkg( SetupSetReport );
HostCtrlTransfer( &cfg, NULL ); // Perform control transfer
}

UINT8 KM_HostAnalyseMouseHid( UINT8 index, UINT8 num)
{
if( ((UINT8 *)TxBuffer)[0] == GLOB_USAGE_PAGE_DEF && ((UINT8 *)TxBuffer)[1] == GLOB_USAGE_PAGE_DESKTOP\
&& ((UINT8 *)TxBuffer)[2] == LOCAL_USAGE_DEF && ((UINT8 *)TxBuffer)[3] == LOCAL_USAGE_MOUSE ){
RootHubDev[index].Interface[num].DeviceType = DEV_TYPE_MOUSE;
return ERR_SUCCESS;
}
else if( ((UINT8 *)TxBuffer)[0] == GLOB_USAGE_PAGE_DEF && ((UINT8 *)TxBuffer)[1] == GLOB_USAGE_PAGE_DESKTOP\
&& ((UINT8 *)TxBuffer)[2] == LOCAL_USAGE_DEF && ((UINT8 *)TxBuffer)[3] == LOCAL_USAGE_KEY ){
RootHubDev[index].Interface[num].DeviceType = DEV_TYPE_KEYBOARD;
return ERR_SUCCESS;
}
return ERR_USB_UNSUPPORT;
}

/************************************************* ******************************
* Function Name: InitRootDevice
* Description: Initialize the USB device of the designated ROOT-HUB port
* Input: UINT8 RootHubIndex designated port, built-in HUB port number 0/1
* Output: None
* Return:
************************************************** *****************************/
UINT8 InitRootDevice( UINT8 RootHubIndex)
{
    UINT8 i, s, cfg, dv_cls, if_cls,ep;
    UINT8 retry=0;
    printf( "Reset root hub %1d# port\n", (UINT16)RootHubIndex );
    while(retry<10)
    {
        mDelaymS( 100*retry );
        retry++;
        ResetRootHubPort( RootHubIndex ); // After detecting the device, reset the USB bus of the corresponding port
        for (i = 0, s = 0; i <100; i ++) // wait for the USB device to reset and reconnect, 100mS timeout
        {
            mDelaymS( 1 );
            if (EnableRootHubPort( RootHubIndex) == ERR_SUCCESS) // Enable ROOT-HUB port
            {
                i = 0;
                s ++; // Time to wait for the stability of the USB device connection
                if (s> 10*retry)
                {
                    break; // 15mS has been connected stably
                }
            }
        }
        if (i) // device is not connected after reset
        {
            DisableRootHubPort( RootHubIndex );
            printf( "Disable root hub %1d# port because of disconnect\n", (UINT16)RootHubIndex );
// return( ERR_USB_DISCON );
            continue;
        }
        SelectHubPort( RootHubIndex, 0 );
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
            s = CtrlSetUsbAddress( RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL );// Set the USB device address, plus RootHubIndex can ensure that the 2 HUB ports are assigned different addresses
            if (s == ERR_SUCCESS)
            {
                RootHubDev[RootHubIndex].DeviceAddress = RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL; // save the USB address
                printf( "GetCfgDescr: ");
                s = CtrlGetConfigDescr( ); // Get configuration descriptor
if (s == ERR_SUCCESS ){
RootHubDev[RootHubIndex].InfCnt = ((PXUSB_CFG_DESCR)TxBuffer)->bNumInterfaces;//Number of interfaces
if( RootHubDev[RootHubIndex].InfCnt> 4) return( ERR_USB_OVER_IF ); //Exceeding 4 interfaces
cfg = ((PXUSB_CFG_DESCR)TxBuffer) -> bConfigurationValue;
#ifdef DEBUG
printf( "ifnum %02X ", (UINT16)RootHubDev[RootHubIndex].InfCnt );
for (i = 0; i <((PXUSB_CFG_DESCR)TxBuffer) -> wTotalLengthL; i ++ ){
printf( "x%02X ", (UINT16)( TxBuffer[i]) );
}
printf("\n");
#endif
//Analyze the configuration descriptor, get the endpoint data/each endpoint address/each endpoint size, etc., update the variables endp_addr and endp_size, etc.
if_cls = ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceClass; // Interface class code 03
if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_HID && ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceSubClass <= 0x01 )// It is a HID device, keyboard/mouse, etc.
{
s = 0;
for(i=0;i<RootHubDev[RootHubIndex].InfCnt;i++){
if(TxBuffer[s+1] == 0x02){//configuration descriptor
s = s + TxBuffer[s];
}
if(TxBuffer[s+1] == 0x04){//interface descriptor
ep = TxBuffer[s+4];
s = s + TxBuffer[s];
}
if(TxBuffer[s+1] == 0x21){//HID descriptor
s = s + TxBuffer[s];
RootHubDev[RootHubIndex].Interface[i].DescrLen = TxBuffer[s-1];
RootHubDev[RootHubIndex].Interface[i].DescrLen <<= 8;
RootHubDev[RootHubIndex].Interface[i].DescrLen |= TxBuffer[s-2]; // Save description length
}
if(TxBuffer[s+1] == 0x05){//endpoint descriptor
while(ep--){
if(TxBuffer[s+2]&0x80){//IN
RootHubDev[RootHubIndex].Interface[i].InAddr = TxBuffer[s+2]&0x0f;
RootHubDev[RootHubIndex].Interface[i].WaitUSB_IN_Interval = TxBuffer[s+6];
if((RootHubDev[RootHubIndex].DeviceSpeed ​​== 0)&&(RootHubDev[RootHubIndex].Interface[i].WaitUSB_IN_Interval<8))
{
RootHubDev[RootHubIndex].Interface[i].WaitUSB_IN_Interval = 8;
}
#ifdef DEBUG
printf( "IN_Ep %02X \n", (UINT16)( RootHubDev[RootHubIndex].Interface[i].InAddr) );
printf( "IN_Val %02X \n", (UINT16)( RootHubDev[RootHubIndex].Interface[i].WaitUSB_IN_Interval) );
#endif
s = s + TxBuffer[s];
continue;
}
if((TxBuffer[s+2]&0x80)==0){//OUT
RootHubDev[RootHubIndex].Interface[i].OutAddr = TxBuffer[s+2]&0x0f;
s = s + TxBuffer[s];
continue;
}
}
}
#ifdef DEBUG
printf( "hidlen %02X ", (UINT16)( RootHubDev[RootHubIndex].Interface[i].DescrLen) );
printf("\n");
#endif
}
s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
if (s == ERR_SUCCESS)
{
for(i=0;i<RootHubDev[RootHubIndex].InfCnt;i++){
CtrlSetUsbIDLE(i);
s = CtrlGetHIDDeviceReport(RootHubDev[RootHubIndex].Interface[i].DescrLen,i); //Get report descriptor
#ifdef DEBUG
printf( "len003 %02X \n", (UINT16)(RootHubDev[RootHubIndex].Interface[i].DescrLen) );
To
for (cfg=0; cfg<(RootHubDev[RootHubIndex].Interface[i].DescrLen); cfg++ ){
printf( "x%02X ", (UINT16)( TxBuffer[cfg]) );
}
printf("\n");
printf( "GetHIDReport: %02X ", (UINT16)s );
#endif
if( s == ERR_SUCCESS ){
s = KM_HostAnalyseMouseHid( RootHubIndex, i );// need analysis
}
else return s;
}
if((RootHubDev[0].Interface[0].DeviceType == DEV_TYPE_KEYBOARD)&&(RootHubDev[1].Interface[0].DeviceType == DEV_TYPE_KEYBOARD)){//Connect 2 keyboards
KM_HostSetReport(ReportData);//Sync LED
}
RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
return( ERR_SUCCESS );
}
}
else //other equipment
{
s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
if (s == ERR_SUCCESS)
{
RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
RootHubDev[RootHubIndex].DeviceType = dv_cls?dv_cls:if_cls;
return( ERR_SUCCESS );
}
}
                }
            }
        }
        printf( "InitRootDev Err = %02X\n", (UINT16)s );
        RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_FAILED;
        SetUsbSpeed( 1 ); // The default is full speed
        continue;
    }
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
    UINT8I s, RootHubIndex;
    printf( "EnumAllRootDev\n" );
    for (RootHubIndex = 0; RootHubIndex <2; RootHubIndex ++)
    {
        printf( "RootHubIndex %02x\n",(UINT16)RootHubIndex );
        if (RootHubDev[RootHubIndex].DeviceStatus == ROOT_DEV_CONNECTED) // The device just inserted has not been initialized
        {
            s = InitRootDevice( RootHubIndex ); // Initialize/enumerate the USB device of the specified HUB port
            if (s != ERR_SUCCESS)
            {
                return( s );
            }
        }
    }
    return( ERR_SUCCESS );
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
* Description: Initialize the USB host, query method
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitUSB_Host( void)
{
    UINT8 i;
    IE_USB = 0;
    USB_CTRL = bUC_HOST_MODE; // Set the mode first
    USB_DEV_AD = 0x00;
    UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN;
    UH_RX_DMA = RxBuffer;
    UH_TX_DMA = TxBuffer;
    UH_RX_CTRL = 0x00;
    UH_TX_CTRL = 0x00;
    USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB host and DMA, and automatically pause before the interrupt flag is cleared
    UH_SETUP = bUH_SOF_EN;
    USB_INT_FG = 0xFF; // Clear interrupt flag
    for (i = 0; i != 2; i ++)
    {
        DisableRootHubPort( i ); // Clear
    }
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
}

/************************************************* ******************************
* Function Name: RootHUB_Detect_USB_Plug
* Description: Detect the plug of ROOTHUB port equipment
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void RootHUB_Detect_USB_Plug( void)
{
UINT8 s;
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
}
/************************************************* ******************************
* Function Name: RootHUB_USB_Dev_Enum
* Description: enumerate the dev under ROOT HUB
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void RootHUB_USB_Dev_Enum( void)
{
UINT8 s;
s = ERR_SUCCESS;
if (FoundNewDev) // There is a new USB device inserted
{
FoundNewDev = 0;
mDelaymS( 200 ); // Since the USB device has just been inserted and is not yet stable, wait for hundreds of milliseconds for the USB device to eliminate the plugging jitter
s = EnumAllRootDevice( ); // Enumerate USB devices of all ROOT-HUB ports
if (s != ERR_SUCCESS)
{
printf( "EnumAllRootDev err = %02X\n", (UINT16)s );
}
}
}

/************************************************* ******************************
* Function Name: RootHUB_Get_Mouse_Data
* Description: Get the mouse data of the ROOT HUB button
* Input:
                   UINT8 RootHubIndex
                   UINT8 Dev_Type
                   PUINT8 MPort0 HUB0 port data, PUINT8 MPort1 HUB1 port data
                   MPortn is 0 means no data
* Output: None
* Return: None
************************************************** *****************************/
void RootHUB_Get_Data( UINT8 RootHubIndex,UINT8 Dev_Type,PUINT8 MPort0, PUINT8 MPort1)
{
UINT8 s,endp,i,flag;
flag = 0;
if((Dev_Type == DEV_TYPE_KEYBOARD)||(Dev_Type == DEV_TYPE_MOUSE))
{
for(i=0;i<RootHubDev[RootHubIndex].InfCnt;i++)
{
if (RootHubDev[RootHubIndex].Interface[i].DeviceType == Dev_Type && RootHubDev[RootHubIndex].DeviceStatus >= ROOT_DEV_SUCCESS && T0Count[RootHubIndex][i] >= RootHubDev[RootHubIndex].Interface[USB_INInterval].Wait
{
T0Count[RootHubIndex][i] = 0; //Clear the timer
SelectHubPort( RootHubIndex, 0 ); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
endp = RootHubDev[RootHubIndex].Interface[i].InAddr; // The address of the interrupt endpoint, bit 7 is used for the synchronization flag
if (endp & USB_ENDP_ADDR_MASK) // Endpoint is valid
{
s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80? bUH_R_TOG | bUH_T_TOG: 0, 0 ); // CH559 transfers transactions, gets data, NAK does not retry
if (s == ERR_SUCCESS)
{
endp ^= 0x80; // Synchronization flag flip
RootHubDev[RootHubIndex].Interface[i].InAddr = endp;
if (USB_RX_LEN) // received data length
{
if(RootHubIndex)
{
MPort1[0] = USB_RX_LEN;
memcpy(MPort1+1,RxBuffer,USB_RX_LEN); //HUB1 port data
}
else
{
MPort0[0] = USB_RX_LEN;
memcpy(MPort0+1,RxBuffer,USB_RX_LEN); //HUB0 port data
}
if(Dev_Type == DEV_TYPE_KEYBOARD)
{
if(RxBuffer[2]==0x39){//Caps Lock
ReportData ^= 0x02;
KM_HostSetReport(ReportData);
flag = 0xAA;
}
if(RxBuffer[2]==0x53){//Num Lock
ReportData ^= 0x01;
KM_HostSetReport(ReportData);
flag = 0xAA;
}
if(RxBuffer[2]==0x47){//Scroll Lock
ReportData ^= 0x04;
KM_HostSetReport(ReportData);
flag = 0xAA;
}
if((flag == 0xAA)&&(RootHubDev[0].Interface[0].DeviceType == DEV_TYPE_KEYBOARD)&&(RootHubDev[1].Interface[0].DeviceType == DEV_TYPE_KEYBOARD)){//Connect 2 Keyboard
s = RootHubIndex ^ 0x01;
SelectHubPort(s,0); // switch hub num
KM_HostSetReport(ReportData);
}
}
return; //The same HUB port compound multiple keyboards and mice to take only 1 device interface data
}
}
}
}
}
}
}
/************************************************* ******************************
* Function Name: main
* Description: main function
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
main()
{
    UINT8 i;
    UINT8 RootHub0[20];
    UINT8 RootHub1[20];
    mInitSTDIO( ); //In order to let the computer monitor the demonstration process through the serial port
    printf("CHIP_ID: %02x\n",(UINT16)ID_CH559);
printf("Host_Keyboard_Mouse_V100:"__DATE__",Debug time:"__TIME__"\n");
    InitUSB_Host( ); //Initialize the USB host
    mTimer0ClkFsys( ); //Set timer 0 1ms timing
    mTimer0ModSetup( 1 );
    mTimer0SetData( WAIT_USB_IN_COUNT );
    mTimer0RunCTL(T0_START);
To
To
    FoundNewDev = 0; //variable initialization
    memset(RootHubDev,0,sizeof(RootHubDev));
    memset(T0Count,0,sizeof(T0Count));
To
    while (1)
    {
RootHUB_Detect_USB_Plug( ); //Detect device plug
RootHUB_USB_Dev_Enum( ); //Enumerate devices under ROOT
To
RootHubId = 0;
memset(RootHub0,0,sizeof(RootHub0));
memset(RootHub1,0,sizeof(RootHub1));
RootHUB_Get_Data( RootHubId,DEV_TYPE_MOUSE,RootHub0,RootHub1);
if(RootHub0[0]) //HUB0 Mouse data
{
printf("HUB0_Mouse data: ");
for (i = 1; i <= RootHub0[0]; i ++)
{
printf("x%02X ",(UINT16)(RootHub0[i]) );
}
printf("\n");
}
memset(RootHub0,0,sizeof(RootHub0));
memset(RootHub1,0,sizeof(RootHub1));
RootHUB_Get_Data( RootHubId,DEV_TYPE_KEYBOARD,RootHub0,RootHub1);
if(RootHub0[0]) //HUB0 Keyboard data
{
printf("HUB0_Keyboard data: ");
for (i = 1; i <= RootHub0[0]; i ++)
{
printf("x%02X ",(UINT16)(RootHub0[i]) );
}
printf("\n");
}
RootHubId = 1;
memset(RootHub0,0,sizeof(RootHub0));
memset(RootHub1,0,sizeof(RootHub1));
RootHUB_Get_Data( RootHubId,DEV_TYPE_MOUSE,RootHub0,RootHub1);
if(RootHub1[0]) //HUB1 Mouse data
{
printf("HUB1_Mouse data: ");
for (i = 1; i <= RootHub1[0]; i ++)
{
printf("x%02X ",(UINT16)(RootHub1[i]) );
}
printf("\n");
}
memset(RootHub0,0,sizeof(RootHub0));
memset(RootHub1,0,sizeof(RootHub1));
RootHUB_Get_Data( RootHubId,DEV_TYPE_KEYBOARD,RootHub0,RootHub1);
if(RootHub1[0]) //HUB1 Keyboard data
{
printf("HUB1_Keyboard data: ");
for (i = 1; i <= RootHub1[0]; i ++)
{
printf("x%02X ",(UINT16)(RootHub1[i]) );
}
printf("\n");
}
To
SetUsbSpeed( 1 ); // The default is full speed
mTimer0Interrupt( ); // 1ms timing
    }
}
