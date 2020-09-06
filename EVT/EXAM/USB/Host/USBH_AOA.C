/********************************** (C) COPYRIGHT *********** ********************
* File Name: USBH_AOA.C
* Author: WCH
* Version: V1.0
* Date: 2018/08/01
* Description:
 USB host example for CH559, start USB device under DP/DM and HP/HM port
 Support the accessory mode (AOA) and communication of Android phones, configure the phone APP to realize two-way data transmission and reception, and support the enumeration of ordinary devices.
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
#define ERR_USB_UNSUPPORT 0xFB /*USB device not supported*/
#define ERR_USB_UNKNOWN 0xFE /*Device operation error*/
#define ERR_AOA_PROTOCOL 0x41 /*Protocol version error */

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
/*Get HUB descriptor*/
UINT8C SetupGetHubDescr[] = {HUB_GET_HUB_DESCRIPTOR, HUB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_HUB, 0x00, 0x00, sizeof( USB_HUB_DESCR ), 0x00 };
/*Get HID device report descriptor*/
UINT8C SetupGetHIDDevReport[] = {0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x41, 0x00 };
//AOA gets the protocol version
UINT8C GetProtocol[] = {0xc0,0x33,0x00,0x00,0x00,0x00,0x02,0x00 };
//Start accessory mode
UINT8C TouchAOAMode[] = {0x40,0x35,0x00,0x00,0x00,0x00,0x00,0x00 };
/* AOA related array definition */
UINT8C Sendlen[]= {0,4,16,35,39,53,67};
//String ID, string information related to the mobile APP
UINT8C StringID[] = {'W','C','H',0x00, //manufacturer name
                      'W','C','H','U','A','R','T','D','e','m','o',0x00, //model name
                      0x57,0x43,0x48,0x20,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x20,0x54,0x65,0x73,0x74,0x00, //description
                      '1','.','0',0x00, //version
                      0x68,0x74,0x74,0x70,0x3a,0x2f,0x2f,0x77,0x63,0x68,0x2e,0x63,0x6e,0,//URI
                      0x57,0x43,0x48,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x31,0x00 //serial number
                     };
//Apply index string command
UINT8C SetStringID[]= {0x40,0x34,0x00,0x00,0x00,0x00,0x04,0x00,
                        0x40,0x34,0x00,0x00,0x01,0x00,12,0x00,
                        0x40,0x34,0x00,0x00,0x02,0x00,19,0x00,
                        0x40,0x34,0x00,0x00,0x03,0x00,4,0x00,
                        0x40,0x34,0x00,0x00,0x04,0x00,0x0E,0x00,
                        0x40,0x34,0x00,0x00,0x05,0x00,0x0E,0x00
                       };

UINT8X UsbDevEndp0Size; /* The maximum packet size of endpoint 0 of the USB device */
/*USB device related information table, CH559 supports up to 2 devices*/
#define ROOT_DEV_DISCONNECT 0
#define ROOT_DEV_CONNECTED 1
#define ROOT_DEV_FAILED 2
#define ROOT_DEV_SUCCESS 3
#define DEV_TYPE_KEYBOARD (USB_DEV_CLASS_HID | 0x20)
#define DEV_TYPE_MOUSE (USB_DEV_CLASS_HID | 0x30)
#define DEF_AOA_DEVICE 0xF0

struct _RootHubDev
{
    UINT8 DeviceStatus; // Device status, 0-no device, 1-device but not initialized, 2-device but initialization enumeration failed, 3-device and initialization enumeration succeeded
    UINT8 DeviceAddress; // The assigned USB address of the device
    UINT8 DeviceSpeed; // 0 is low speed, non-zero is full speed
    UINT8 DeviceType; // Device type
UINT16 DeviceVID; // Device VID
UINT16 DevicePID; // Device PID
    UINT8 GpVar[4]; // General variable
} xdata RootHubDev[2];
#define HUB_MAX_PORTS 4
/*
Convention: USB device address allocation rules (refer to USB_DEVICE_ADDR)
Address value Device location
0x02 USB device or external HUB under built-in Root-HUB0
0x03 USB device or external HUB under built-in Root-HUB1
0x1x USB device under port x of the external HUB under built-in Root-HUB0, x is 1~n
0x2x USB device under port x of the external HUB under built-in Root-HUB1, x is 1~n
*/
struct _DevOnHubPort
{
    UINT8 DeviceStatus; // Device status, 0-no device, 1-device but not initialized, 2-device but initialization enumeration failed, 3-device and initialization enumeration succeeded
    UINT8 DeviceAddress; // The assigned USB address of the device
    UINT8 DeviceSpeed; // 0 is low speed, non-zero is full speed
    UINT8 DeviceType; // Device type
//..... struct _Endp_Attr Endp_Attr[4]; //The attributes of the endpoint, supports up to 4 endpoints
    UINT8 GpVar; // General variable
} xdata DevOnHubPort[2][HUB_MAX_PORTS]; // Assumption: no more than 2 external HUBs, and no more than HUB_MAX_PORTS ports for each external HUB (no matter if there are more)

UINT8X RxBuffer[ MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[ MAX_PACKET_SIZE] _at_ 0x0040; // OUT, must even address
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
UINT8X COM_BUF[200]; //Open up a shared buffer to store descriptors exceeding 64 bytes
bit RootHubId; // The currently operating root-hub port number: 0=HUB0,1=HUB1
bit FoundNewDev;
#pragma NOAREGS
#define WAIT_USB_TOUT_200US 200 // Waiting for USB interrupt timeout time 200uS@Fsys=12MHz
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
UINT8 CtrlGetHubDescr( void ); // Get the HUB descriptor and return it in TxBuffer
UINT8 HubGetPortStatus( UINT8 HubPortIndex ); // Query the status of the HUB port and return it in TxBuffer
UINT8 HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt ); // Set HUB port characteristics
UINT8 HubClearPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt ); // Clear HUB port feature
UINT8 AnalyzeHidIntEndp( PUINT8X buf ); // Analyze the address of the HID interrupt endpoint from the descriptor
UINT8 AnalyzeBulkEndp( PUINT8X buf, UINT8 RootHubIndex) ;//Analyze the bulk endpoint under RootHUB
UINT8 TouchStartAOA(void); // try to start AOA mode
UINT8 InitRootDevice( UINT8 RootHubIndex ); // Initialize the USB device of the designated ROOT-HUB port
// Input: Built-in HUB port number 0/1
UINT8 EnumAllRootDevice( void ); // Enumerate all USB devices of ROOT-HUB port
UINT8 InitDevOnHub( UINT8 RootHubIndex, UINT8 HubPortIndex ); // Initialize the secondary USB device after enumerating external HUB
UINT8 EnumHubPort( UINT8 RootHubIndex ); // Enumerate each port of the external HUB hub on the specified ROOT-HUB port, check whether each port is connected or remove the event and initialize the secondary USB device
UINT8 EnumAllHubPort( void ); // Enumerate all secondary USB devices behind the external HUB under the ROOT-HUB port
UINT16 SearchTypeDevice( UINT8 type ); // Search for the port number where the device of the specified type is located on each port of ROOT-HUB and external HUB. If the output port number is 0xFFFF, it is not found
// The upper 8 bits of the output are the ROOT-HUB port number, the lower 8 bits are the port number of the external HUB, and the lower 8 bits are 0, the device is directly on the ROOT-HUB port
void mInitSTDIO( void ); // Initialize the serial port for printf and getkey input and output
void InitUSB_Host( void ); // Initialize the USB host
//#define FREQ_SYS 12000000 // System frequency is 12MHz
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
RootHubDev[ RootHubIndex ].DeviceAddress = 0x00;
if (RootHubIndex == 1)
{
UHUB1_CTRL = 0x00; // Clear the control data related to HUB1, it is not actually necessary
}
else
{
UHUB0_CTRL = 0x00; // Clear the control data related to HUB0, actually does not need to be cleared
}
To
    SetUsbSpeed( 1 ); // The default is full speed
    if (RootHubIndex == 1)
    {
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
        mDelaymS( 15 ); // reset time 10mS to 20mS
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_BUS_RESET; // end reset
    }
    else
    {
        UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// The default is full speed, start reset
        mDelaymS( 15 ); // reset time 10mS to 20mS
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
    if (HubPortIndex) // Select the designated port of the external HUB that operates the designated ROOT-HUB port
    {
        SetHostUsbAddr( DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceAddress );// Set the USB device address currently operated by the USB host
        SetUsbSpeed( DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceSpeed ​​);// Set the current USB speed
        if (DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceSpeed ​​== 0 )// The front ID is required to communicate with low-speed USB devices through an external HUB
        {
            UH_SETUP |= bUH_PRE_PID_EN; // enable PRE PID
            mDelayuS(100);
        }
    }
    else // Select the ROOT-HUB port specified by the operation
    {
        SetHostUsbAddr( RootHubDev[RootHubIndex].DeviceAddress );// Set the USB device address currently operated by the USB host
        SetUsbSpeed( RootHubDev[RootHubIndex].DeviceSpeed ​​); // Set the current USB speed
    }
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
    UINT8 TransRetry;
// #define TransRetry UEP0_T_LEN // Save memory
    UINT8 r;
    UINT16 i;
    UH_RX_CTRL = tog;
    UH_TX_CTRL =tog;
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
// if (s != ERR_SUCCESS) return( s ); // interrupt timeout, it may be a hardware exception
        if (UIF_TRANSFER == 0)
        {
            return( ERR_USB_UNKNOWN );
        }
// if (UIF_DETECT) // USB device plug-in event
// {
// // mDelayuS( 200 ); // wait for the transfer to complete
// // UIF_DETECT = 0; // Clear interrupt flag
// s = AnalyzeRootHub( ); // Analyze ROOT-HUB status
// if (s == ERR_USB_CONNECT)
// {
// FoundNewDev = 1;
//}
// if ((RootHubDev[RootHubId].DeviceStatus == ROOT_DEV_DISCONNECT)||(CH559DiskStatus == DISK_DISCONNECT))
// {
// return( ERR_USB_DISCON ); // USB device disconnect event
//}
// if ((RootHubDev[RootHubId].DeviceStatus == ROOT_DEV_CONNECTED)||(CH559DiskStatus == DISK_CONNECT))
// {
// return( ERR_USB_CONNECT ); // USB device connection event
//}
// // if ((USB_HUB_ST & bUHS_H0_ATTACH) == 0x00) return( ERR_USB_DISCON );// USB device disconnect event
// mDelayuS( 200 ); // Wait for the transfer to complete
//}
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
    s = HostCtrlTransfer( COM_BUF, &len ); // execute control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    UsbDevEndp0Size = ((PXUSB_DEV_DESCR)COM_BUF) -> bMaxPacketSize0;// The maximum packet length of endpoint 0, which is simplified processing. Normally, you should get the first 8 bytes and immediately update UsbDevEndp0Size before continuing
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
    s = HostCtrlTransfer( COM_BUF, &len ); // execute control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetCfgDescr) -> wLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Return length error
    }
    len = ((PXUSB_CFG_DESCR)COM_BUF) -> wTotalLengthL;
    CopySetupReqPkg( SetupGetCfgDescr );
    pSetupReq -> wLengthL = len; // The total length of the complete configuration descriptor
    s = HostCtrlTransfer( COM_BUF, &len ); // Perform control transfer (at this time it may exceed 64 bytes, stored in COM_BUF)
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetCfgDescr) -> wLengthL || len <((PXUSB_CFG_DESCR)COM_BUF) -> wTotalLengthL)
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
* Function Name: CtrlSetUsbIntercace
* Description: Set the USB device interface
* Input: UINT8 cfg configuration value
* Output: None
* Return: ERR_SUCCESS success
                   other
************************************************** *****************************/
UINT8 CtrlSetUsbIntercace( UINT8 cfg)
{
    CopySetupReqPkg( SetupSetUsbInterface );
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
* Function Name: CtrlGetHIDDeviceReport
* Description: Get the HID device report descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: ERR_SUCCESS success
                   Other errors
************************************************** *****************************/
UINT8 CtrlGetHIDDeviceReport( void)
{
    UINT8 s;
    UINT8 len;
    UINT8 tmp[]= {0x21,0x0a,0x00,0x00,0x00,0x00,0x00,0x00};
    for (s = 0; s != sizeof( tmp ); s ++)
    {
        ((PUINT8X)pSetupReq)[ s] = tmp[s];
    }
    s = HostCtrlTransfer( COM_BUF, &len ); // execute control transfer
// if (s != ERR_SUCCESS)
// {
// return( s );
//}
    CopySetupReqPkg( SetupGetHIDDevReport );
    s = HostCtrlTransfer( COM_BUF, &len ); // execute control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Descriptor length error
    }
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: CtrlGetHubDescr
* Description: Get the HUB descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: ERR_SUCCESS success
                   ERR_USB_BUF_OVER Length error
************************************************** *****************************/
UINT8 CtrlGetHubDescr( void)
{
    UINT8 s;
    UINT8D len;
    CopySetupReqPkg( SetupGetHubDescr );
    s = HostCtrlTransfer( TxBuffer, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <((PUSB_SETUP_REQ)SetupGetHubDescr) -> wLengthL)
    {
        return( ERR_USB_BUF_OVER ); // Descriptor length error
    }
// if (len <4) return( ERR_USB_BUF_OVER ); // Descriptor length error
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: HubGetPortStatus
* Description: Query the status of the HUB port and return it in TxBuffer
* Input: UINT8 HubPortIndex
* Output: None
* Return: ERR_SUCCESS success
                   ERR_USB_BUF_OVER Length error
************************************************** *****************************/
UINT8 HubGetPortStatus( UINT8 HubPortIndex)
{
    UINT8 s;
    UINT8D len;
    pSetupReq -> bRequestType = HUB_GET_PORT_STATUS;
    pSetupReq -> bRequest = HUB_GET_STATUS;
    pSetupReq -> wValueL = 0x00;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x04;
    pSetupReq -> wLengthH = 0x00;
    s = HostCtrlTransfer( TxBuffer, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    if (len <4)
    {
        return( ERR_USB_BUF_OVER ); // Descriptor length error
    }
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: HubSetPortFeature
* Description: Set HUB port characteristics
* Input: UINT8 HubPortIndex //HUB port
                   UINT8 FeatureSelt //HUB port feature
* Output: None
* Return: ERR_SUCCESS success
                   Other errors
************************************************** *****************************/
UINT8 HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt)
{
    pSetupReq -> bRequestType = HUB_SET_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_SET_FEATURE;
    pSetupReq -> wValueL = FeatureSelt;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x00;
    pSetupReq -> wLengthH = 0x00;
    return( HostCtrlTransfer( NULL, NULL) ); // Perform control transfer
}
/************************************************* ******************************
* Function Name: HubClearPortFeature
* Description: Clear HUB port characteristics
* Input: UINT8 HubPortIndex //HUB port
                   UINT8 FeatureSelt //HUB port feature
* Output: None
* Return: ERR_SUCCESS success
                   Other errors
************************************************** *****************************/
UINT8 HubClearPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt)
{
    pSetupReq -> bRequestType = HUB_CLEAR_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_CLEAR_FEATURE;
    pSetupReq -> wValueL = FeatureSelt;
    pSetupReq -> wValueH = 0x00;
    pSetupReq -> wIndexL = HubPortIndex;
    pSetupReq -> wIndexH = 0x00;
    pSetupReq -> wLengthL = 0x00;
    pSetupReq -> wLengthH = 0x00;
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
    for (i = 0; i <((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) // search interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    {
        if (((PXUSB_ENDP_DESCR)(buf+i)) -> bDescriptorType == USB_DESCR_TYP_ENDP // is the endpoint descriptor
                && (((PXUSB_ENDP_DESCR)(buf+i)) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_INTER// is the interrupt endpoint
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
* Function Name: AnalyzeBulkEndp, only used to analyze RootHub
* Description: Analyze bulk endpoints, GpVar[0], GpVar[1] store upload endpoints. GpVar[2], GpVar[3] store download endpoints
* Input: buf: the address of the data buffer to be analyzed. HubPortIndex: 0 means the root HUB, non-zero means the port number under the external HUB
* Output: None
* Return: 0
************************************************** *****************************/
UINT8 AnalyzeBulkEndp( PUINT8X buf,UINT8 RootHubIndex)
{
    UINT8 i, s1,s2, l;
    s1 = 0; s2 = 2;

memset( RootHubDev[RootHubIndex].GpVar,0,sizeof(RootHubDev[RootHubIndex].GpVar) ); //clear the array

    for (i = 0; i <((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) // search interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    {
        if ((( (PXUSB_ENDP_DESCR)(buf+i)) -> bDescriptorType == USB_DESCR_TYP_ENDP) // is the endpoint descriptor
                && ((( (PXUSB_ENDP_DESCR)(buf+i)) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_BULK)) // is a batch

        {

if(( (PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_DIR_MASK)
RootHubDev[RootHubIndex].GpVar[s1++] = ((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
else
RootHubDev[RootHubIndex].GpVar[s2++] = ((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
To
if(s1 == 2) s1 = 1;
if(s2 == 4) s2 = 3;
}
        l = ((PXUSB_ENDP_DESCR)(buf+i)) -> bLength; // current descriptor length, skip
        if (l> 16)
        {
            break;
        }
    }
    return( 0 );
}

//Try to start AOA mode
UINT8 TouchStartAOA(void)
{
UINT8 len,s,i,Num;
    //Get the protocol version number
    CopySetupReqPkg( GetProtocol );
    s = HostCtrlTransfer( COM_BUF, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
if(COM_BUF[0]<2) return ERR_AOA_PROTOCOL;

    //Output string
    for(i=0; i<6; i++)
    {
        Num=Sendlen[i];
        CopySetupReqPkg(&SetStringID[8*i]);
        s = HostCtrlTransfer( &StringID[Num], &len ); // execute control transfer
        if (s != ERR_SUCCESS)
        {
            return( s );
        }
    }

    CopySetupReqPkg(TouchAOAMode);
    s = HostCtrlTransfer( COM_BUF, &len ); // Perform control transfer
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    return ERR_SUCCESS;
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
    UINT8 i, s, cfg, dv_cls, if_cls;
UINT8 touchaoatm = 0;
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
                printf( "x%02X ", (UINT16)( COM_BUF[i]) );
            }
            printf( "\n" ); // display the descriptor
To
RootHubDev[RootHubIndex].DeviceVID=(((UINT16)((PXUSB_DEV_DESCR)COM_BUF)->idVendorH)<<8) + ((PXUSB_DEV_DESCR)COM_BUF)->idVendorL; //Save VID PID information
RootHubDev[RootHubIndex].DevicePID=(((UINT16)((PXUSB_DEV_DESCR)COM_BUF)->idProductH)<<8) + ((PXUSB_DEV_DESCR)COM_BUF)->idProductL;
            dv_cls = ((PXUSB_DEV_DESCR)COM_BUF) -> bDeviceClass; // device class code
            s = CtrlSetUsbAddress( RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL );// Set the USB device address, plus RootHubIndex can ensure that the 2 HUB ports are assigned different addresses
            if (s == ERR_SUCCESS)
            {
                RootHubDev[RootHubIndex].DeviceAddress = RootHubIndex + ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL; // save the USB address
                printf( "GetCfgDescr: ");
                s = CtrlGetConfigDescr( ); // Get configuration descriptor
                if (s == ERR_SUCCESS)
                {
                    cfg = ((PXUSB_CFG_DESCR)COM_BUF) -> bConfigurationValue;
                    for (i = 0; i <((PXUSB_CFG_DESCR)COM_BUF) -> wTotalLengthL; i ++)
                    {
                        printf( "x%02X ", (UINT16)( COM_BUF[i]) );
                    }
                    printf("\n");
                    //Analyze the configuration descriptor, get the endpoint data/each endpoint address/each endpoint size, etc., update the variables endp_addr and endp_size, etc.
                    if_cls = ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceClass; // interface class code
                    if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE )// It is a USB storage device, basically confirm it is a U disk
                    {
                        s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                        if (s == ERR_SUCCESS)
                        {
                            RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
                            RootHubDev[RootHubIndex].DeviceType = USB_DEV_CLASS_STORAGE;
                            printf( "USB-Disk Ready\n" );
                            SetUsbSpeed( 1 ); // The default is full speed
                            return( ERR_SUCCESS );
                        }
                    }
                    else if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_HID && ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceSubClass <= 0x01 )// It is a HID device, keyboard/mouse, etc.
                    {
                        s = AnalyzeHidIntEndp( COM_BUF ); // Analyze the address of the HID interrupt endpoint from the descriptor
                        RootHubDev[RootHubIndex].GpVar[0] = s & USB_ENDP_ADDR_MASK ;// Save the address of the interrupt endpoint, bit 7 is used to synchronize the flag bit, cleared to 0
                        if_cls = ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceProtocol;
                        s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                        printf( "HID: %02x",(UINT16)s );
                        if (s == ERR_SUCCESS)
                        {
                            printf( "GetHIDReport: ");
                            s = CtrlGetHIDDeviceReport(); //Get report descriptor
                            if(s == ERR_SUCCESS)
                            {
                                for (i = 0; i <64; i++)
                                {
                                    printf( "x%02X ", (UINT16)( COM_BUF[i]) );
                                }
                                printf("\n");
                            }
                            //Set_Idle( );
                            //The endpoint information needs to be saved for the main program to perform USB transfer
                            RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
                            CtrlSetUsbIDLE(0x0a);
                            if (if_cls == 1)
                            {
                                RootHubDev[RootHubIndex].DeviceType = DEV_TYPE_KEYBOARD;
                                //Further initialization, such as device keyboard indicator LED, etc.
                                printf( "USB-Keyboard Ready\n" );
                                SetUsbSpeed( 1 ); // The default is full speed
                                return( ERR_SUCCESS );
                            }
                            else if (if_cls == 2)
                            {
                                RootHubDev[RootHubIndex].DeviceType = DEV_TYPE_MOUSE;
                                //In order to query the mouse status in the future, the descriptor should be analyzed to obtain the address and length of the interrupt port
                                printf( "USB-Mouse Ready\n" );
                                SetUsbSpeed( 1 ); // The default is full speed
                                return( ERR_SUCCESS );
                            }
                            s = ERR_USB_UNSUPPORT;
                        }
                    }
                    else if (dv_cls == USB_DEV_CLASS_HUB) // is a HUB device, hub, etc.
                    {
                        s = AnalyzeHidIntEndp( COM_BUF ); // Analyze the address of the HID interrupt endpoint from the descriptor
                        RootHubDev[RootHubIndex].GpVar[1] = s & USB_ENDP_ADDR_MASK ;// Save the address of the interrupt endpoint, bit 7 is used to synchronize the flag bit, cleared to 0
                        printf( "GetHubDescr: ");
                        s = CtrlGetHubDescr( );
                        if (s == ERR_SUCCESS)
                        {
                            for( i = 0; i <TxBuffer[0]; i++)
                            {
                                printf( "x%02X ",(UINT16)(TxBuffer[i]) );
                            }
                            printf("\n");
                            RootHubDev[RootHubIndex].GpVar[0] = ((PXUSB_HUB_DESCR)TxBuffer) -> bNbrPorts;// Save the number of HUB ports
                            if (RootHubDev[RootHubIndex].GpVar[0]> HUB_MAX_PORTS)
                            {
                                RootHubDev[RootHubIndex].GpVar[0] = HUB_MAX_PORTS;// Because when defining the structure DevOnHubPort, it is artificially assumed that each HUB does not exceed HUB_MAX_PORTS ports
                            }
                            //if (((PXUSB_HUB_DESCR)TxBuffer) -> wHubCharacteristics[0] & 0x04) printf("Composite device with hub\n");
                            //else printf("Single hub product\n");
                            s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                            if (s == ERR_SUCCESS)
                            {
                                RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
                                RootHubDev[RootHubIndex].DeviceType = USB_DEV_CLASS_HUB;
                                //The endpoint information needs to be saved for the main program to perform USB transmission. The interrupt endpoint can be used for HUB event notification, but this program uses query status control transmission instead
                                //Power on each port of the HUB, query the status of each port, initialize the HUB port with the device connected, and initialize the device
                                for (i = 1; i <= RootHubDev[RootHubIndex].GpVar[0]; i ++ )// Power on all ports of the HUB
                                {
                                    DevOnHubPort[RootHubIndex][i-1].DeviceStatus = ROOT_DEV_DISCONNECT; // Clear the status of the device on the external HUB port
                                    s = HubSetPortFeature( i, HUB_PORT_POWER );
                                    if (s != ERR_SUCCESS)
                                    {
                                        printf( "Ext-HUB Port_%1d# power on error\n",(UINT16)i );// Port power on failure
                                    }
                                }
// for (i = 1; i <= RootHubDev[RootHubIndex].GpVar[0]; i ++) // Clear the connection status of each port of the HUB
// {
// s = HubClearPortFeature( i, HUB_C_PORT_CONNECTION );
// if (s != ERR_SUCCESS)
// {
// printf( "Ext-HUB Port_%1d# clear connection error\n",(UINT16)i );// Port connection status clearing failed
//}
//}
                                for (i = 1; i <= RootHubDev[RootHubIndex].GpVar[0]; i ++) // Query the connection status of each HUB port
                                {
                                    s = HubGetPortStatus( i ); // Get port status
                                    if (s != ERR_SUCCESS)
                                    {
                                        printf( "Ext-HUB Port_%1d# clear connection error\n",(UINT16)i ); // port connection status clearing failed
                                    }
                                }
                                SetUsbSpeed( 1 ); // The default is full speed
                                return( ERR_SUCCESS );
                            }
                        }
                    }
                    else // can be further analyzed, it may be a mobile phone
                    {
                        printf("dv_cls=%02x,if_cls =%02x\n",(UINT16)dv_cls,(UINT16)if_cls);
AnalyzeBulkEndp(COM_BUF, RootHubIndex );
for(i=0;i!=4;i++)
{
printf("%02x ",(UINT16)RootHubDev[RootHubIndex].GpVar[i] );
}
printf("\n");

s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
                        if (s == ERR_SUCCESS)
                        {
printf("vid pid:%02x %02x\n",(UINT16)RootHubDev[RootHubIndex].DeviceVID,(UINT16)RootHubDev[RootHubIndex].DevicePID);
To
if((RootHubDev[RootHubIndex].DeviceVID==0x18D1)&&(RootHubDev[RootHubIndex].DevicePID&0xff00)==0x2D00) //If it is an AOA accessory
{
printf("AOA Mode\n");
RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
RootHubDev[RootHubIndex].DeviceType = DEF_AOA_DEVICE; //This is just a custom variable class, not a USB protocol class
SetUsbSpeed( 1 ); // The default is full speed
return( ERR_SUCCESS );
}
else //If it is not AOA accessory mode, try to activate accessory mode.
{
s = TouchStartAOA();
if( s == ERR_SUCCESS)
{
if(touchaoatm<3) // limit the number of attempts to AOA start
{
touchaoatm++;
mDelaymS(500); //Some Android devices automatically disconnect and reconnect, so it’s better to have a delay here
continue; //In fact, there is no need to jump here. The AOA protocol stipulates that the device will automatically reconnect to the bus.
}
//If you reach this point, it may not support AOA or other devices
RootHubDev[RootHubIndex].DeviceType = dv_cls? Dv_cls: if_cls;
RootHubDev[RootHubIndex].DeviceStatus = ROOT_DEV_SUCCESS;
SetUsbSpeed( 1 ); // The default is full speed
return( ERR_SUCCESS ); // The unknown device is initialized successfully
}
}

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
* Function Name: InitDevOnHub
* Description: Initialize the secondary USB device after enumerating the external HUB
* Input: UINT8 RootHubIndex specifies ROOT_HUB
                   UINT8 HubPortIndex specifies the external HUB
* Output: None
* Return: ERR_SUCCESS success
                   ERR_USB_UNKNOWN unknown device
************************************************** *****************************/
UINT8 InitDevOnHub( UINT8 RootHubIndex, UINT8 HubPortIndex)
{
    UINT8 i, s, cfg, dv_cls, if_cls;
    printf( "Init dev @ExtHub-port_%1d ", (UINT16)HubPortIndex );
    printf( "@Root_%1d\n", (UINT16)RootHubIndex );
    if (HubPortIndex == 0)
    {
        return( ERR_USB_UNKNOWN );
    }
    SelectHubPort( RootHubIndex, HubPortIndex ); // Select the designated port of the external HUB that operates the designated ROOT-HUB port, and select the speed
    printf( "UH_SETUP = GetDevDescr: %02x",(UINT16)UH_SETUP );
    s = CtrlGetDeviceDescr( ); // Get device descriptor
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    dv_cls = ((PXUSB_DEV_DESCR)COM_BUF) -> bDeviceClass; // device class code
    cfg = ((RootHubIndex+1)<<4) + HubPortIndex; // Calculate a USB address to avoid address overlap
    s = CtrlSetUsbAddress( cfg ); // Set USB device address
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceAddress = cfg; // Save the assigned USB address
    printf( "GetCfgDescr: ");
    s = CtrlGetConfigDescr( ); // Get configuration descriptor
    if (s != ERR_SUCCESS)
    {
        return( s );
    }
    cfg = ((PXUSB_CFG_DESCR)COM_BUF) -> bConfigurationValue;
    for (i = 0; i <((PXUSB_CFG_DESCR)COM_BUF) -> wTotalLengthL; i ++)
    {
        printf( "x%02X ", (UINT16)( COM_BUF[i]) );
    }
    printf("\n");
    /* Analyze the configuration descriptor, get the endpoint data/each endpoint address/each endpoint size, etc., update the variables endp_addr and endp_size, etc. */
    if_cls = ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceClass; // interface class code
    if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE) // It is a USB storage device, basically confirm it is a U disk
    {
        s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
        if (s == ERR_SUCCESS)
        {
            DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceStatus = ROOT_DEV_SUCCESS;
            DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType = USB_DEV_CLASS_STORAGE;
            printf( "USB-Disk Ready\n" );
            SetUsbSpeed( 1 ); // The default is full speed
            return( ERR_SUCCESS );
        }
    }
    else if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_HID && ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceSubClass <= 0x01) // It is a HID device, keyboard/mouse, etc.
    {
        s = AnalyzeHidIntEndp( COM_BUF ); // Analyze the address of the HID interrupt endpoint from the descriptor
        DevOnHubPort[RootHubIndex][HubPortIndex-1].GpVar = s; // Save the address of the interrupt endpoint, bit 7 is used for the synchronization flag, cleared to 0
        if_cls = ((PXUSB_CFG_DESCR_LONG)COM_BUF) -> itf_descr.bInterfaceProtocol;
        s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
        if (s == ERR_SUCCESS)
        {
            //The endpoint information needs to be saved for the main program to perform USB transfer
            DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceStatus = ROOT_DEV_SUCCESS;
            if (if_cls == 1)
            {
                DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType = DEV_TYPE_KEYBOARD;
                //Further initialization, such as device keyboard indicator LED, etc.
                printf( "USB-Keyboard Ready\n" );
                SetUsbSpeed( 1 ); // The default is full speed
                return( ERR_SUCCESS );
            }
            else if (if_cls == 2)
            {
                DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType = DEV_TYPE_MOUSE;
                //In order to query the mouse status in the future, the descriptor should be analyzed to obtain the address, length and other information of the interrupt port
                printf( "USB-Mouse Ready\n" );
                SetUsbSpeed( 1 ); // The default is full speed
                return( ERR_SUCCESS );
            }
            s = ERR_USB_UNSUPPORT;
        }
    }
    else if (dv_cls == USB_DEV_CLASS_HUB) // is a HUB device, hub, etc.
    {
        DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType = USB_DEV_CLASS_HUB;
        printf( "This program don't support Level 2 HUB\n"); // Need to support multi-level HUB cascade, please refer to this program to expand
        s = HubClearPortFeature( i, HUB_PORT_ENABLE ); // Disable HUB port
        if (s != ERR_SUCCESS)
        {
            return( s );
        }
        s = ERR_USB_UNSUPPORT;
    }
    else // can be further analyzed
    {
        s = CtrlSetUsbConfig( cfg ); // Set USB device configuration
        if (s == ERR_SUCCESS)
        {
            //The endpoint information needs to be saved for the main program to perform USB transfer
            DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceStatus = ROOT_DEV_SUCCESS;
            DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType = dv_cls? Dv_cls: if_cls;
            SetUsbSpeed( 1 ); // The default is full speed
            return( ERR_SUCCESS ); //Unknown device initialized successfully
        }
    }
    printf( "InitDevOnHub Err = %02X\n", (UINT16)s );
    DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceStatus = ROOT_DEV_FAILED;
    SetUsbSpeed( 1 ); // The default is full speed
    return( s );
}
/************************************************* ******************************
* Function Name: EnumHubPort
* Description: Enumerate each port of the external HUB hub on the specified ROOT-HUB port, check whether each port is connected or removed, and initialize the secondary USB device
* Input: UINT8 RootHubIndex ROOT_HUB0 and ROOT_HUB1
* Output: None
* Return: ERR_SUCCESS success
                   Other failure
************************************************** *****************************/
UINT8 EnumHubPort( UINT8 RootHubIndex)
{
    UINT8 i, s;
    printf( "EnumHubPort\n" );
    for (i = 1; i <= RootHubDev[RootHubIndex].GpVar[0]; i ++) // Query whether the port of the hub has changed
    {
        SelectHubPort( RootHubIndex, 0 ); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
        s = HubGetPortStatus( i ); // Get port status
        if (s != ERR_SUCCESS)
        {
            return( s ); // Maybe the HUB is disconnected
        }
        if ((( TxBuffer[0]&(1<<(HUB_PORT_CONNECTION&0x07))) && (TxBuffer[2]&(1<<(HUB_C_PORT_CONNECTION&0x07)) ))||(TxBuffer[2] == 0x10))
        {
            // Found a device connection
            DevOnHubPort[RootHubIndex][i-1].DeviceStatus = ROOT_DEV_CONNECTED; // There is a device connection
            DevOnHubPort[RootHubIndex][i-1].DeviceAddress = 0x00;
            s = HubGetPortStatus( i ); // Get port status
            if (s != ERR_SUCCESS)
            {
                return( s ); // It may be that the HUB is disconnected
            }
            DevOnHubPort[RootHubIndex][i-1].DeviceSpeed ​​= TxBuffer[1] & (1<<(HUB_PORT_LOW_SPEED&0x07))? 0: 1;// Low speed or full speed
            if (DevOnHubPort[RootHubIndex][i-1].DeviceSpeed)
            {
                printf( "Found full speed device on port %1d\n", (UINT16)i );
            }
            else
            {
                printf( "Found low speed device on port %1d\n", (UINT16)i );
            }
            mDelaymS( 200 ); // Wait for the device to be stable
            s = HubSetPortFeature( i, HUB_PORT_RESET ); // reset the port with the device connected
            if (s != ERR_SUCCESS)
            {
                return( s ); // Maybe the HUB is disconnected
            }
            printf( "Reset port and then wait in\n" );
            do // Query the reset port until the reset is completed, and display the status after completion
            {
                mDelaymS( 1 );
                s = HubGetPortStatus( i );
                if (s != ERR_SUCCESS)
                {
                    return( s ); // Maybe the HUB is disconnected
                }
            }
            while (TxBuffer[0] & (1<<(HUB_PORT_RESET&0x07)) ); // The port is reset and wait
            mDelaymS( 100 );
            s = HubClearPortFeature( i, HUB_C_PORT_RESET ); // Clear reset completion flag
// s = HubSetPortFeature( i, HUB_PORT_ENABLE ); // enable HUB port
            s = HubClearPortFeature( i, HUB_C_PORT_CONNECTION ); // Clear connection or remove change flag
            if (s != ERR_SUCCESS)
            {
                return( s );
            }
            s = HubGetPortStatus( i ); // Read the status again and check if the device is still
            if (s != ERR_SUCCESS)
            {
                return( s );
            }
            if ((TxBuffer[0]&(1<<(HUB_PORT_CONNECTION&0x07))) == 0)
            {
                DevOnHubPort[RootHubIndex][i-1].DeviceStatus = ROOT_DEV_DISCONNECT; // The device is gone
            }
            s = InitDevOnHub( RootHubIndex, i ); // initialize secondary USB device
            if (s != ERR_SUCCESS)
            {
                return( s );
            }
            SetUsbSpeed( 1 ); // The default is full speed
        }
        else if (TxBuffer[2]&(1<<(HUB_C_PORT_ENABLE&0x07))) // device connection error
        {
            HubClearPortFeature( i, HUB_C_PORT_ENABLE ); // Clear the connection error flag
            printf( "Device on port error\n" );
            s = HubSetPortFeature( i, HUB_PORT_RESET ); // reset the port with the device connected
            if (s != ERR_SUCCESS)
            {
                return( s ); // Maybe the HUB is disconnected
            }
            do // Query the reset port until the reset is completed, and display the status after completion
            {
                mDelaymS( 1 );
                s = HubGetPortStatus( i );
                if (s != ERR_SUCCESS)
                {
                    return( s ); // Maybe the HUB is disconnected
                }
            }
            while (TxBuffer[0] & (1<<(HUB_PORT_RESET&0x07)) ); // The port is reset and wait
        }
        else if ((TxBuffer[0]&(1<<(HUB_PORT_CONNECTION&0x07))) == 0) // The device has been disconnected
        {
            if (DevOnHubPort[RootHubIndex][i-1].DeviceStatus >= ROOT_DEV_CONNECTED)
            {
                printf( "Device on port %1d removed\n", (UINT16)i );
            }
            DevOnHubPort[RootHubIndex][i-1].DeviceStatus = ROOT_DEV_DISCONNECT; // There is a device connection
            if (TxBuffer[2]&(1<<(HUB_C_PORT_CONNECTION&0x07)))
            {
                HubClearPortFeature( i, HUB_C_PORT_CONNECTION ); // Clear the removal change flag
            }
        }
    }
    return( ERR_SUCCESS ); // Return operation is successful
}
/************************************************* ******************************
* Function Name: EnumAllHubPort
* Description: Enumerate all secondary USB devices behind the external HUB under the ROOT-HUB port
* Input: None
* Output: None
* Return: ERR_SUCCESS success
                   Other failure
************************************************** *****************************/
UINT8 EnumAllHubPort( void)
{
    UINT8 s, RootHubIndex;
    printf( "EnumAllHubPort\n" );
    for (RootHubIndex = 0; RootHubIndex != 2; RootHubIndex ++)
    {
        if (RootHubDev[RootHubIndex].DeviceStatus >= ROOT_DEV_SUCCESS && RootHubDev[RootHubIndex].DeviceType == USB_DEV_CLASS_HUB )// HUB enumeration succeeded
        {
            SelectHubPort( RootHubIndex, 0 ); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
            //What to do? Power on each port of the HUB, query the status of each port, initialize the HUB port with the device connected, and initialize the device
// for (i = 1; i <= RootHubDev[RootHubIndex].GpVar[0]; i ++ ){ // Initialize each port of HUB
// s = HubSetPortFeature( i, HUB_PORT_POWER ); // Power on each port of HUB
// if (s != ERR_SUCCESS)
// {
// return( s ); // The HUB may be disconnected
//}
//}
            s = EnumHubPort( RootHubIndex ); // Enumerate each port of the external HUB hub on the specified ROOT-HUB port, check whether each port is connected or removed
            if (s != ERR_SUCCESS) // Maybe the HUB is disconnected
            {
                printf( "EnumAllHubPort err = %02X\n", (UINT16)s );
            }
            SetUsbSpeed( 1 ); // The default is full speed
        }
    }
    return( ERR_SUCCESS );
}
/************************************************* ******************************
* Function Name: SearchTypeDevice
* Description: Search for the port number where the specified type of device is located on each port of the ROOT-HUB and external HUB. If the output port number is 0xFFFF, it is not found
* Input: UINT8 type Search device type
* Output: None
* Return: The output high 8 bits are the ROOT-HUB port number, the low 8 bits are the port number of the external HUB, and the low 8 bits are 0, the device is directly on the ROOT-HUB port
                   Of course, you can also search according to the PID of the USB manufacturer's VID product (record the VID and PID of each device in advance), and specify the search serial number
************************************************** *****************************/
UINT16 SearchTypeDevice( UINT8 type)
{
    UINT8 RootHubIndex, HubPortIndex;
    for (RootHubIndex = 0; RootHubIndex != 2; RootHubIndex ++) // The current search can avoid the problem that the device is unplugged in the middle and some information is not updated in time
    {
        if (RootHubDev[RootHubIndex].DeviceType == type && RootHubDev[RootHubIndex].DeviceStatus >= ROOT_DEV_SUCCESS)
        {
            return( (UINT16)RootHubIndex << 8 ); // The type matches and the enumeration is successful, on the ROOT-HUB port
        }
        if (RootHubDev[RootHubIndex].DeviceType == USB_DEV_CLASS_HUB && RootHubDev[RootHubIndex].DeviceStatus >= ROOT_DEV_SUCCESS )// External hub HUB and enumeration is successful
        {
            for (HubPortIndex = 1; HubPortIndex <= RootHubDev[RootHubIndex].GpVar[0]; HubPortIndex ++ )// Search each port of the external HUB
            {
                if (DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceType == type && DevOnHubPort[RootHubIndex][HubPortIndex-1].DeviceStatus >= ROOT_DEV_SUCCESS)
                {
                    return( ((UINT16)RootHubIndex << 8) | HubPortIndex ); // Type matches and enumeration is successful
                }
            }
        }
    }
    return( 0xFFFF );
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
    UINT8 i;
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
    for (i = 0; i != 2; i ++)
    {
        DisableRootHubPort( i ); // Clear
    }
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
// IE_USB = 1; // query method
}


main()
{
    UINT8 i,k,s, len, endp,HUBFlag;
    UINT16 loc;
    HUBFlag = 0;
    mInitSTDIO( ); //In order to let the computer monitor the demonstration process through the serial port
    printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
    InitUSB_Host( );
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
        if (FoundNewDev && s == ERR_USB_CONNECT) // There is a new USB device inserted
        {
            FoundNewDev = 0;
            mDelaymS( 200 ); // Since the USB device has just been inserted and is not yet stable, wait for hundreds of milliseconds for the USB device to eliminate plugging jitter
            s = EnumAllRootDevice( ); // Enumerate USB devices of all ROOT-HUB ports
            if (s != ERR_SUCCESS)
            {
                printf( "EnumAllRootDev err = %02X\n", (UINT16)s );
            }
        }
        loc = SearchTypeDevice( DEF_AOA_DEVICE ); // Search for the port number of the specified type of device on each port of ROOT-HUB and external HUB
        if (loc != 0xFFFF) // Found it, what if there are two MOUSEs?
        {
            i = (UINT8)( loc >> 8 );
            len = (UINT8)loc;
            SelectHubPort( i, len ); // Select the ROOT-HUB port specified by the operation, set the current USB speed and the USB address of the operated device
            endp = len? DevOnHubPort[i][len-1].GpVar: RootHubDev[i].GpVar[0]; // find IN endpoint

            if (endp & USB_ENDP_ADDR_MASK) // Endpoint is valid
            {
                s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80? bUH_R_TOG | bUH_T_TOG: 0, 0 ); // CH559 transfers transactions, gets data, NAK does not retry
                if (s == ERR_SUCCESS)
                {
                    endp ^= 0x80; // Synchronization flag flip
                    if (len)
                    {
                        DevOnHubPort[i][len-1].GpVar = endp; // Save the synchronization flag
                    }
                    else
                    {
                        RootHubDev[i].GpVar[0] = endp;
                    }
                    len = USB_RX_LEN; // The received data length
                    if (len)
                    {
                        printf("recv data: ");
                        for (k = 0; k <len; k ++)
                        {
                            printf("x%02X ",(UINT16)(RxBuffer[k]) );
                        }
                        printf("\n");
                    }
To
//Return data
memcpy(TxBuffer,RxBuffer,len); //Return
endp = RootHubDev[i].GpVar[2]; //download endpoint to send OUT packet
UH_TX_LEN = len;
s = USBHostTransact( USB_PID_OUT << 4 | endp & 0x7F, endp & 0x80? bUH_R_TOG | bUH_T_TOG: 0, 0xffff ); //Retry downloading unlimited times
if(s == ERR_SUCCESS)
{
endp ^= 0x80; // Synchronization flag flip
RootHubDev[i].GpVar[2] = endp; // Save the synchronization flag
printf("send back\n");
}

                }
                else if (s != (USB_PID_NAK | ERR_USB_TRANSFER))
                {
                    printf("transmit error %02x\n",(UINT16)s); // may be disconnected
                }
            }
            else
            {
                printf("no interrupt endpoint\n");
            }
            SetUsbSpeed( 1 ); // The default is full speed
        }

    }
}
