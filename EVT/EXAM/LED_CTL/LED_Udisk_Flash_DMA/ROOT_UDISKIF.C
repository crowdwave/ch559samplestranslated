/********************************** (C) COPYRIGHT *********** ********************
* File Name: ROOT_UDISKIF.C
* Author: WCH
* Version: V1.1
* Date: 2015/10/13
* Description: CH559 U Disk Read and Write Interface
************************************************** *****************************/
#include "../../CH559.H"
//You also need to add LIB library files
#include "../../USB_LIB/CH559UFI.H"
#include "../../USB_LIB/CH559UFI.C"

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
UINT8X RxBuffer[ MAX_PACKET_SIZE] _at_ 0x0000; // IN, must even address
UINT8X TxBuffer[ MAX_PACKET_SIZE] _at_ 0x0040; // OUT, must even address


/************************************************* ******************************
* Function Name: DisableRootHubPort( void)
* Description: Close the Roothub port
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void DisableRootHubPort( void) // Close the port, in fact the hardware has been automatically closed, here is just to clear some structure states
{
    CH559DiskStatus = DISK_DISCONNECT;
    UHUB0_CTRL = 0x00; // Clear the control data related to HUB0, actually does not need to be cleared
}

/************************************************* ******************************
* Function Name: AnalyzeRootHub( void)
* Description: analyze port status
* Input: None
* Output: None
* Return: UINT8 s
************************************************** *****************************/
UINT8 AnalyzeRootHub( void) // Analyze the port status and handle the port device plug-in event
// Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, return ERR_USB_DISCON for detected disconnection
{
//Process the port plug-in event, if the device is unplugged, call the DisableRootHubPort() function in the function to close the port, insert the event, and set the status bit of the corresponding port
    UINT8 s;
    s = ERR_SUCCESS;
    if (USB_HUB_ST & bUHS_H0_ATTACH) // device exists
    {
        if (CH559DiskStatus == DISK_DISCONNECT || (UHUB0_CTRL & bUH_PORT_EN) == 0x00) // Device insertion is detected, but it has not been allowed, indicating that it has just been inserted
        {
            DisableRootHubPort( ); // Close the port
            CH559DiskStatus = DISK_CONNECT;
            s = ERR_USB_CONNECT;
        }
    }
    else if (CH559DiskStatus >= DISK_CONNECT)
    {
        DisableRootHubPort( ); // Close the port
        if (s == ERR_SUCCESS)
        {
            s = ERR_USB_DISCON;
        }
    }
    return( s );
}

/************************************************* ******************************
* Function Name: SetHostUsbAddr( UINT8 addr)
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
* Function Name: ResetRootHubPort( void)
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
* Function Name: EnableRootHubPort( void)
* Description: Enable the port, the corresponding bUH_PORT_EN is set to 1 to enable the port, the device may be disconnected, which may cause the return failure
* Input: None
* Output: None
* Return: ERR_USB_DISCON
************************************************** *****************************/
UINT8 EnableRootHubPort( void)
{
    if (CH559DiskStatus <DISK_CONNECT)
    {
        CH559DiskStatus = DISK_CONNECT;
    }
    if (USB_HUB_ST & bUHS_H0_ATTACH) // There is a device
    {
        UHUB0_CTRL |= bUH_PORT_EN; //Enable HUB port
        return( ERR_SUCCESS );
    }
    return( ERR_USB_DISCON );
}

/************************************************* ******************************
* Function Name: WaitUSB_Interrupt( void)
* Description: Wait for USB interrupt
* Input: None
* Output: None
* Return: UIF_TRANSFER? ERR_SUCCESS: ERR_USB_UNKNOWN
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
* Function Name: USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout)
* Description: CH559 transmission transaction, enter the destination endpoint address/PID token, synchronization flag, the total NAK retry time in units of 20uS (0 means no retry, 0xFFFF unlimited retry), return 0 success, timeout/error retry test
* Input: UINT8 endp_pid, UINT8 tog, UINT16 timeout
* Output: None
* Return: some
************************************************** *****************************/
UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout) // endp_pid: the upper 4 bits are the token_pid token, the lower 4 bits are the endpoint address
{
 // This subprogram focuses on easy understanding, and in actual applications, in order to provide running speed, the code of this subprogram should be optimized
    #define TransRetry UEP0_T_LEN // Save memory
    UINT8 s, r;
    UINT16 i;
    UH_RX_CTRL = UH_TX_CTRL = tog;
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
        if (UIF_DETECT) // USB device plug-in event
        {
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
                    if (r)
                    {
                        return( r | ERR_USB_TRANSFER ); // Not a timeout/error, unexpected response
                    }
                    break; // Retry after timeout
                case USB_PID_IN:
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
    return( ERR_USB_TRANSFER ); // Response timeout
}

/************************************************* ******************************
* Function Name: HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen)
* Description: Perform control transmission, the 8-byte request code is in pSetupReq, and DataBuf is an optional transceiver buffer
* Input: PUINT8X DataBuf, PUINT8I RetLen
* Output: None
* Return: some
************************************************** *****************************/
UINT8 HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen)
// If you need to receive and send data, then DataBuf needs to point to a valid buffer for storing subsequent data, and the total length of the actual successful transmission and reception is stored in the byte variable pointed to by ReqLen
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
                s = USBHostTransact( USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, 200000/20 ); // OUT data
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
* Function Name: CopySetupReqPkg( PUINT8C pReqPkt)
* Description: Copy control transmission request packet
* Input: PUINT8C pReqPkt
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
* Function Name: CtrlGetDeviceDescr( void)
* Description: Get the device descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: some
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
* Function Name: CtrlGetConfigDescr( void)
* Description: Get the configuration descriptor and return it in TxBuffer
* Input: None
* Output: None
* Return: some
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
* Function Name: CtrlSetUsbAddress( UINT8 addr)
* Description: Set the USB device address
* Input: UINT8 addr
* Output: None
* Return: some
************************************************** *****************************/
UINT8 CtrlSetUsbAddress( UINT8 addr) // Set the USB device address
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
* Function Name: CtrlSetUsbConfig( UINT8 cfg)
* Description: Set USB device configuration
* Input: UINT8 cfg
* Output: None
* Return: HostCtrlTransfer( NULL, NULL)
************************************************** *****************************/
UINT8 CtrlSetUsbConfig( UINT8 cfg) // Set USB device configuration
{
    CopySetupReqPkg( SetupSetUsbConfig );
    pSetupReq -> wValueL = cfg; // USB device configuration
    return( HostCtrlTransfer( NULL, NULL) ); // Perform control transfer
}

/************************************************* ******************************
* Function Name: CtrlClearEndpStall( UINT8 endp)
* Description: Clear endpoint STALL
* Input: UINT8 endp
* Output: None
* Return: HostCtrlTransfer( NULL, NULL)
************************************************** *****************************/
UINT8 CtrlClearEndpStall( UINT8 endp) // Clear endpoint STALL
{
    CopySetupReqPkg( SetupClrEndpStall ); // Clear the error of the endpoint
    pSetupReq -> wIndexL = endp; // endpoint address
    return( HostCtrlTransfer( NULL, NULL) ); /* Perform control transfer */
}

/************************************************* ******************************
* Function Name: AnalyzeHidIntEndp( PUINT8X buf)
* Description: Analyze the address of the HID interrupt endpoint from the descriptor
* Input: PUINT8X buf
* Output: None
* Return: UINT8 s
************************************************** *****************************/
UINT8 AnalyzeHidIntEndp( PUINT8X buf) // Analyze the address of the HID interrupt endpoint from the descriptor
{
    UINT8 i, s, l;
    s = 0;
    for (i = 0; i <((PXUSB_CFG_DESCR)buf) -> wTotalLengthL; i += l) // search interrupt endpoint descriptor, skip configuration descriptor and interface descriptor
    {
        if (((PXUSB_ENDP_DESCR)(buf+i)) -> bDescriptorType == USB_DESCR_TYP_ENDP // is the endpoint descriptor
                && (((PXUSB_ENDP_DESCR)(buf+i)) -> bmAttributes & USB_ENDP_TYPE_MASK) == USB_ENDP_TYPE_INTER // is the interrupt endpoint
                && (((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_DIR_MASK)) // is the IN endpoint
        {
            s = ((PXUSB_ENDP_DESCR)(buf+i)) -> bEndpointAddress & USB_ENDP_ADDR_MASK; // interrupt endpoint address
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
* Function Name: InitRootDevice( void)
* Description: Initialize the USB device
* Input: None
* Output: None
* Return: some
************************************************** *****************************/
UINT8 InitRootDevice( void) // Initialize the USB device
{
    UINT8 i, s, cfg, dv_cls, if_cls;
    ResetRootHubPort( ); // After detecting the device, reset the USB bus of the corresponding port
    for (i = 0, s = 0; i <100; i ++) // wait for the USB device to reset and reconnect, 100mS timeout
    {
        mDelaymS( 1 );
        if (EnableRootHubPort() == ERR_SUCCESS) // enable port
        {
            i = 0;
            s ++; // Time to wait for the stability of the USB device connection
            if (s> 100)
            {
                break; // has been connected stably for 100mS
            }
        }
    }
    if (i) // device is not connected after reset
    {
        DisableRootHubPort( );
        return( ERR_USB_DISCON );
    }
    SetUsbSpeed( ThisUsbDev.DeviceSpeed ​​); // Set the current USB speed
    s = CtrlGetDeviceDescr( ); // Get device descriptor
    if (s == ERR_SUCCESS)
    {
        for (i = 0; i <((PUSB_SETUP_REQ)SetupGetDevDescr) -> wLengthL; i ++)
        {
           ;
        }
        dv_cls = ((PXUSB_DEV_DESCR)TxBuffer) -> bDeviceClass; // device class code
        s = CtrlSetUsbAddress( ((PUSB_SETUP_REQ)SetupSetUsbAddr) -> wValueL ); // Set USB device address
        if (s == ERR_SUCCESS)
        {
            s = CtrlGetConfigDescr( ); // Get configuration descriptor
            if (s == ERR_SUCCESS)
            {
                cfg = ((PXUSB_CFG_DESCR)TxBuffer) -> bConfigurationValue;
                for (i = 0; i <((PXUSB_CFG_DESCR)TxBuffer) -> wTotalLengthL; i ++)
                {
                  ;
                }
                /* Analyze the configuration descriptor, get the endpoint data/each endpoint address/each endpoint size, etc., update the variables endp_addr and endp_size, etc. */
                if_cls = ((PXUSB_CFG_DESCR_LONG)TxBuffer) -> itf_descr.bInterfaceClass; // interface class code
                if (dv_cls == 0x00 && if_cls == USB_DEV_CLASS_STORAGE) // It is a USB storage device, basically confirm it is a U disk
                {
                    CH559DiskStatus = DISK_USB_ADDR;
                    return( ERR_SUCCESS );
                }
                else
                {
                    return( ERR_USB_UNSUPPORT );
                }
            }
        }
    }
    CH559DiskStatus = DISK_CONNECT;
    SetUsbSpeed( 1 ); // The default is full speed
    return( s );
}

/************************************************* ******************************
* Function Name: InitUSB_Host( void)
* Description: Initialize the USB host
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitUSB_Host( void) // initialize the USB host
{
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
    USB_INT_FG |= 0xFF; // Clear interrupt flag
    DisableRootHubPort( ); // Clear
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
}

/************************************************* ******************************
* Function Name: mStopIfError( UINT8 iError)
* Description: Check the operating status, if there is an error, display the error code and stop
* Input: UINT8 iError
* Output: None
* Return: None
************************************************** *****************************/
void mStopIfError( UINT8 iError)
{
    if (iError == ERR_SUCCESS)
    {
        return; /* The operation was successful */
    }
    printf( "Error: %02X\n", (UINT16)iError ); /* display error */
    /* After encountering an error, you should analyze the error code and CH559DiskStatus status. For example, call CH559DiskReady to query whether the current U disk is connected. If the U disk is disconnected, then wait for the U disk to be plugged in and then operate.
       Suggested processing steps after error:
       1. Call CH559DiskReady once, and continue the operation if successful, such as Open, Read/Write, etc.
       2. If CH559DiskReady is unsuccessful, then forcibly start the operation from the beginning (waiting for U disk connection, CH559DiskReady, etc.) */
    while (1)
    {
// LED_TMP=0; /* LED flashes */
// mDelaymS( 100 );
// LED_TMP=1;
// mDelaymS( 100 );
    }
}
