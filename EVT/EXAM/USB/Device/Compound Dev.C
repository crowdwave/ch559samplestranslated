/********************************** (C) COPYRIGHT *********** ********************
* File Name :Compound_Dev.C
* Author: WCH
* Version: V1.4
* Date: 2018/03/06
* Description: CH559 simulates USB composite device, keyboard and mouse, supports class commands
************************************************** *****************************/
#include "..\..\DEBUG.C" //Printing debugging information
#include "..\..\DEBUG.H"
#include <string.h>
#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE
UINT8X Ep0Buffer[64<(THIS_ENDP0_SIZE+2)?64:(THIS_ENDP0_SIZE+2)] _at_ 0x0000; //Endpoint 0 OUT&IN buffer, must be an even address
UINT8X Ep1Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000A; //Endpoint 1 IN buffer, must be an even address
UINT8X Ep2Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050; //Endpoint 2 IN buffer, must be an even address
UINT8 SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig;
PUINT8 pDescr; //USB configuration flag
USB_SETUP_REQ SetupReqBuf; //temporarily store the Setup package
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0

#pragma NOAREGS

/*Device descriptor*/
UINT8C DevDesc[18] = {0x12,0x01,0x10,0x01,0x00,0x00,0x00,0x08,
                      0x3d,0x41,0x07,0x21,0x00,0x00,0x00,0x00,
                      0x00,0x01
                     };
UINT8C CfgDesc[59] =
{
    0x09,0x02,0x3b,0x00,0x02,0x01,0x00,0xA0,0x32, //Configuration descriptor
    0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00, //interface descriptor, keyboard
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00, //HID class descriptor
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a, //endpoint descriptor
    0x09,0x04,0x01,0x00,0x01,0x03,0x01,0x02,0x00, //interface descriptor, mouse
    0x09,0x21,0x10,0x01,0x00,0x01,0x22,0x34,0x00, //HID class descriptor
    0x07,0x05,0x82,0x03,0x04,0x00,0x0a //endpoint descriptor
};
/*String descriptor*/
/*HID report descriptor*/
UINT8C KeyRepDesc[62] =
{
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};
UINT8C MouseRepDesc[52] =
{
    0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,
    0xA1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x03,
    0x81,0x02,0x75,0x05,0x95,0x01,0x81,0x01,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
    0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x03,
    0x81,0x06,0xC0,0xC0
};
/*Mouse data*/
UINT8 HIDMouse[4] = {0x0,0x0,0x0,0x0};
UINT8 HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
/************************************************* ******************************
* Function Name: USBDeviceCfg()
* Description: USB device mode configuration
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void USBDeviceCfg()
{
    USB_CTRL = 0x00; //Clear the USB control register
    USB_CTRL &= ~bUC_HOST_MODE; //This bit is to select the device mode
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; //USB device and internal pull-up enable, automatically return to NAK before the interrupt flag is cleared during the interrupt
    USB_DEV_AD = 0x00; //Device address initialization
    UDEV_CTRL &= ~bUD_RECV_DIS; //Enable the receiver
    USB_CTRL |= bUC_LOW_SPEED;
    UDEV_CTRL |= bUD_LOW_SPEED; //Select low speed 1.5M mode
// USB_CTRL &= ~bUC_LOW_SPEED;
// UDEV_CTRL &= ~bUD_LOW_SPEED; //Select the full speed 12M mode, the default mode
    UDEV_CTRL |= bUD_DP_PD_DIS | bUD_DM_PD_DIS; //Prohibit DM, DP pull-down resistor
    UDEV_CTRL |= bUD_PORT_EN; //Enable physical port
}
/************************************************* ******************************
* Function Name: USBDeviceIntCfg()
* Description: USB device mode interrupt initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void USBDeviceIntCfg()
{
    USB_INT_EN |= bUIE_SUSPEND; //Enable device suspension interrupt
    USB_INT_EN |= bUIE_TRANSFER; //Enable USB transfer completion interrupt
    USB_INT_EN |= bUIE_BUS_RST; //Enable device mode USB bus reset interrupt
    USB_INT_FG |= 0x1F; //Clear interrupt flag
    IE_USB = 1; //Enable USB interrupt
    EA = 1; //Allow MCU interrupt
}
/************************************************* ******************************
* Function Name: USBDeviceEndPointCfg()
* Description: USB device mode endpoint configuration, simulating a composite device, in addition to the control transmission of endpoint 0, it also includes the interrupt upload of endpoints 1 and 2
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void USBDeviceEndPointCfg()
{
    UEP1_DMA = Ep1Buffer; //Endpoint 1 data transmission address
    UEP4_1_MOD |= bUEP1_TX_EN; //Endpoint 1 transmit enable
    UEP4_1_MOD &= ~bUEP1_RX_EN; //Endpoint 1 reception prohibited
    UEP4_1_MOD &= ~bUEP1_BUF_MOD; //Single 64-byte send buffer for endpoint 1
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //Endpoint 1 automatically flips the synchronization flag, and the IN transaction returns NAK
    UEP2_DMA = Ep2Buffer; //Endpoint 2 data transmission address
    UEP2_3_MOD |= bUEP2_TX_EN; //Endpoint 2 transmit enable
    UEP2_3_MOD &= ~bUEP2_RX_EN; //Endpoint 2 reception prohibited
    UEP2_3_MOD &= ~bUEP2_BUF_MOD; //Endpoint 2 single 64-byte send buffer
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //Endpoint 2 automatically flips the synchronization flag, and the IN transaction returns NAK
    UEP0_DMA = Ep0Buffer; //Endpoint 0 data transmission address
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN); //Endpoint 0 single 64-byte transceiver buffer
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // OUT transaction returns ACK, IN transaction returns NAK
}
/************************************************* ******************************
* Function Name: enp1IntIn()
* Description: Interrupted upload of endpoint 1 in USB device mode
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void enp1IntIn()
{
    while(( UEP1_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK); //Wait for upload to complete, avoid reading and writing buffer at the same time
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey)); //Load upload data
    UEP1_T_LEN = sizeof(HIDKey); //Upload data length
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //Upload data and answer ACK when there is data
}
/************************************************* ******************************
* Function Name: enp2IntIn()
* Description: Interrupt upload of endpoint 2 in USB device mode
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void enp2IntIn()
{
    while(( UEP2_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK); //Wait for upload to complete, avoid reading and writing buffer at the same time
    memcpy( Ep2Buffer, HIDMouse, sizeof(HIDMouse)); //Load upload data
    UEP2_T_LEN = sizeof(HIDMouse); //Upload data length
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //Upload data and reply ACK when there is data
}

/************************************************* ******************************
* Function Name: DeviceInterrupt()
* Description: CH559USB interrupt processing function
************************************************** *****************************/
void DeviceInterrupt( void) interrupt INT_NO_USB using 1 //USB interrupt service routine, use register set 1
{
    UINT8 len;
#if DEBUG
    printf("%02X ",(UINT16)USB_INT_FG);
#endif
    if(UIF_TRANSFER) //USB transfer complete flag
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2: //endpoint 2# Interrupt endpoint upload
            UEP2_T_LEN = 0; //Pre-used transmission length must be cleared
// UEP1_CTRL ^= bUEP_T_TOG; //If auto flip is not set, manual flip is required
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
            break;
        case UIS_TOKEN_IN | 1: //endpoint 1# Interrupt endpoint upload
            UEP1_T_LEN = 0; //The pre-used transmission length must be cleared
// UEP2_CTRL ^= bUEP_T_TOG; //If auto flip is not set, manual flip is required
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
            FLAG = 1; /*Transfer complete flag*/
            break;
        case UIS_TOKEN_SETUP | 0: //SETUP transaction
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen> 0x7F)
                {
                    SetupLen = 0x7F; // limit the total length
                }
                len = 0; // The default is success and upload 0 length
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD )/* Only standard requests are supported */
                {
// len = 0xFF; // operation failed
// printf("ErrEp0ReqType=%02X\n",(UINT16)UsbSetupBuf->bRequestType);
                }
else{
                //Standard request
                SetupReq = UsbSetupBuf->bRequest;
#if DEBUG
                printf("REQ %02X ",(UINT16)SetupReq);
#endif
                switch(SetupReq) //Request code
                {
                case USB_GET_DESCRIPTOR:
                    switch(UsbSetupBuf->wValueH)
                    {
                    case 1: //device descriptor
                        pDescr = DevDesc; //Send the device descriptor to the buffer to be sent
                        len = sizeof(DevDesc);
                        break;
                    case 2: //Configuration descriptor
                        pDescr = CfgDesc; //Send the device descriptor to the buffer to be sent
                        len = sizeof(CfgDesc);
                        break;
                    case 0x22: //Report descriptor
#if DEBUG
                        printf("RREQ %02X ",(UINT16)SetupReq);
#endif
                        if(UsbSetupBuf->wIndexL == 0) //interface 0 report descriptor
                        {
                            pDescr = KeyRepDesc; //Data is ready to upload
                            len = sizeof(KeyRepDesc);
                        }
                        else if(UsbSetupBuf->wIndexL == 1) //interface 1 report descriptor
                        {
                            pDescr = MouseRepDesc; //Data is ready to upload
                            len = sizeof(MouseRepDesc);
                            Ready = 1; //If there are more interfaces, the standard bit should be valid after the last interface is configured
                        }
                        else
                        {
                            len = 0xff; //This program has only 2 interfaces, this sentence cannot be executed normally
                        }
                        break;
                    default:
                        len = 0xff; //unsupported command or error
                        break;
                    }
                    if (len == 0xff ){
                        break; //Error or command not supported
                    }
                    if (SetupLen> len ){
                        SetupLen = len; //Limit the total length
                    }
                    len = SetupLen >= 8? 8: SetupLen; //This transmission length
                    memcpy(Ep0Buffer,pDescr,len); //Load upload data
                    SetupLen -= len;
                    pDescr += len;
                    break;
                case USB_SET_ADDRESS:
                    SetupLen = UsbSetupBuf->wValueL; // temporarily store the USB device address
                    break;
                case USB_GET_CONFIGURATION:
                    Ep0Buffer[0] = UsbConfig;
                    if (SetupLen >= 1)
                    {
                        len = 1;
                    }
                    break;
                case USB_SET_CONFIGURATION:
                    UsbConfig = UsbSetupBuf->wValueL;
                    break;
                case 0x0A:
                    break;
                case USB_CLEAR_FEATURE: //Clear Feature
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP )// Endpoint
                    {
                       switch( UsbSetupBuf->wIndexL)
                       {
                          case 0x82:
                               UEP2_CTRL = UEP2_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                               break;
                          case 0x81:
                               UEP1_CTRL = UEP1_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                               break;
                          case 0x01:
                               UEP1_CTRL = UEP1_CTRL & ~ (bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                               break;
                          default:
                               len = 0xFF; // Unsupported endpoint
                               break;
                        }
                     }
                     else
                     {
                        len = 0xFF; // Not that the endpoint does not support
                     }
                    break;
                    case USB_SET_FEATURE: /* Set Feature */
                    if( (UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* Set up the device */
                    {
                        if( (((UINT16 )UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                        {
                            if( CfgDesc[ 7] & 0x20)
                            {
                                 /* Set wake-up enable flag */
                            }
                            else
                            {
                                len = 0xFF; /* Operation failed */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* Operation failed */
                        }
                    }
                    else if( (UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* Set endpoint */
                    {
                        if( (((UINT16 )UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                        {
                            switch( ((UINT16 )UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                            {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* Set endpoint 2 IN STALL */
                                    break;

                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* Set endpoint 2 OUT Stall */
                                    break;

                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* Set endpoint 1 IN STALL */
                                    break;

                                default:
                                    len = 0xFF; /* Operation failed */
                                    break;
                            }
                        }
                        else
                        {
                            len = 0xFF; /* Operation failed */
                        }
                    }
                    else
                    {
                        len = 0xFF; /* Operation failed */
                    }
                    break;
                case USB_GET_STATUS:
                     Ep0Buffer[0] = 0x00;
                     Ep0Buffer[1] = 0x00;
                     if (SetupLen >= 2)
                     {
                        len = 2;
                     }
                     else
                     {
                        len = SetupLen;
                     }
                     break;
                default:
                    len = 0xff; //The operation failed
                    break;
                }
}
            }
            else
            {
                len = 0xff; //Package length error
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= 8) //Upload data or return 0 length package in status stage
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
#if DEBUG
                printf("S_U\n");
#endif
            }
            else
            {
                UEP0_T_LEN = 0; //Although it has not yet reached the status stage, it is preset to upload 0-length data packets in advance to prevent the host from entering the status stage early
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
            }
            break;
        case UIS_TOKEN_IN | 0: //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8? 8: SetupLen; //This transmission length
                memcpy( Ep0Buffer, pDescr, len ); //Load upload data
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //Synchronization flag bit flip
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //The status phase completes the interrupt or the forced upload of a 0-length data packet ends the control transmission
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            if(SetupReq == 0x09)
            {
                if(Ep0Buffer[0])
                {
                    printf("Light on Num Lock LED!\n");
                }
                else if(Ep0Buffer[0] == 0)
                {
                    printf("Light off Num Lock LED!\n");
                }
            }
UEP0_CTRL ^= bUEP_R_TOG; //Synchronization flag bit flip
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //Write 0 to clear the interrupt
    }
    if(UIF_BUS_RST) //device mode USB bus reset interrupt
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; //Clear interrupt flag
    }
    if (UIF_SUSPEND) //USB bus suspend/wake up completed
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //suspend
        {
#if DEBUG
            printf( "zz" ); //sleep state
#endif
            while (XBUS_AUX & bUART0_TX)
            {
                ; //Waiting for sending completion
            }
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO; //USB or RXD0 can be woken up when there is a signal
            PCON |= PD; //sleep
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    }
    else {//Unexpected interruption, impossible situation
        USB_INT_FG = 0xFF; //Clear interrupt flag
// printf("UnknownInt N");
    }
}
void HIDValueHandle()
{
    UINT8 i;
    if (RI)
    {
        RI = 0;
        i = getkey( );
        printf( "%c", (UINT8)i );
        switch(i)
        {
//Mouse data upload example
        case'L': //Left button
            HIDMouse[0] = 0x01;
            enp2IntIn();
            HIDMouse[0] = 0;
            break;
        case'R': //right click
            HIDMouse[0] = 0x02;
            enp2IntIn();
            HIDMouse[0] = 0;
            break;
//Keyboard data upload example
        case'A': //A key
            FLAG = 0;
            HIDKey[2] = 0x04; //Press to start
            enp1IntIn();
            HIDKey[2] = 0; //End of key
            while(FLAG == 0); /*Wait for transmission to complete*/
            enp1IntIn();
            while(FLAG == 0); /*Wait for transmission to complete*/
            break;
        case'P': //P key
            FLAG = 0;
            HIDKey[2] = 0x13;
            enp1IntIn();
            HIDKey[2] = 0; //End of key
            while(FLAG == 0); /*Wait for transmission to complete*/
            enp1IntIn();
            while(FLAG == 0); /*Wait for transmission to complete*/
            break;
        case'Q': //Num Lock key
            FLAG = 0;
            HIDKey[2] = 0x53;
            enp1IntIn();
            HIDKey[2] = 0; //End of key
            while(FLAG == 0); /*Wait for transmission to complete*/
            enp1IntIn();
            while(FLAG == 0); /*Wait for transmission to complete*/
            break;
        default: //other
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
            break;
        }
    }
    else
    {
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; //Default response NAK
    }
}
main()
{
// CfgFsys( ); //CH559 clock selection configuration
// mDelaymS(5); //Wait for the external crystal oscillator to stabilize
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
    USBDeviceCfg(); //simulate mouse
    USBDeviceEndPointCfg(); //Endpoint configuration
    USBDeviceIntCfg(); //Interrupt initialization
    UEP1_T_LEN = 0; //The pre-used transmission length must be cleared
    UEP2_T_LEN = 0; //Pre-used transmission length must be cleared
    FLAG = 0;
    Ready = 0;
    while(1)
    {
        if(Ready)
        {
            HIDValueHandle();
        }
        mDelaymS( 100 ); //Simulate the MCU to do other things
    }
}
