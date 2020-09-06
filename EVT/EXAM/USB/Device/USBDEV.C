/********************************** (C) COPYRIGHT *********** ********************
* File Name :USBDEV.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 simulates USB 37X device
 USB device example for CH559, use CH372 tool TEST.exe to check it under Windows
************************************************** *****************************/
#include "..\..\DEBUG.C" //Printing debugging information
#include "..\..\DEBUG.H"
#include <string.h>

#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE

/*Device descriptor*/
UINT8C MyDevDescr[] = {0x12, 0x01, 0x10, 0x01,
                         0xFF, 0x80, 0x55, THIS_ENDP0_SIZE,
                         0x48, 0x43, 0x37, 0x55, // vendor ID and product ID
                         0x00, 0x01, 0x01, 0x02,
                         0x00, 0x01
                       };
/*Configuration descriptor*/
UINT8C MyCfgDescr[] = {0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
                         0x09, 0x04, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x55, 0x00,
                         0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
                         0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
                         0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x00
                       };
/*Language descriptor*/
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04 };
/*Manufacturer information*/
UINT8C MyManuInfo[] = {0x0E, 0x03,'w', 0,'c', 0,'h', 0,'.', 0,'c', 0,'n', 0 };
/*product information*/
UINT8C MyProdInfo[] = {0x0C, 0x03,'C', 0,'H', 0, '5', 0, '5', 0, '9', 0 };

UINT8 UsbConfig = 0; // USB configuration flag
UINT8X Ep0Buffer[64<(THIS_ENDP0_SIZE+2)?64:(THIS_ENDP0_SIZE+2)] _at_ 0x0000; // OUT&IN, must even address
UINT8X Ep1Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0200; // IN, must even address
UINT8X Ep2Buffer[128<(2*MAX_PACKET_SIZE+4)? 128:(2*MAX_PACKET_SIZE+4)] _at_ 0x0240 ;// OUT+IN, must even address

#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

/*Operation status indication*/
sbit LED_RUN = P1^0;
sbit LED_CFG = P1^1;
sbit LED_TMP = P1^2;

#pragma NOAREGS

/************************************************* ******************************
* Function Name: USB_DeviceInterrupt()
* Description: CH559USB analog setting interrupt processing function
************************************************** *****************************/
void USB_DeviceInterrupt( void) interrupt INT_NO_USB using 1 /* USB interrupt service routine, use register set 1 */
{
    UINT8 i, len;
    static UINT8 SetupReqCode, SetupLen;
    static PUINT8 pDescr;
    LED_TMP = 0;
To
    if (UIF_TRANSFER) // USB transfer completed
    {
        if (U_IS_NAK) // not enable for this example
        {
// switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) // analyze operation token and endpoint number
// {
// case UIS_TOKEN_OUT | 2: // endpoint 2# Batch endpoint download
// break;
// case UIS_TOKEN_IN | 2: // endpoint 2# Bulk endpoint upload
// break;
// case UIS_TOKEN_IN | 1: // endpoint 1# Interrupt endpoint upload
// break;
// default:
// break;
//}
            printf("NAK INT,PrepareData\n");
        }
        else
        {
            switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) // analyze operation token and endpoint number
            {
            case UIS_TOKEN_OUT | 2: // endpoint 2# Batch endpoint download
                if (U_TOG_OK) // unsynchronized data packets will be discarded
                {
// UEP2_CTRL ^= bUEP_R_TOG; // Automatically flipped
                    len = USB_RX_LEN;
                    for (i = 0; i <len; i ++)
                    {
                        Ep2Buffer[MAX_PACKET_SIZE+i] = Ep2Buffer[i] ^ 0xFF; // OUT data is reversed to IN and verified by computer
                    }
                    UEP2_T_LEN = len;
                    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; // Allow upload
                }
                break;
            case UIS_TOKEN_IN | 2: // endpoint 2# Bulk endpoint upload
// UEP2_CTRL ^= bUEP_T_TOG; // Automatically flipped
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; // Pause upload
                break;
            case UIS_TOKEN_IN | 1: // endpoint 1# Interrupt endpoint upload
// UEP1_CTRL ^= bUEP_T_TOG; // Automatically flipped
                UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK; // Pause upload
                break;
            case UIS_TOKEN_SETUP | 0: // endpoint 0# SETUP
                len = USB_RX_LEN;
                if (len == sizeof( USB_SETUP_REQ)) // SETUP packet length
                {
                    SetupLen = UsbSetupBuf->wLengthL;
                    if (UsbSetupBuf->wLengthH || SetupLen> 0x7F)
                    {
                        SetupLen = 0x7F; // limit the total length
                    }
                    len = 0; // The default is success and upload 0 length
                    if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD )/* Only standard requests are supported */
                    {
                        len = 0xFF; // operation failed
                        printf("ErrEp0ReqType=%02X\n",(UINT16)UsbSetupBuf->bRequestType);
                    }
                    else // standard request
                    {
                        SetupReqCode = UsbSetupBuf->bRequest;
                        switch( SetupReqCode) // request code
                        {
                        case USB_GET_DESCRIPTOR:
                            switch( UsbSetupBuf->wValueH)
                            {
                            case 1: // device descriptor
                                pDescr = (PUINT8)( &MyDevDescr[0] );
                                len = sizeof( MyDevDescr );
                                break;
                            case 2: // configuration descriptor
                                pDescr = (PUINT8)( &MyCfgDescr[0] );
                                len = sizeof( MyCfgDescr );
                                break;
                            case 3: // string descriptor
                                switch( UsbSetupBuf->wValueL)
                                {
                                case 1:
                                    pDescr = (PUINT8)( &MyManuInfo[0] );
                                    len = sizeof( MyManuInfo );
                                    break;
                                case 2:
                                    pDescr = (PUINT8)( &MyProdInfo[0] );
                                    len = sizeof( MyProdInfo );
                                    break;
                                case 0:
                                    pDescr = (PUINT8)( &MyLangDescr[0] );
                                    len = sizeof( MyLangDescr );
                                    break;
                                default:
                                    len = 0xFF; // unsupported string descriptor
                                    break;
                                }
                                break;
                            default:
                                len = 0xFF; // Descriptor type not supported
                                break;
                            }
                            if (SetupLen> len)
                            {
                                SetupLen = len; // limit the total length
                            }
                            len = SetupLen >= THIS_ENDP0_SIZE? THIS_ENDP0_SIZE: SetupLen; // This transmission length
                            memcpy( Ep0Buffer, pDescr, len ); /* Load upload data */
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
                            if (UsbConfig)
                            {
                                LED_CFG = 0;
                            }
                            else
                            {
                                LED_CFG = 1;
                            }
                            break;
                        case USB_CLEAR_FEATURE:
                            if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP )// Endpoint
                            {
                                switch( UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & ~ (bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & ~ (bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                    break;
                                case 0x01:
                                    UEP1_CTRL = UEP1_CTRL & ~ (bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                    break;
                                default:
                                    len = 0xFF; // unsupported endpoint
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
                            if( MyCfgDescr[ 7] & 0x20)
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
                        case USB_GET_INTERFACE:
                            Ep0Buffer[0] = 0x00;
                            if (SetupLen >= 1)
                            {
                                len = 1;
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
                            len = 0xFF; // operation failed
                            printf("ErrEp0ReqCode=%02X\n",(UINT16)SetupReqCode);
                            break;
                        }
                    }
                }
                else
                {
                    len = 0xFF; // SETUP packet length error
                    printf("ErrEp0ReqSize\n");
                }
                if (len == 0xFF) // operation failed
                {
                    SetupReqCode = 0xFF;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;// STALL
                }
                else if (len <= THIS_ENDP0_SIZE) // upload data or return 0 length packet in status stage
                {
                    UEP0_T_LEN = len;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1
                }
                else // download data or other
                {
                    UEP0_T_LEN = 0; // Although it has not yet reached the status stage, upload a 0-length data packet in advance to prevent the host from entering the status stage in advance
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1
                }
                break;
            case UIS_TOKEN_IN | 0: // endpoint 0# IN
                switch( SetupReqCode)
                {
                case USB_GET_DESCRIPTOR:
                    len = SetupLen >= THIS_ENDP0_SIZE? THIS_ENDP0_SIZE: SetupLen; // This transmission length
                    memcpy( Ep0Buffer, pDescr, len ); /* Load upload data */
                    SetupLen -= len;
                    pDescr += len;
                    UEP0_T_LEN = len;
                    UEP0_CTRL ^= bUEP_T_TOG; // flip
                    break;
                case USB_SET_ADDRESS:
                    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                default:
                    UEP0_T_LEN = 0; // The status phase is completed interrupted or a 0-length data packet is forcibly uploaded to end the control transmission
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                }
                break;
            case UIS_TOKEN_OUT | 0: // endpoint 0# OUT
                switch( SetupReqCode)
                {
// case download:
// if (U_TOG_OK) {// out-of-sync packets will be discarded
// UEP0_CTRL ^= bUEP_R_TOG; // flip
// get_data;
// //UEP0_CTRL = UEP0_CTRL & bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Preset upload 0 length data packet DATA1 to prevent the host from entering the state stage early
//}
// break;
                case USB_GET_DESCRIPTOR:
                default:
                    if (U_TOG_OK) // unsynchronized data packets will be discarded
                    {
// if (USB_RX_LEN) control_status_error;
// else control_ok; // Receiving a 0-length packet indicates that the read operation/upload is OK
                    }
// else control_status_error;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // prepare for the next control transmission
                    break;
                }
                break;
            default:
                printf("ErrEndp INT\n");
                break;
            }
        }
        UIF_TRANSFER = 0; // Clear interrupt flag
    }
    else if (UIF_BUS_RST) // USB bus reset
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        LED_CFG = 1;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; // Clear interrupt flag
    }
    else if (UIF_SUSPEND) // USB bus suspend/wake up completed
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) // hang
        {
            printf( "Zzz" ); // sleep state
            LED_RUN = 1;
            while (XBUS_AUX & bUART0_TX ); // Wait for the transmission to be completed
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO; // USB or RXD0 can be woken up when there is a signal
            PCON |= PD; // sleep
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
        else // wake up
        {
            LED_RUN = 0;
        }
    }
    else
    {// Unexpected interruption, impossible situation
        printf("Unknown INT\n");
        USB_INT_FG = 0xFF; // Clear interrupt flag
    }
    LED_TMP = 1;
}

void InitUSB_Device( void) // initialize the USB device
{
    IE_USB = 0;
    LED_CFG = 1;
    LED_RUN = 0;
    USB_CTRL = 0x00; // Set the mode first
    UEP4_1_MOD = bUEP1_TX_EN; // Endpoint 1 upload IN
    UEP2_3_MOD = bUEP2_RX_EN | bUEP2_TX_EN; // Endpoint 2 downloads OUT and uploads IN
    UEP0_DMA = Ep0Buffer;
    UEP1_DMA = Ep1Buffer;
    UEP2_DMA = Ep2Buffer;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_DP_PD_DIS | bUD_DM_PD_DIS; // Disable DP/DM pull-down resistor
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // Start the USB device and DMA, and automatically return to NAK before the interrupt flag is cleared during the interrupt
    UDEV_CTRL |= bUD_PORT_EN; // Allow USB port
    USB_INT_FG = 0xFF; // Clear interrupt flag
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

main()
{
    UINT8 i;
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
    mInitSTDIO( ); /* In order to let the computer monitor the demonstration process through the serial port */
    printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
    InitUSB_Device( );
    EA = 1;
    while (1)
    {
        i = getkey( );
        printf( "%c", (UINT8)i );
        if (i >= '0' && i <='z')
        {
            while(( UEP1_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK); //Wait for upload to complete, avoid reading and writing buffer at the same time
            memcpy( Ep1Buffer, (PUINT8C)(i-'0'), MAX_PACKET_SIZE ); /* Load upload data */
            UEP1_T_LEN = i-'0'> 8? 8: i-'0';
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
        }
    }
}
