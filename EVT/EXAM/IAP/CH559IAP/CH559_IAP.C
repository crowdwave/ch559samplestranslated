/********************************** (C) COPYRIGHT *********** *******************
* File Name: CH559_IAP.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: IAP function demonstration example program
* 1. Support serial port download. The serial port number is 0 and the baud rate is 57600. Because the internal crystal oscillator is used, the crystal oscillator has errors, so add the serial port accumulation and the accumulation and error to retransmit
*2, support USB download, USB is a full-speed device
                       3. Support EEPROM programming
                       4. Support chip model judgment
************************************************** *****************************/

#include <stdlib.h>
#include <string.h>
#include <intrins.h>
#include "../../CH559.H"
#include "CH559_IAP.H"

sbit DisableIAP = P1^0; //Return to user program detection pin
#define IAP_CODE_ADDR (0xE800) //Integer multiple of 1k, because 55X Flash erases at least 1K at a time
#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE //The size of the default endpoint 0

// Device descriptor
UINT8C MyDevDescr[] = {0x12, 0x01, 0x10, 0x01,
                           0xFF, 0x80, 0x55, THIS_ENDP0_SIZE,
                           0x48, 0x43, 0xe0, 0x55,
                           0x00, 0x01, 0x00, 0x00,0x00, 0x01
                         };
// configuration descriptor
UINT8C MyCfgDescr[] = {0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
                           0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x80, 0x55, 0x00,
                           0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
                           0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00
                         };
UINT8 UsbConfig = 0; // USB configuration flag
UINT8X Ep0Buffer[THIS_ENDP0_SIZE] _at_ 0x0000; // OUT&IN, must even address
UINT8X Ep2Buffer[2*MAX_PACKET_SIZE] _at_ 0x0008; // OUT+IN, must even address
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

iap_cmd1 xdata iap_cmd _at_ 0x0088; //IAP command
UINT8 uart_bit; //Global flag bit of download mode, 1 means serial port, 2 means USB port
UINT16 chip_id,eeprom_len;
PUINT8C pCode;

#pragma NOAREGS

/************************************************* ******************************
* Function Name: EraseBlock
* Description: Chip erase function, the default erase block is 1KB
* Input: Addr chip erase address, 1KB is the basic unit
* Output: None
* Return: Chip erase return status
                    0x00 successfully erased
                    0x01 Erase timeout
                    0x02 Unknown error, erase failed
************************************************** *****************************/
UINT8 EraseBlock( UINT16 Addr)
{
ROM_ADDR = Addr;
if (ROM_STATUS & bROM_ADDR_OK) {// The operation address is valid
ROM_CTRL = ROM_CMD_ERASE;
return( (ROM_STATUS ^ bROM_ADDR_OK) & 0x7F ); // return status, 0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
}
else return( 0x40 );
}
/************************************************* ******************************
* Function Name: ProgWord
* Description: Chip programming function
* Input: Addr chip programming address address, the address is an even address
                   Data Programming data, based on WORD
* Output: None
* Return: chip programming return status
                    0x00 successful programming
                    0x01 Programming timeout
                    0x02 Unknown error, programming failed
************************************************** *****************************/
UINT8 ProgWord( UINT16 Addr, UINT16 Data)
{
ROM_ADDR = Addr;
ROM_DATA = Data;
if (ROM_STATUS & bROM_ADDR_OK) {// The operation address is valid
ROM_CTRL = ROM_CMD_PROG;
return( (ROM_STATUS ^ bROM_ADDR_OK) & 0x7F ); // return status, 0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
}
else return( 0x40 );
}
/************************************************* ******************************
* Function Name: FlashVerify
* Description: Flash check
* Input: Addr chip programming address address, the address is an even address
                   pData programming data, based on WORD
                   len check length
* Output: None
* Return: Return to verification status
                    0x00 verification successful
                    0xff verification failed
************************************************** *****************************/
UINT8 FlashVerify( UINT16 Addr, UINT8 *pData, UINT16 len)
{
    UINT16 i;
    pCode = (PUINT8C)( Addr );
    for( i=0; i!=len; i++)
    {
        if( *pData != *pCode)
        {
            return 0xff;
        }
        pCode++;
        pData++;
    }
    return 0;
}
/************************************************* ******************************
* Function Name: UART_Send
* Description: Serial 0 byte sending
* Input: dat serial port data to be sent
* Output: None
* Return: None
************************************************** *****************************/
void UART_Send( UINT8 dat)
{
    TI = 0;
    SBUF = dat;
    while( TI == 0) {;}
}
/************************************************* ******************************
* Function Name: UART_Receive
* Description: Serial port 0 byte reception
* Input: None
* Output: None
* Return: SBUF serial port receive byte
************************************************** *****************************/
UINT8 UART_Receive( void)
{
    while( RI == 0 ){;}
    RI = 0;
    return SBUF;
}
/************************************************* ******************************
* Function Name: CH55X_Respond
* Description: Chip response function when IAP is upgraded
* Input: s valid response byte
* Output: None
* Return: SBUF serial port receive byte
************************************************** *****************************/
void CH55X_Respond( UINT8 s)
{
    if( uart_bit == 2) // USB mode
    {
        Ep2Buffer[ MAX_PACKET_SIZE] = s;
        Ep2Buffer[ MAX_PACKET_SIZE+1] = 0X00;
        UEP2_T_LEN = 2;
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; // Allow upload
    }
    else //Serial port mode
    {
        UART_Send( s );
        UART_Send( 0x00 );
    }
}
/************************************************* ******************************
* Function Name: CH559_USB_ISPDownload
* Description: CH559 download function
*:
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH55X_IAPDownload( void)
{
    UINT8 s;
    UINT16 i;
    UINT16 len,Data;
    UINT32 addr;
    switch( iap_cmd.other.buf[0]) // Analyze the command code
    {
    case CMD_IAP_PROM: // ISP programming command
        len = iap_cmd.program.len>>1; //must be an integer multiple of 2, and operate as half-word
        addr = (iap_cmd.program.addr[0] | (UINT16)iap_cmd.program.addr[1]<<8);
        SAFE_MOD = 0x55;
        SAFE_MOD = 0xAA;
        GLOBAL_CFG |= bCODE_WE | bDATA_WE; //Write Flash
        for( i=0; i!=len; i++)
        {
            Data = (iap_cmd.program.buf[2*i] | (UINT16)iap_cmd.program.buf[2*i+1]<<8);
            s = ProgWord( addr,Data );
            addr+=2;
            if( s != 0x00)
            {
                break;
            }
        }
        SAFE_MOD = 0x55;
        SAFE_MOD = 0xAA;
        GLOBAL_CFG &= ~ (bCODE_WE | bDATA_WE );
        CH55X_Respond( s ); //Return to check
        break;
    case CMD_IAP_ERASE: // ISP erase command
        SAFE_MOD = 0x55;
        SAFE_MOD = 0xAA;
        GLOBAL_CFG |= bCODE_WE | bDATA_WE;
        addr = (iap_cmd.erase.addr[0] | (UINT16)iap_cmd.erase.addr[1]<<8);
        for( i=0; addr <IAP_CODE_ADDR; i++)
        {
            s = EraseBlock( addr );
            addr+=1024;
            if( s != 0)
            {
                break;
            }
        }
        SAFE_MOD = 0x55;
        SAFE_MOD = 0xAA;
        GLOBAL_CFG &= ~ (bCODE_WE | bDATA_WE );
        CH55X_Respond( s );
        break;
    case CMD_IAP_VERIFY: // ISP verification command
        addr = (iap_cmd.verify.addr[0] | (UINT16)iap_cmd.verify.addr[1]<<8);
        len = iap_cmd.verify.len>>1; // Must be an integer multiple of 2, and operate according to the word
        s = FlashVerify( addr,&(iap_cmd.verify.buf[0]),iap_cmd.verify.len );
        CH55X_Respond( s );
        break;
    case CMD_IAP_END: ​​// ISP end command
        SAFE_MOD = 0x55;
        SAFE_MOD = 0xAA;
        GLOBAL_CFG |= bSW_RESET; // Reset the microcontroller and enter the user program
        break;
    default:
        CH55X_Respond( 0xfe ); // unknown command
        break;
    }
}
/************************************************* ******************************
* Function Name: UATR_Handle
* Description: Serial port receive processing function
*:
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void UATR_Handle( void)
{
    UINT8 dat,i,len,add_value;
    dat = UART_Receive();
    if( dat == Uart_Sync_Head1) //data start byte
    {
        dat = UART_Receive();
        if( dat == Uart_Sync_Head2)
        {
            iap_cmd.other.buf[0] = UART_Receive(); //Command code
            add_value = 0;
            add_value+=iap_cmd.other.buf[0];
            len = iap_cmd.other.buf[1] = UART_Receive(); //Length of subsequent data
            add_value+=iap_cmd.other.buf[1];
            if( iap_cmd.other.buf[0] == CMD_IAP_PROM || iap_cmd.other.buf[0] == CMD_IAP_VERIFY )//The command code needs to add 2 bytes for programming verification
            {
                len+=2;
            }
            for( i=0; i!=len; i++)
            {
                iap_cmd.other.buf[i+2] = UART_Receive();
                add_value+=iap_cmd.other.buf[i+2];
            }
            i = UART_Receive();
            if( add_value != i)
            {
                UART_Send( 0x55 );
                UART_Send( 0xaa ); //accumulation and error, request the computer to resend
            }
            else
            {
                uart_bit = 1; //indicating to enter the serial port to download
                CH55X_IAPDownload( );
            }
        }
    }
}
/************************************************* ******************************
* Function Name: USB_DeviceInterrupt
* Description: USB interrupt query function, IAP program cannot use interrupt
*:
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void USB_DeviceInterrupt( void)
{
    UINT8 len;
    static UINT8 SetupReqCode, SetupLen;
    static PUINT8 pDescr;
    if( UIF_TRANSFER) // USB transfer completed
    {
        if (U_IS_NAK ){} // This example does not need to deal with NAK
        else
        {
            switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP) )//Analyze operation token and endpoint number
            {
            case UIS_TOKEN_OUT | 2: // endpoint 2# Batch endpoint download
                if (U_TOG_OK) // unsynchronized data packets will be discarded
                {
                    len = USB_RX_LEN;
                    memcpy( iap_cmd.other.buf,Ep2Buffer,len );
                    uart_bit = 2;
                    CH55X_IAPDownload( );
                }
                break;
            case UIS_TOKEN_IN | 2: // endpoint 2# Bulk endpoint upload
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;//Suspend upload
                break;
            case UIS_TOKEN_IN | 1: // endpoint 1# Interrupt endpoint upload
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
                    if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD )//Only supports standard requests
                    {
                        len = 0xFF; // operation failed
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
                            default:
                                len = 0xFF; // Descriptor type not supported
                                break;
                            }
                            if (SetupLen> len)
                            {
                                SetupLen = len; // limit the total length
                            }
                            len = SetupLen >= THIS_ENDP0_SIZE? THIS_ENDP0_SIZE: SetupLen; // This transmission length
                            memcpy( Ep0Buffer, pDescr, len ); // Load upload data
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
                        default:
                            len = 0xFF; // operation failed
                            break;
                        }
                    }
                }
                else
                {
                    len = 0xFF; // SETUP packet length error
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
                    memcpy( Ep0Buffer, pDescr, len ); // Load upload data
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
                    UEP0_T_LEN = 0; // The status phase is completed interrupted or a 0-length data packet is forced to upload to end the control transmission
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                }
                break;
            case UIS_TOKEN_OUT | 0: // endpoint 0# OUT
                switch( SetupReqCode)
                {
                default:
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // prepare for the next control transmission
                    break;
                }
                break;
            default:
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
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; // Clear interrupt flag
    }
    else if (UIF_SUSPEND) // USB bus suspend/wake up completed
    {
        UIF_SUSPEND = 0;
    }
    else // unexpected interruption, impossible situation
    {
        USB_INT_FG = 0xFF; // Clear interrupt flag
    }
}
/************************************************* ******************************
* Function Name: UART0_Init
* Description: Serial port initialization function
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void UART0_Init( void)
{
    SCON = 0x50;
    PCON |= SMOD;
    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;
    TH1 = 0-13;
    TR1 = 1;
    TI = 0;
    RI = 0;
    P0 = 0;
    PORT_CFG |= bP0_OC;
    P0_DIR |= bTXD_;
    P0_PU |= bTXD_ | bRXD_;
    PIN_FUNC = bUART0_PIN_X; /*Serial port is mapped to P0.2 and P0.3*/
    ES = 0;
}
/************************************************* ******************************
* Function Name: USB_DeviceInit
* Description: USB device mode initialization function
*:
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void USB_DeviceInit( void)
{
    IE_USB = 0;
    USB_CTRL = 0x00; /* Set the mode first*/
    UEP2_3_MOD = bUEP2_RX_EN | bUEP2_TX_EN; /* Endpoint 2 download OUT and upload IN */
    UEP0_DMA = Ep0Buffer;
    UEP2_DMA = Ep2Buffer;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK; /*Set the synchronization flag to automatically flip, receive ACK and send NAK*/
    USB_DEV_AD = 0x00;
    UHUB0_CTRL = bUH_DP_PD_DIS | bUH_DM_PD_DIS; /* Disable DP/DM pull-down resistor*/
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; /* Start the USB device and DMA, and automatically return to NAK before the interrupt flag is cleared during the interruption*/
    UHUB0_CTRL |= bUH_PORT_EN; /* Allow USB port */
    USB_INT_FG = 0xFF; /* Clear interrupt flag */
    IE_USB = 0; /* Turn off USB interrupt */
}
/************************************************* ******************************
* Function Name: mDelay20us(UNIT16 n)
* Description: 20us delay function, the main frequency is 12MHz, the delay is not accurate, other main frequencies refer to the delay function of DEBUG.C
* Input: UNIT16 n
* Output: None
* Return: None
************************************************** *****************************/
void mDelay20us( UINT16 n)
{
    for( n <<= 3; n; --n)
    {
        _nop_( );
    }
}

/************************************************* ******************************
* Function Name: main
* Description: main function
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void main( void)
{
    UINT16 i=0,j;
    EA = 0; // Turn off the interrupt, the interrupt cannot be used in IAP mode
    uart_bit = 0;
    P4_DIR |= 0x0f; // P40-P43 enable output
    P4_OUT = 0x0f;
    UART0_Init( ); /* Serial port initialization function, query method */
    USB_DeviceInit( ); /* USB device mode initialization function, query method */
    while(1)
    {
        i++;
        j++;
        if( RI)
        {
            UATR_Handle( ); // Serial port receive data processing
        }
        if( j> 30)
        {
            j = 0;
            if( USB_INT_FG)
            {
                USB_DeviceInterrupt( ); // Query usb interrupt, it is recommended not to query frequently
            }
        }
        if( i == 20000)
        {
            i = 0;
        }
        if( i == 0) //It is just a light indicator, meaningless
        {
            P4_OUT = 0x0f;
        }
        if( i == 10000)
        {
            P4_OUT = ~(1<<3); //Flashing light
        }
        mDelay20us(1); // Delay
        /* Exit iap download */
        if( DisableIAP == 0) // Query the soft reset when P10 is low and execute the user program again
        {
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            GLOBAL_CFG |= bSW_RESET; //Software resets the microcontroller and enters the user program */
        }
    }
}
