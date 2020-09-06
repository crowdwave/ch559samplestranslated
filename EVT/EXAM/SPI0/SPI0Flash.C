/********************************** (C) COPYRIGHT *********** ********************
* File Name: SPI0Flash.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 SPI0 read and write external Flash
************************************************** *****************************/
#include ".\DEBUG.C" //Printing debugging information
#include ".\DEBUG.H"

#pragma NOAREGS

sbit CHIP_SELECT = P1^4;
#define SENDBYTE_SPI( d) {SPI0_DATA = d;while(S0_FREE == 0);}
#define RECVBYTE_SPI( d) {SPI0_DATA = 0xff;while(S0_FREE == 0);d = SPI0_DATA;}

UINT8 buf[50];

/************************************************* ******************************
* Function Name: InitHostSPI0( void)
* Description: SPI0 master mode initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitHostSPI0( void)
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER); /*Set to host mode*/
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE; /*Host writes, the default does not start the write transfer, if bS0_DATA_DIR is enabled*/
                                                                               /*Then automatically generate a byte clock after sending data for fast data sending and receiving*/
    P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 ); /*bMOSI, bSCK, bSCS are set as output direction*/
    P1_DIR &= ~bMISO;
    SPI0_CK_SE = 0x02; /*Frequency is 6M*/
// SPI0_STAT = 0xFF; /*Clear interrupt flag*/
// IE_SPI0 = 1;
}

/************************************************* ******************************
* Function Name: ReadExternalFlashStatusReg_SPI
* Description: used to read the status register and return the value of the status register
* Input: None
* Output: None
* Return: ExFlashRegStatus
************************************************** *****************************/
UINT8 ReadExternalFlashStatusReg_SPI( void)
{
    UINT8 ExFlashRegStatus;
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x05); /*Send command to read status register */
    RECVBYTE_SPI(ExFlashRegStatus); /*Read status register*/
    CHIP_SELECT = 1;
    return ExFlashRegStatus;
}

/************************************************* ******************************
* Function Name: WaitExternalFlashIfBusy
* Description: Wait for the chip to be free (after executing Byte-Program, Sector-Erase, Block-Erase, Chip-Erase operations)
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void WaitExternalFlashIfBusy( void)
{
    while ((ReadExternalFlashStatusReg_SPI())&0x01 == 0x01) /*Wait until Flash is free*/
    {
        ReadExternalFlashStatusReg_SPI( );
    }
}

/************************************************* ******************************
* Function Name: WriteExternalFlashStatusReg_SPI
* Description: write a byte to the status register
* Input: status-the data written
* Output: None
* Return: None
************************************************** *****************************/
void WriteExternalFlashStatusReg_SPI( UINT8 status)
{
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x01); /*Send write status register*/
    SENDBYTE_SPI(status); /*Change BPx or BPL in the register (only 2,3,4,5,7 bits can be rewritten)*/
    CHIP_SELECT = 1;
}

/************************************************* ******************************
* Function Name: WriteExternalFlashEnable_SPI
* Description: write enable, can also be used to enable write status register
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void WriteExternalFlashEnable_SPI( void)
{
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x06); /*Send write enable command*/
    CHIP_SELECT = 1;
}

/************************************************* ******************************
* Function Name: CheckExternalFlashWriteEnable_SPI
* Description: Check whether the WEL bit is 1 before the erase/write operation
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CheckExternalFlashWriteEnable_SPI( void)
{
    UINT8 WRENStatus;
    WRENStatus = ReadExternalFlashStatusReg_SPI(); /*Read status register*/
    if ((WRENStatus&0x02) != 0x02) /*Check WEL position bit*/
    {
        WriteExternalFlashEnable_SPI( ); /*If it is not set to 1, perform corresponding processing, such as write enable operation on it*/
    }
}

/************************************************* ******************************
* Function Name: EraseExternalFlash_SPI
* Description: Erase external Flash
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void EraseExternalFlash_SPI( void)
{
    CheckExternalFlashWriteEnable_SPI();
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x60); /*Send Chip Erase command (60h or C7h)*/
    CHIP_SELECT = 1;
    WaitExternalFlashIfBusy();
}

/************************************************* ******************************
* Function Name: ByteReadExternalFlash_SPI
* Description: Read one byte of data in an address. Return the read data
* Input: UINT32 StarAddr -Destination Address 000000H-1FFFFFH
* Output: None
* Return: byte-the data read
************************************************** *****************************/
UINT8 ByteReadExternalFlash_SPI(UINT32 StarAddr)
{
    UINT8 dat = 0;
    CHIP_SELECT = 0; //enable device
    SENDBYTE_SPI(0x03); //read command
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16)); //send 3 address bytes
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    RECVBYTE_SPI(dat);
    CHIP_SELECT = 1; //disable device
    return dat; //return one byte read
}

/************************************************* ******************************
* Function Name: ByteWriteExternalFlash_SPI
* Description: write data
* Input: StarAddr -Destination Address 000000H-1FFFFFH
* dat-the data to be written
* Output: None
* Return: None
************************************************** *****************************/
void ByteWriteExternalFlash_SPI(UINT32 StarAddr, UINT8 dat)
{
    WriteExternalFlashEnable_SPI();
    CHIP_SELECT = 0; //chip enable
    SENDBYTE_SPI(0x02); //Send write operation instruction
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16)); //Send 3-byte address
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    SENDBYTE_SPI(dat); //Send the data to be written
    CHIP_SELECT = 1;
    WaitExternalFlashIfBusy();
}

/************************************************* ******************************
* Function Name: BlukReadExternalFlash_SPI
* Description: Read the data of multiple bytes (Len) in the start address (StarAddr) and store it in the buffer RcvBuffer
* Input: StarAddr -Destination Address 000000H-1FFFFFH
                   Len read data length
                   RcvBuffer receiving buffer start address
* Output: None
* Return: None
************************************************** *****************************/
void BlukReadExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 RcvBuffer)
{
    UINT16 i;
    for(i=0; i<Len; i++) /*receive data*/
    {
        RcvBuffer[i] = ByteReadExternalFlash_SPI(StarAddr);
        StarAddr++; /*Read the next address*/
    }
}

/************************************************* ******************************
* Function Name: BlukWriteExternalFlash_SPI
* Description: Write data to external Flash
* Input: StarAddr -Destination Address 000000H-1FFFFFH
                   Len send data length
* SendBuffer-Send data buffer
* Output: None
* Return: None
************************************************** *****************************/
void BlukWriteExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 SendBuffer)
{
    UINT16 i;
    for(i=0; i<Len; i++) /*Send the data to be written*/
    {
        ByteWriteExternalFlash_SPI(StarAddr,*(SendBuffer+i));
        StarAddr++;
    }
}

void main()
{
    UINT32 i;
// mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize
    mInitSTDIO( ); /* In order to let the computer monitor the demonstration process through the serial port */
    printf( "Start SPI FLASH @ChipID=%02X\n", (UINT16)CHIP_ID );

    InitHostSPI0( );
#if 0
    printf("Address(0xF8) = %02x\n",(UINT16)SPI0_STAT);
    printf("Address(0xF9) = %02x\n",(UINT16)SPI0_DATA);
    printf("Address(0xFA) = %02x\n",(UINT16)SPI0_CTRL);
    printf("Address(0xFB) = %02x\n",(UINT16)SPI0_CK_SE);
    printf("Address(0xFC) = %02x\n",(UINT16)SPI0_SETUP);
#endif
    WriteExternalFlashEnable_SPI( ); //FLASH write enable
    WriteExternalFlashStatusReg_SPI( 0x00 ); //Write register
    EraseExternalFlash_SPI( ); //FLASH whole chip erase
    printf("Chip_Erase over\n");
    for(i = 0; i <50; i ++)
    {
        buf[i] = i;
    }
    BlukWriteExternalFlash_SPI(0,50,&buf[0]); //Write data at FLASH address 0x00000000
    printf("Write over\n");
    BlukReadExternalFlash_SPI(0,50,&buf[0]); //Read data from FLASH address 0x00000000
    for(i = 0; i <50; i ++)
    {
        printf(" %02x ",(UINT16)buf[i]);
    }
while(1);
}
