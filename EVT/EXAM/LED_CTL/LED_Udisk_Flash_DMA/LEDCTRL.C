/********************************** (C) COPYRIGHT *********** **********************
* File Name: LEDCTRL.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559's DMA mode realizes LED control, you can use U disk to update the screen display content
                       Support U disk hot swap
************************************************** ********************************/

/*********************************The header file contains ************* *************************/
#include <stdio.h>
#include <string.h>
#include "../../CH559.H" //CH559 header file
#include "../../DEBUG.H" //Serial port module
#include "../../DEBUG.C"
#include "ROOT_UDISKIF.H" //U Disk Interface Module
#include "ROOT_UDISKIF.C"

#pragma NOAREGS

/*********************************The header file contains ************* *************************/
#define ScreenLength 1024 //The length of the screen, that is, the number of columns (bit)
#define ScreenWidth 16 //The width of the screen, that is, the number of lines (bit)
#define UnicodeSize 2048 //ScreenLength*ScreenWidth/8 The number of characters read from the USB flash drive
#define LargeUnicodeSize 1024 //byte
#define SingleSendSize 128 //ScreenLength/8 LED's DMA single send data length

#ifndef DEBUG //Print debugging information
#define DEBUG 1
#endif

// #if LargeUnicodeSize>=UnicodeSize
// UINT8X LEDBuffer[CountUnicode] _at_ 0x0000; //LED DMA sending buffer
// #else
UINT8X LEDBuffer[LargeUnicodeSize] _at_ 0x0000; //LED DMA sending buffer
UINT8X LEDBuffer1[LargeUnicodeSize] _at_ (0x0000+LargeUnicodeSize); //LED DMA sending buffer
// #endif

sbit LA = P2^0;
sbit LB = P2^1;
sbit LC = P2^2;
sbit LD = P2^3;
sbit EN = P2^4;
sbit STB = P2^5;
#define EN_L() {EN = 0;}
#define EN_H() {EN = 1;}
#define STB_L() {STB = 0;}
#define STB_H() {STB = 1;}
#define LINE0 {LD=0;LC=0;LB=0;LA=0;}
#define LINE1 {LD=0;LC=0;LB=0;LA=1;}
#define LINE2 {LD=0;LC=0;LB=1;LA=0;}
#define LINE3 {LD=0;LC=0;LB=1;LA=1;}
#define LINE4 {LD=0;LC=1;LB=0;LA=0;}
#define LINE5 {LD=0;LC=1;LB=0;LA=1;}
#define LINE6 {LD=0;LC=1;LB=1;LA=0;}
#define LINE7 {LD=0;LC=1;LB=1;LA=1;}
#define LINE8 {LD=1;LC=0;LB=0;LA=0;}
#define LINE9 {LD=1;LC=0;LB=0;LA=1;}
#define LINE10 {LD=1;LC=0;LB=1;LA=0;}
#define LINE11 {LD=1;LC=0;LB=1;LA=1;}
#define LINE12 {LD=1;LC=1;LB=0;LA=0;}
#define LINE13 {LD=1;LC=1;LB=0;LA=1;}
#define LINE14 {LD=1;LC=1;LB=1;LA=0;}
#define LINE15 {LD=1;LC=1;LB=1;LA=1;}


/******************************SPI0 Flash***************** *********************/
/************************************************* ******************************
* Function Name: InitSPI_Host( void)
* Description: SPI0 master mode initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitSPI_Host( void)
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER); // set to host mode
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE; //The host writes, the write transfer is not started by default, if bS0_DATA_DIR is enabled,
                                                                              //Then automatically generate a byte clock after sending data for fast data sending and receiving
    P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 ); //bMOSI, bSCK, bSCS are set to the output direction
    P1_DIR &= ~bMISO;
    SPI0_CK_SE = 0x02; //Frequency is 6M
// SPI0_STAT = 0xFF; // Clear interrupt flag
// IE_SPI0 = 1;
}

/************************************************* ******************************
* Function Name: SPI0_MASTER_Trans
* Description: Send one byte of data
* Input: buffer-data to be sent
* Output: None
* Return: None
************************************************** *****************************/
void SPI0_MASTER_Trans( UINT8 dat)
{
    SPI0_DATA = dat;
    while( S0_FREE == 0 ); //Wait for the completion of data transmission
}

/************************************************* ******************************
* Function Name: SPI0_MASTER_Recv
* Description: Receive one byte of data
* Input: None
* Output: None
* Return: data received
************************************************** *****************************/
UINT8 SPI0_MASTER_Recv( void)
{
    SPI0_DATA = 0xFF;
    while( S0_FREE == 0 ); //Waiting for data to come back
    return SPI0_DATA;
}

/************************************************* ******************************
* Function Name: Read_Status_Register
* Description: Used to read the status register and return the value of the status register. To ensure the speed, the function is not called
* Input: None
* Output: None
* Return: SPI0_DATA-register status value
************************************************** *****************************/
UINT8 Read_Status_Register( void)
{
    UINT8 byte = 0;
    SCS = 0; //Enable the device
    SPI0_DATA = 0x05; //Send the command to read the status register
    while( S0_FREE == 0 ); //Wait for the completion of data transmission
    SPI0_DATA = 0xFF;
    while( S0_FREE == 0 ); //Read status register
    SCS = 1; //Disable device
    return SPI0_DATA;
}

/************************************************* ******************************
* Function Name: Wait_Busy
* Description: Wait for the chip to be free (after executing Byte-Program, Sector-Erase, Block-Erase, Chip-Erase operations)
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void Wait_Busy( void)
{
    while ((Read_Status_Register())&0x01 == 0x01) //waste time until not busy
          Read_Status_Register( );
}

/************************************************* ******************************
* Function Name: WRSR
* Description: write a byte to the status register
* Input: byte-the data written
* Output: None
* Return: None
************************************************** *****************************/
void WRSR( UINT8 byte)
{
    SCS = 0; //Enable the device
    SPI0_MASTER_Trans(0x01); //Send write status register
    SPI0_MASTER_Trans(byte); //Change BPx or BPL in the register (only 2, 3, 4, 5, 7 bits can be rewritten)
    SCS = 1; //Disable device
}

/************************************************* ******************************
* Function Name: WREN
* Description: write enable, can also be used to enable write status register
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void WREN( void)
{//SCS conflicts with download, use AIN3 instead
   SCS = 0;
   SPI0_MASTER_Trans(0x06); //Send WREN command
   SCS = 1;
}

/************************************************* ******************************
* Function Name: WREN_Check
* Description: Check whether the WEL bit is 1 before the erase/write operation
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void WREN_Check( void)
{
    UINT8 byte;
    byte = Read_Status_Register( ); //Read status register
    if ((byte&0x02) != 0x02) //Check the WEL position
    {
        WREN( ); //If it is not set to 1, perform corresponding processing, such as writing to it to make operation
    }
}

/************************************************* ******************************
* Function Name: Chip_Erase
* Description: Erase
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void Chip_Erase( void)
{
    WREN_Check();
    SCS = 0;
    SPI0_MASTER_Trans(0x60); //Send Chip Erase command (60h or C7h)
    SCS = 1;
    Wait_Busy();
}

/************************************************* ******************************
* Function Name: FlashRead
* Description: Read the data of SingleSendSize bytes in the starting address. Return the read data, high-speed reading
* Input: Dst -Destination Address 000000H-1FFFFFH
                   buffer Start address of receiving buffer
* Output: None
* Return: None
************************************************** *****************************/
void FlashRead(UINT32 Dst,PUINT8 buffer)
{
    UINT8 i;
    SCS = 0; //enable device
    SPI0_DATA = 0x03; //read command
    while( S0_FREE == 0 ); //Wait for the completion of data transmission
    SPI0_DATA = ((Dst & 0xFFFFFF) >> 16); //send 3 address bytes
    while( S0_FREE == 0 ); //Wait for the completion of data transmission
    SPI0_DATA = ((Dst & 0xFFFF) >> 8);
    while( S0_FREE == 0 );
    SPI0_DATA = Dst & 0xFF;
    while( S0_FREE == 0 );
    for(i=0;i<SingleSendSize;i++)
    {//Wait for data to return
        SPI0_DATA = 0xFF;
        while( S0_FREE == 0 );
        *(buffer+i) = SPI0_DATA;
    }
    SCS = 1; //disable device
}

/************************************************* ******************************
* Function Name: Byte_Program
* Description: write data
* Input: Dst -Destination Address 000000H-1FFFFFH
* byte-the data to be written
* Output: None
* Return: None
************************************************** *****************************/
void Byte_Program(UINT32 Dst, UINT8 byte)
{
    WREN();
    SCS = 0; //chip enable
    SPI0_MASTER_Trans(0x02); //Send write operation instruction
    SPI0_MASTER_Trans(((Dst & 0xFFFFFF) >> 16)); //Send 3-byte address
    SPI0_MASTER_Trans(((Dst & 0xFFFF) >> 8));
    SPI0_MASTER_Trans(Dst & 0xFF);
    SPI0_MASTER_Trans(byte); //Send the data to be written
    SCS = 1;
    Wait_Busy();
}

/************************************************* ******************************
* Function Name: CopyData2Flash(UINT16 Num,UINT16 Addr)
* Description: Read out the corresponding Chinese character code and store it in Flash
* Input: UINT16 Num The number of bytes written to Flash this time
                   UINT16 Addr The starting address of this write Flash
* Output: None
* Return: None
************************************************** *****************************/
void CopyData2Flash(UINT16 Num,UINT16 Addr)
{
UINT16 i;

for(i=0;i<Num;i++)
{
            Byte_Program( Addr+i ,LEDBuffer1[i] );
}
}
/****************************SPI0 Flash END****************** *******************/

/*****************************Line selection and IO configuration*************** ***********************/
/************************************************* ******************************
* Function Name: InitLED(void)
* Description: LED
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void InitLED()
{
    P2_DIR = 0xff;
    P3_DIR = 0xff;
    //P4_DIR = 0xff;
    PORT_CFG |= bP2_DRV | bP3_DRV;
    EN_H( );
    STB_L( );
    LED_CTRL = 0x00|bLED_OUT_EN |bLED_BIT_ORDER;
    LED_CK_SE=0x06;
}

/************************************************* ******************************
* Function Name: Showline(UINT8 LineNum)
* Description: LED
* Input: UINT8 LineNum
* Output: None
* Return: None
************************************************** *****************************/
void Showline(UINT8 LineNum)
{
    switch(LineNum)
{
         case 0: LINE0;break;
         case 1: LINE1;break;
         case 2: LINE2;break;
         case 3: LINE3;break;
         case 4: LINE4;break;
         case 5: LINE5;break;
         case 6: LINE6;break;
         case 7: LINE7;break;
         case 8: LINE8;break;
         case 9: LINE9;break;
         case 10:LINE10;break;
         case 11:LINE11;break;
         case 12:LINE12;break;
         case 13:LINE13;break;
         case 14:LINE14;break;
         case 15:LINE15;break;
         default:break;
    }
}
/******************************IO end***************** **************************/

/************************************************* ******************************
* Function Name: ReadDisplayFile()
* Description: Read the Chinese characters that need to be displayed in the U disk (each Chinese character corresponds to 2 characters), and store them in Flash
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void ReadDisplayFile()
{
    UINT8 s,i;
    UINT16 len;
To
WREN( ); //FLASH write enable
WRSR( 0x00 ); //Write register
Chip_Erase( ); //FLASH whole chip erase

    strcpy( mCmdParam.Open.mPathName, "/COMMAND.TXT" ); //Set the file name to be opened
    s = CH559FileOpen( );
    if (s == ERR_MISS_DIR || s == ERR_MISS_FILE)
    {
        printf( "Miss File,Please Check the file name...\n" );
    }
    else
    {
      mCmdParam.ByteLocate.mByteOffset = 602; //Set the offset pointer
      s = CH559ByteLocate();
      mStopIfError( s );
      for(i=0;i<(UnicodeSize/LargeUnicodeSize);i++) //Length is greater than LargeUnicodeSize
      {//Read LargeUnicodeSize each time
          mCmdParam.ByteRead.mByteCount = LargeUnicodeSize;
          mCmdParam.ByteRead.mByteBuffer = LEDBuffer1; //Set the read file buffer address
          s = CH559ByteRead( );
          mStopIfError( s );
          printf("%02X \n",(UINT16)i);
#if DEBUG
          for(len=0;len<LargeUnicodeSize;len++)
          {
              printf("%02X ",(UINT16)LEDBuffer1[len]);
          }
#endif
          CopyData2Flash(LargeUnicodeSize,i*LargeUnicodeSize); //Write to Flash
      }
      mCmdParam.ByteRead.mByteCount = UnicodeSize%LargeUnicodeSize; //Read remaining bytes
      mCmdParam.ByteRead.mByteBuffer = LEDBuffer1; //Set the read file buffer address
      s = CH559ByteRead( );
      mStopIfError( s );
#if DEBUG
      for(len=0;len<(UnicodeSize%LargeUnicodeSize);len++)
      {
          printf("%02X ",(UINT16)LEDBuffer1[len]);
      }
#endif
      CopyData2Flash(UnicodeSize%LargeUnicodeSize,i*LargeUnicodeSize); //Write to Flash
  }
  mCmdParam.Close.mUpdateLen = 0; //It is forbidden to update the file length
  s = CH559FileClose( ); //Close the file
  mStopIfError( s );
}

/************************************************* ******************************
* Function Name: SendLeddata(UINT8 lineNum)
* Description: Send column data in DMA mode
* Input: UINT8 lineNum
* Output: None
* Return: None
************************************************** *****************************/
void SendLeddata(UINT8 lineNum)
{
    UINT8 i,len;
    len = SingleSendSize/2;
    i=0;
To
    if(lineNum%2 == 0)
    {
        LED_DMA = LEDBuffer1;
        LED_DMA_CN = len; //Since DMA is a double word, divide by 2 here
        LED_CTRL |= bLED_DMA_EN;
        To
        FlashRead((UINT16)SingleSendSize*((lineNum+1)%ScreenWidth),LEDBuffer); //Read data from FLASH
    }
    else
    {
        LED_DMA = LEDBuffer;
        LED_DMA_CN = len; //Since DMA is a double word, divide by 2 here
        LED_CTRL |= bLED_DMA_EN;

        FlashRead((UINT16)SingleSendSize*((lineNum+1)%ScreenWidth),LEDBuffer1);//Read data from FLASH
    }
    while(LED_FIFO_CN||!(LED_STAT&bLED_FIFO_EMPTY));
    LED_CTRL &= ~ bLED_DMA_EN;
#if 0
    for(i=0;i<SingleSendSize;i++)
    {
        if(lineNum%2 == 0)
        {
            printf("%02X ",(UINT16)LEDBuffer[i]);
        }
        else
        {
            printf("%02X ",(UINT16)LEDBuffer1[i]);
        }
    }
    printf("\n");
#endif
}


/************************************************* ******************************
* Function Name: Leddisplay(void)
* Description: Screen display function
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void Leddisplay()
{
    UINT8 i;
#if 0
    printf("benin\n");
#endif
    for(i=0;i<16;i++)
    {
STB_H();
mDelayuS(2);
STB_L();
SendLeddata(i);
EN_L();
Showline(i);
mDelayuS(500);
        EN_H();
    }
}

/**********************************Main function************* **************************/
void main()
{
    UINT8 s,i;
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
    mInitSTDIO( ); //In order to let the computer monitor the demonstration process through the serial port
    printf("Start LED contol....\n");
    InitUSB_Host( );
    InitLED( );
    InitSPI_Host( );
    CH559LibInit( ); //Initialize CH559 library to support U disk files
    FoundNewDev = 0;
    UIF_DETECT=0;
    FlashRead(0,LEDBuffer1); //Read the first line of data from FLASH
    while(1)
    {
        if (UIF_DETECT) //Detect U port device plug
        {
            UIF_DETECT = 0; //Clear interrupt flag
            s = AnalyzeRootHub( ); //Analyze ROOT-HUB status
            if( s == ERR_USB_CONNECT)
            {
                FoundNewDev = 1;
            }
            if( FoundNewDev || s == ERR_USB_CONNECT) //There is a new USB device inserted
            {
                FoundNewDev = 0;
                mDelaymS( 200 ); //Because the USB device has not been stable after being inserted, wait for hundreds of milliseconds for the USB device to eliminate the plugging jitter
                s = InitRootDevice( ); //Initialize the USB device
                if( s == ERR_SUCCESS)
                {
                    // U disk operation process: USB bus reset, U disk connection, get device descriptor and set USB address, optional get configuration descriptor, then arrive here, CH559 subroutine library will continue to complete the follow-up work
                    CH559DiskStatus = DISK_USB_ADDR;
                    for( i = 0; i != 10; i ++)
                    {
                        s = CH559DiskReady( );
                        if (s == ERR_SUCCESS)
                        {
                            break;
                        }
                        mDelaymS( 50 );
                    }
                    if( CH559DiskStatus >= DISK_MOUNTED) //U disk is ready
                    {
                        printf("Read Command File....\n");
                        ReadDisplayFile();
                        printf("Finish Reading File\n");
                    }
                    else
                    {
                        printf( "U_Disk not ready ERR =%02X\n", (UINT16)s );
                    }
                 }
                 else
                 {
                     printf("U_Disk Init Failed,Please retry again\n");
                 }
             }
             SetUsbSpeed( 1 ); // The default is full speed
        }
  Leddisplay( ); //Static display function
    }
}



