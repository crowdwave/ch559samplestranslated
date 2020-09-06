/********************************** (C) COPYRIGHT *********** ********************
* File Name: SPI0Mater.C
* Author: WCH
* Version: V1.3
* Date: 2019/07/22
* Description: CH559 provides SPI0 host mode operation interface function
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#define SPI0Interrupt 0 //Set the SPI0 data receiving and sending interrupt mode or query mode
UINT8 flag;
UINT8 TmpBuf;

#pragma NOAREGS
#define SET_SPI0_CK( d) {SPI0_CK_SE = d;} //d>=2

/*Hardware interface definition*/
/************************************************* *****************************
Use CH559 hardware SPI interface
         CH559 DIR
         P1.4 <==> SCS
         P1.5 <==> MOSI
         P1.6 <==> MISO
         P1.7 <==> SCK
************************************************** *****************************/


/************************************************* ******************************
* Function Name: CH559SPI0HostInit()
* Description: CH559SPI0 initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI0HostInit(void)
{
    PORT_CFG &= ~bP1_OC;
    P1_DIR |= (bSCK | bMOSI | bSCS);
    P1_IE |= bMISO; //Pin configuration
To
    SPI0_SETUP &= ~(bS0_MODE_SLV | bS0_BIT_ORDER); //Set to host mode, byte order is big endian
    SPI0_CTRL |= bS0_MOSI_OE | bS0_SCK_OE; //MISO output enable, SCK output enable
    SPI0_CTRL &= ~(bS0_MST_CLK | bS0_2_WIRE);
    SPI0_CTRL &= ~(bS0_DATA_DIR); //The host writes, the write transmission is not started by default, if bS0_DATA_DIR is enabled,
//Then automatically generate a byte clock after sending data for fast data sending and receiving
    SET_SPI0_CK(6); // divide by 6
    SPI0_CTRL &= ~bS0_CLR_ALL; //Clear the FIFO of SPI0, the default is 1, and it must be set to zero to send data
}

/************************************************* ******************************
* Function Name: CH559SPI0Write(UINT8 dat)
* Description: CH559 hardware SPI write data
* Input: UINT8 dat data
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI0Write(UINT8 dat)
{
    SPI0_DATA = dat;
    while(S0_FREE == 0); //Wait for completion of transmission
//If bS0_DATA_DIR is 1, one byte of data can be directly read here for fast reading and writing
}

/************************************************* ******************************
* Function Name: CH559SPI0Read()
* Description: CH559 hardware SPI0 read data
* Input: None
* Output: None
* Return: UINT8 ret
************************************************** *****************************/
UINT8 CH559SPI0Read()
{
    SPI0_DATA = 0xff;
    while(S0_FREE == 0);
    return SPI0_DATA;
}



void main()
{
    UINT8 ret,i=0;
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
// CfgFsys( );
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
    CH559SPI0HostInit(); //SPI0 host mode initialization
mDelaymS(100);
    while(1)
    {
SCS = 0; //SPI host sends data
        CH559SPI0Write(i);
        mDelaymS(5);
        ret = CH559SPI0Read(); //Receive the data returned by the SPI slave, reverse and return
        SCS = 1;
        if(ret != (i^0xff))
        {
            printf("Err: %02X %02X \n",(UINT16)i,(UINT16)ret); //If it is not equal to the inversion of the sent data, print the error message
        }
        else
        {
            printf("success %02x\n",(UINT16)i);
        }
        i = i+1;
        mDelaymS(50);
    }
}
