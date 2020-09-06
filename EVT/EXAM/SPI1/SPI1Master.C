/********************************** (C) COPYRIGHT *********** ********************
* File Name: SPI1Master.C
* Author: WCH
* Version: V1.3
* Date: 2019/07/22
* Description: CH559 SPI1 master interface to operate SPI slave
************************************************** *****************************/

/*Hardware interface definition*/
/************************************************* *****************************
Use CH559 hardware SPI1 to operate CH376
Hardware interface
         CH559 DIR CH376
         P2.1 <==> MOSI
         P2.2 <==> MISO
         P2.3 <==> SCK
         P1.4 <==> SCS
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

UINT8 buffer,flag;
#define SET_SPI1_CK( d) {SPI1_CK_SE = d;} //d>=2

/************************************************* ******************************
* Function Name: CH559SPI1Init()
* Description: CH559SPI1 initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI1Init()
{
P1_DIR |= bSCS;
P2_DIR |= bMOSI1 | bSCK1; //Set the pin direction of the SPI interface
SPI1_CTRL |= bS1_SCK_OE | bS1_AUTO_IF; //MISO output enable, SCK output enable
SPI1_CTRL &= ~(bS1_DATA_DIR | bS1_2_WIRE); //Use 3-wire SPI, read data does not start transmission
//If bS1_DATA_DIR is enabled, a byte clock is automatically generated after data is sent for fast data transmission and reception
SPI1_CK_SE = 0x20; //Set the SPI working clock, you can configure it yourself
SPI1_CTRL &= ~bS1_CLR_ALL; //Clear the FIFO of SPI1, the default is 1, and it must be set to zero to send data
}

/************************************************* **************************** **
* Function Name: CH559SPI1Write(UINT8 dat)
* Description: SPI1 output data
* Input: UINT8 dat data
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI1Write(UINT8 dat)
{
// SCS = 0;
    SPI1_DATA = dat; //Output data
    while((SPI1_STAT & 0x08) == 0); //Wait for completion of transmission
//If bS1_DATA_DIR is 1, one byte of data can be directly read here for fast reading and writing
// SCS = 1;
}

/************************************************* ******************************
* Function Name: CH559SPI1Read()
* Description: SPI1 read data
* Input: None
* Output: None
* Return: UINT8 ret
************************************************** *****************************/
UINT8 CH559SPI1Read()
{
    SPI1_DATA = 0xff; //Start the clock
    while((SPI1_STAT & 0x08) == 0); //Wait for completion of transmission
    return SPI1_DATA;
}

main()
{
    UINT8 ret,i = 0;
// CfgFsys( ); //Clock configuration
    mDelaymS(5); //Wait for the internal crystal oscillator to stabilize, must be added

    P4_DIR |= bLED2;
    P3_DIR |= bTXD;
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("master_SPI1_start ...\n");
To
    CH559SPI1Init();
    mDelaymS(100); //Initialization of SPI1
    while(1)
    {
SCS = 0;
CH559SPI1Write(i);
mDelayuS(2);
ret = CH559SPI1Read();
SCS = 1;
if(ret != (i^0xff))
        {
            printf("Err: %02X %02X \n",(UINT16)i,(UINT16)ret); //If it is not equal to the inversion of the sent data, print the error message
        }
        else
        {
            printf("success %02x\n",(UINT16)i);
        }
i++;
mDelaymS(20);
    }
}
