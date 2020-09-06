/********************************** (C) COPYRIGHT *********** ********************
* File Name: SPI0Slv.C
* Author: WCH
* Version: V1.3
* Date: 2019/07/22
* Description: CH559 provides SPI0 slave mode operation interface function
Note: When the chip select is valid, the preset value of SPI0_S_PRE is automatically loaded from the opportunity to the transmit shift buffer, so it is better to select
Write the pre-transmission value to the SPI0_S_PRE register before it is valid, or discard the first received byte at the host side. Note that the host will first
Take the value in SPI0_S_PRE to generate an S0_IF_BYTE interrupt.
If the chip select is from invalid to valid, and the slave transmits first, it is best to put the first byte of the output in the SPI0_S_PRE register;
If the chip select is already active, the data data can use SPI0_DATA.
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"
#pragma NOAREGS

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
* Function Name: CH559SPI0SlvWrite(UINT8 dat)
* Description: CH559 hardware SPI write data, slave mode
* Input: UINT8 dat data
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI0SlvWrite(UINT8 dat)
{
    SPI0_DATA = dat;
    while(S0_FREE==0)
    {
        ;
    }
}
/************************************************* ******************************
* Function Name: CH559SPI0SlvRead()
* Description: CH559 hardware SPI0 read data, slave mode
* Input: None
* Output: None
* Return: UINT8 ret
************************************************** *****************************/
UINT8 CH559SPI0SlvRead()
{
    while(S0_FREE == 0)
    {
        ;
    }
    return SPI0_DATA;
}
/************************************************* ******************************
* Function Name: CH559SPI0InterruptInit()
* Description: CH559SPI0 interrupt initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI0InterruptInit()
{
    //IP_EX |= bIP_SPI0; //SPI0 interrupt priority setting
    //SPI0_SETUP |= bS0_IE_FIRST; //Enable the first byte receive interrupt
    //Generally used for the first byte is the command code
    SPI0_SETUP |= bS0_IE_BYTE; //Enable receiving 1-byte interrupt, enable FIFO overflow interrupt
    SPI0_CTRL |= bS0_AUTO_IF; //Automatically clear the S0_IF_BYTE interrupt flag
    SPI0_STAT |= 0xff; //Clear SPI0 interrupt flag
    IE_SPI0 = 1; //Enable SPI0 interrupt
EA = 1; //Turn on the total interrupt
}

/************************************************* ******************************
* Function Name: CH559SPI0Init()
* Description: CH559SPI0 initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH559SPI0Init(void)
{
    SPI0_S_PRE = 0x59; //Preset data, used for exchange data obtained after the SPI host sends the first byte of command code
    PORT_CFG &= ~bP1_OC;
    P1_DIR &= 0x0F;
    P1_PU &= 0x0F; //SCS, MISO, MOSI, SCK pins are set as high-impedance input, chip selection is valid and MISO automatically enables output
    SPI0_SETUP |= bS0_MODE_SLV; //Set to device mode,
    SPI0_SETUP &= ~ (bS0_BIT_ORDER); //The byte order is big endian
    SPI0_CTRL &= ~(bS0_MOSI_OE | bS0_SCK_OE); //Turn off SCK and MOSI output
    SPI0_CTRL |= bS0_MISO_OE;
    SPI0_CTRL &= ~(bS0_MST_CLK | bS0_DATA_DIR | bS0_2_WIRE);
    SPI0_CTRL &= ~bS0_CLR_ALL; //Clear the FIFO of SPI0, the default is 1, and it must be set to zero to send data
}

#ifdef SPI0Interrupt
/************************************************* ******************************S
* Function Name: SPI0HostInterrupt(void)
* Description: SPI0 slave mode interrupt service routine
* Input: None
* Output: None
* Return: UINT8 ret
************************************************** *****************************/
void SPI0HostInterrupt( void) interrupt INT_NO_SPI0 //* SPI0 interrupt service routine, use register set 1
{
UINT8 dat;
    dat = CH559SPI0SlvRead();
    CH559SPI0SlvWrite(dat^0xFF);
    printf("Read#%02x\n",(UINT16)dat);
}
#endif

main()
{
    UINT8 ret;
    mDelaymS(10); //Power-on delay, wait for the internal crystal oscillator to stabilize
// CfgFsys( );
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
    CH559SPI0Init();
#ifdef SPI0Interrupt
    CH559SPI0InterruptInit();
#endif
To
    while(1)
    {
#ifndef SPI0Interrupt
        ret = CH559SPI0SlvRead(); //The host keeps CS=0
        CH559SPI0SlvWrite(ret^0xFF); //SPI waits for the host to take the data away, the SPI host sets CS=0 before each reading, and CS=1 after reading
        printf("Read#%02x\n",(UINT16)ret);
#endif
    }
}
