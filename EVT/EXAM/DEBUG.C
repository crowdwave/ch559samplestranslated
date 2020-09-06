/********************************** (C) COPYRIGHT *********** ********************
* File Name: DEBUG.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 DEBUG Interface
                     (1) Serial port 0 outputs print information, and the baud rate is variable;
************************************************** *****************************/

#include <stdio.h>
#include "CH559.H"

#define FREQ_SYS 12000000 //System frequency is 12MHz
#ifndef BUAD
#define BUAD 57600
#endif

/************************************************* ******************************
* Function Name: CfgFsys()
* Description: CH559 clock selection and configuration function, the default internal crystal oscillator 12MHz, if FREQ_SYS is defined
                   According to PLL_CFG and CLOCK_CFG configuration, the formula is as follows:
                   Fsys = (Fosc * (PLL_CFG & MASK_PLL_MULT ))/(CLOCK_CFG & MASK_SYS_CK_DIV);
                   The specific clock needs to be configured by yourself
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CfgFsys()
{
    SAFE_MOD = 0x55; //Enable safe mode
    SAFE_MOD = 0xAA;
// CLOCK_CFG |= bOSC_EN_XT; //Enable external crystal oscillator
// CLOCK_CFG &= ~bOSC_EN_INT;
// CLOCK_CFG &= ~MASK_SYS_CK_DIV;
// CLOCK_CFG |= 6; //Configure system clock 48MHz
// CLOCK_CFG |= 8; //Configure system clock 36MHz
// CLOCK_CFG |= 10; //Configure system clock 28.8MHz
// CLOCK_CFG |= 12; //Configure system clock 24MHz
// CLOCK_CFG |= 16; //Configure system clock 18MHz
/*56MHz
// CLOCK_CFG &= ~MASK_SYS_CK_DIV;
// CLOCK_CFG |= 6; //Configure system clock 56MHz
    PLL_CFG = 0xFC;
*/
    SAFE_MOD = 0xFF; //Close the safe mode
// If you modify the main frequency, you must modify FREQ_SYS at the same time, otherwise the delay function will be inaccurate
}

/************************************************* ******************************
* Function Name: mDelayus(UNIT16 n)
* Description: us delay function
* Input: UNIT16 n
* Output: None
* Return: None
************************************************** *****************************/
void mDelayuS( UINT16 n) // Delay in uS
{
while (n) {// total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
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
#if FREQ_SYS >= 42000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 44000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 46000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 48000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 50000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 52000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 54000000
++ SAFE_MOD;
#endif
#if FREQ_SYS >= 56000000
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
* Function Name: CH559UART0Alter()
* Description: CH559 serial port 0 pin mapping, serial port mapping to P0.2 and P0.3
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void CH559UART0Alter()
{
    PORT_CFG |= bP0_OC;
    P0_DIR |= bTXD_;
    P0_PU |= bTXD_ | bRXD_;
    PIN_FUNC |= bUART0_PIN_X; //Serial port is mapped to P0.2 and P0.3
}

/************************************************* ******************************
* Function Name: mInitSTDIO()
* Description: CH559 serial port 0 is initialized, T1 is used as the baud rate generator of UART0 by default, and T2 can also be used
                   As a baud rate generator
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void mInitSTDIO()
{
    UINT32 x;
    UINT8 x2;

    SM0 = 0;
    SM1 = 1;
    SM2 = 0; //Serial port 0 use mode 1
                                                                               //Use Timer1 as baud rate generator
    RCLK = 0; //UART0 receiving clock
    TCLK = 0; //UART0 sending clock
    PCON |= SMOD;
    x = 10 * FREQ_SYS / BUAD / 16; //If you change the main frequency, pay attention to the value of x not to overflow
    x2 = x% 10;
    x /= 10;
    if (x2 >= 5) x ++; //Rounding

    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1; //0X20, Timer1 as an 8-bit auto-reload timer
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK; //Timer1 clock selection
    TH1 = 0-x; //12MHz crystal oscillator, buad/12 is the actual baud rate to be set
    TR1 = 1; //Start timer 1
    TI = 1;
    REN = 1; //Serial port 0 receiving enable
}

/************************************************* ******************************
* Function Name: CH559UART0RcvByte()
* Description: CH559UART0 receives one byte
* Input: None
* Output: None
* Return: SBUF
************************************************** *****************************/
UINT8 CH559UART0RcvByte()
{
    while(RI == 0); //Query receiving, interrupt mode is not necessary
    RI = 0;
    return SBUF;
}

/************************************************* ******************************
* Function Name: CH559UART0SendByte(UINT8 SendDat)
* Description: CH559UART0 sends one byte
* Input: UINT8 SendDat; the data to be sent
* Output: None
* Return: None
************************************************** *****************************/
void CH559UART0SendByte(UINT8 SendDat)
{
SBUF = SendDat; //Query sending, interrupt mode can not use the following 2 sentences, but TI=0 before sending
while(TI == 0);
TI = 0;
}
