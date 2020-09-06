/********************************** (C) COPYRIGHT *********** ********************
* File Name: Timer1.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 TIME1 interface function
************************************************** *****************************/

#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

//CH559 Timer1 clock selection
//bTMR_CLK affects Timer0&1&2 at the same time, please pay attention when using
#define mTimer1ClkFsys() (T2MOD |= bTMR_CLK | bT1_CLK) //Timer, clock=Fsys
#define mTimer1Clk4DivFsys() (T2MOD &= ~bTMR_CLK;T2MOD |= bT1_CLK) //Timer, clock=Fsys/4
#define mTimer1Clk12DivFsys() (T2MOD &= ~(bTMR_CLK | bT1_CLK)) //Timer, clock=Fsys/12
#define mTimer1CountClk() (TMOD |= bT1_CT) //Counter, the falling edge of T1 pin is valid

//CH559 Timer1 start (SS=1)/end (SS=0)
#define mTimer1RunCTL( SS) (TR1 = SS? START: STOP)

/************************************************* ******************************
* Function Name: mTimer1ModSetup(UINT8 mode)
* Description: CH559 timer counter 1 mode setting
* Input: UINT8 mode, Timer1 mode selection
                   0: Mode 0, 13-bit timer, the upper 3 bits of TL1 are invalid
                   1: Mode 1, 16-bit timer
                   2: Mode 2, 8-bit automatic reload timer
                   3: Mode 3, stop Timer1
* Output: None
* Return: None
************************************************** *****************************/
void mTimer1ModSetup(UINT8 mode)
{
    TMOD &= 0x0f;
    TMOD |= mode << 4;
}

/************************************************* ******************************
* Function Name: mTimer1SetData(UINT16 dat)
* Description: CH559Timer1 TH1 and TL1 assignment
* Input: UINT16 dat; timer assignment
* Output: None
* Return: None
************************************************** *****************************/
void mTimer1SetData(UINT16 dat)
{
  UINT16 tmp;
  tmp = 65536-dat;
TL1 = tmp & 0xff;
TH1 = (tmp>>8) & 0xff;
}

/************************************************* ******************************
* Function Name: mTimer1Interrupt()
* Description: CH559 timer counter 1 timer counter interrupt processing function
************************************************** *****************************/
void mTimer1Interrupt( void) interrupt INT_NO_TMR1 using 1 //timer1 interrupt service routine, use register set 1
{//In mode 3, stop Timer1
    RXD_ = !RXD_;
// mTimer1SetData(0x3737); //For non-automatic reload mode, re-assign TH1 and TL1
}

main()
{
// CfgFsys( ); //CH559 clock selection configuration
    mDelaymS(5); //Wait for the external crystal oscillator to stabilize
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");

    mTimer1Clk12DivFsys( ); //Clock selects Fsys timer mode
    mTimer1ModSetup(2); //Method 2, automatic reload 8 is the timer
    mTimer1SetData(0x8080); //Timer assign initial value
    mTimer1RunCTL(1); //Start the timer
    ET1 = 1; //Enable timer counter 1 interrupt
    EA = 1; //Enable global interrupt
To
    while(1);
}

