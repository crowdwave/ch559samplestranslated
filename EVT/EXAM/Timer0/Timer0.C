/********************************** (C) COPYRIGHT *********** ********************
* File Name: Timer0.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 Timer0 interface function
************************************************** *****************************/

#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

//CH559 Timer0 clock selection
//bTMR_CLK affects Timer0&1&2 at the same time, please pay attention when using
#define mTimer0ClkFsys() (T2MOD |= bTMR_CLK | bT0_CLK) //Timer, clock=Fsys
#define mTimer0Clk4DivFsys() (T2MOD &= ~bTMR_CLK;T2MOD |= bT0_CLK) //Timer, clock=Fsys/4
#define mTimer0Clk12DivFsys() (T2MOD &= ~(bTMR_CLK | bT0_CLK)) //Timer, clock=Fsys/12
#define mTimer0CountClk() (TMOD |= bT0_CT) //Counter, the falling edge of T0 pin is valid

//CH559 Timer0 start (SS=1)/end (SS=0)
#define mTimer0RunCTL( SS) (TR0 = SS? START: STOP)

/************************************************* ******************************
* Function Name: mTimer0ModSetup(UINT8 mode)
* Description: CH559 timer counter 0 mode 0 setting
* Input: UINT8 mode, Timer0 mode selection
                   0: Mode 0, 13-bit timer, the upper 3 bits of TL0 are invalid
                   1: Mode 1, 16-bit timer
                   2: Mode 2, 8-bit automatic reload timer
                   3: Mode 3, two 8-bit timers
* Output: None
* Return: None
************************************************** *****************************/
void mTimer0ModSetup(UINT8 mode)
{
    TMOD &= 0xf0;
    TMOD |= mode;
}

/************************************************* ******************************
* Function Name: mTimer0SetData(UINT16 dat)
* Description: CH559Timer0 TH0 and TL0 assignment
* Input: UINT16 dat; timer assignment
* Output: None
* Return: None
************************************************** *****************************/
void mTimer0SetData(UINT16 dat)
{
    UINT16 tmp;
    tmp = 65536-dat;
    TL0 = tmp & 0xff;
    TH0 = (tmp>>8) & 0xff;
}

/************************************************* ******************************
* Function Name: mTimer0Interrupt()
* Description: CH559 timer counter 0 timer counter interrupt processing function
************************************************** *****************************/
void mTimer0Interrupt( void) interrupt INT_NO_TMR0 using 1 //timer0 interrupt service routine, use register set 1
{//In mode 3, TH0 uses the interrupt resource of Timer1
    CAP1 = !CAP1;
// mTimer0SetData(0x2000) //The non-automatic reload mode needs to re-assign TH0 and TL0
}

main()
{
// CfgFsys( ); //CH559 clock selection configuration
    mDelaymS(5); //Wait for the external crystal oscillator to stabilize
    mInitSTDIO(); //Serial port 0, can be used for debugging, the default baud rate is 57600bps
    printf("start ...\n");

    mTimer0ModSetup(2); //Method 2, automatic reload 8 is the timer
    mTimer0ClkFsys( ); //Clock selects Fsys timer mode
    mTimer0SetData(0x2323); //Timer assign initial value
    mTimer0RunCTL( 1 ); //Start the timer
// printf("%02X %02X",(UINT16)TH0,(UINT16)TL0);
    ET0 = 1; //Enable timer counter 0 interrupt
    EA = 1; //Enable global interrupt
To
    while(1);
}
