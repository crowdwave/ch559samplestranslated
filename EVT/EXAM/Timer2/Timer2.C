/********************************** (C) COPYRIGHT *********** ********************
* File Name: Timer2.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 TIME2 interface function
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#ifndef TIMER
#define TIMER 0 //T2 as a timer
#define T2EX_CAP 0 //T2ex capture pin level
#define T2_CAP 1 //T2 capture pin level
#endif

#pragma NOAREGS

//CH559 Timer2 clock selection
//bTMR_CLK affects Timer0&1&2 at the same time, please pay attention when using
#define mTimer2ClkFsys() {T2MOD |= (bTMR_CLK | bT2_CLK);C_T2=0;} //Timer, clock=Fsys
#define mTimer2Clk4DivFsys() {T2MOD &= ~bTMR_CLK;T2MOD |= bT2_CLK;C_T2 = 0;}//Timer, clock=Fsys/4
#define mTimer2Clk12DivFsys() {T2MOD &= ~(bTMR_CLK | bT2_CLK);C_T2 = 0;} //Timer, clock=Fsys/12
#define mTimer2CountClk() {C_T2 = 1;} //Counter, the falling edge of T2 pin is valid

//CH559 Timer2 start (SS=1)/end (SS=0)
#define mTimer2RunCTL( SS) {TR2 = SS? START: STOP;}

UINT8 FLAG;
UINT16 Cap[8] = {0};

/************************************************* ******************************
* Function Name: mTimer2Setup(UINT8 T2Out)
* Description: CH559 Timing 2 initialization
* Input: UINT8 T2Out, whether to allow T2 output clock
                   0: No output allowed
                   1: Allow output
* Output: None
* Return: None
************************************************** *****************************/
void mTimer2Setup(UINT8 T2Out)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 0; //Start the automatic reload timer function
    if(T2Out)
    {
T2MOD |= T2OE; //Whether to allow T2 output clock, if allowed clock = 1/2 timer 2 overflow rate
    }
    else
    {
T2MOD &= ~T2OE;
    }
}

/************************************************* ******************************
* Function Name: mTimer2Init(UINT16 Tim)
* Description: CH559 T2 timer assignment initial value
* Input: UINT16 Tim, timer initial value
* Output: None
* Return: None
************************************************** *****************************/
void mTimer2Init(UINT16 Tim)
{
    UINT16 tmp;
    tmp = 65536-Tim;
    RCAP2L = TL2 = tmp & 0xff;
    RCAP2H = TH2 = (tmp >> 8) & 0xff;
}

/************************************************* ******************************
* Function Name: T2exCaptureSetup(UINT8 mode)
* Description: CH559 timer counter 2 T2EX pin capture function initialization
                   UINT8 mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: Between any edges of T2ex
                   3: T2ex from rising edge to next rising edge
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void T2exCaptureSetup(UINT8 mode)
{
    C_T2 = 0;
    EXEN2 = 1;
    CP_RL2 = 1; //Start the capture function of T2ex
    T2MOD |= mode << 2; //Edge capture mode selection
}

/************************************************* ******************************
* Function Name: T2CaptureSetup(UINT8 mode)
* Description: CH559 timer counter 2 T2 pin capture function initialization T2
                   UINT8 mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: Between any edges of T2ex
                   3: T2ex from rising edge to next rising edge
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void T2CaptureSetup(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 1;
    C_T2 = 0;
    T2MOD &= ~T2OE; //Enable T2 pin capture function
    T2MOD |= (mode << 2) | bT2_CAP1_EN; //Edge capture mode selection
}

/************************************************* ******************************
* Function Name: mTimer2Interrupt()
* Description: CH559 timer counter 2 timer counter interrupt processing function
************************************************** *****************************/
void mTimer2Interrupt( void) interrupt INT_NO_TMR2 using 2 //timer2 interrupt service routine, use register set 1
{
    mTimer2RunCTL( 0 ); //Turn off the timer
#if T2EX_CAP
    if(EXF2) //T2ex level change interrupt interrupt flag
    {
        MOSI1 = !MOSI1; //P2.1 level indicator monitoring
        Cap[FLAG++] = RCAP2; //T2EX
        EXF2 = 0; //Clear the T2ex capture interrupt flag
    }
#endif

#if T2_CAP
    if(CAP1F) //T2 level capture interrupt flag
    {
        Cap[FLAG++] = T2CAP1; //T2;
        CAP1F = 0; //Clear the T2 capture interrupt flag
    }
#endif
To
#if TIMER
    if(TF2)
    {
        TF2 = 0; //Clear Timer 2 overflow interrupt
        UDTR = !UDTR; //P0.2 level indicator monitoring
    }
#endif
    mTimer2RunCTL( 1 ); //Start the timer
}

main()
{
    UINT8 i;
// CfgFsys( ); //CH559 clock selection configuration
    mDelaymS(5); //Wait for the external crystal oscillator to stabilize

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");

    FLAG = 0;
    PORT_CFG |= bP0_OC; //P0.2 is set to weak quasi-bidirectional IO
    P0_DIR |= bUDTR;
    P0_PU |= bUDTR;


    mTimer2ClkFsys( ); //Clock selects Fsys timer mode
#if TIMER
    mTimer2Setup(0); //Timer function demonstration
    mTimer2Init(0x2000); //Timer assign initial value
    ET2 = 1; //Enable timer counter 2 interrupt
    EA = 1; //Enable global interrupt
    mTimer2RunCTL( 1 ); //Start the timer
#endif

#if T2EX_CAP
    T2exCaptureSetup(1); //T2ex pin capture demo
    ET2 = 1; //Enable timer counter 2 interrupt
    EA = 1; //Enable global interrupt
    mTimer2RunCTL( 1 ); //Start the timer
    T2EX = 0; //Analog T2ex pin level change
    mDelayuS(500);
    T2EX = 1;
    mDelayuS(500);
    T2EX = 0;
    mDelayuS(500);
    T2EX = 1;
    mDelaymS(1); //Ensure that the data is collected the last time
    mTimer2RunCTL( 0 ); //Close the timer
#endif

#if T2_CAP
    T2CaptureSetup(1); //T2 pin capture demo
    ET2 = 1; //Enable timer counter 2 interrupt
    EA = 1; //Enable global interrupt
    mTimer2RunCTL( 1 ); //Start the timer
    T2 = 0; //Analog T2 pin level change
    mDelayuS(90);
    T2 = 1;
    mDelayuS(200);
    T2 = 0;
    mDelaymS(1); //Ensure that the data is collected the last time
    mTimer2RunCTL( 0 ); //Close the timer
#endif

#if T2EX_CAP|T2_CAP //Capture data print
    EA = 0; //Enable global interrupt
    ET2 = 0; //Enable timer counter 2 interrupt
    printf("FLAG %02X\n",(UINT16)FLAG);
    for(i=0;i<FLAG;i++)
    {
        printf("%04X ",(UINT16)Cap[i]);
    }
#endif
    while(1);
}
