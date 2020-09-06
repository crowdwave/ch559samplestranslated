/********************************** (C) COPYRIGHT *********** ********************
* File Name: Timer3.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 TIME3 interface function
************************************************** *****************************/
#include ".\DEBUG.C" //Printing debugging information
#include ".\DEBUG.H"

#pragma NOAREGS

UINT8V FLAG;
UINT16 Cap[8];

#define TIMER3 0
#define T3PWM3 0
#define T3CAP3 1

#define mTimer3Stop() {T3_CTRL &= ~bT3_CNT_EN;} //Turn off timer 3
#define mTimer3Start() {T3_CTRL |= bT3_CNT_EN;} //Start timer/counter 3
#define mTimer3Init( dat) {T3_END_L = dat & 0xff;T3_END_H = (dat >> 8) & 0xff;} //T3 timer assign initial value

/************************************************* ******************************
* Function Name: mSetTimer3Clk(UINT16 DIV)
* Description: CH559 Timer3 clock setting
* Input: UINT16 division factor
* Output: None
* Return: None
************************************************** *****************************/
void mSetTimer3Clk(UINT16 DIV)
{
    T3_SETUP |= bT3_EN_CK_SE;
    T3_CK_SE_L = DIV & 0xff;
    T3_CK_SE_H = (DIV >> 8) & 0xff;
    T3_SETUP &= ~bT3_EN_CK_SE;
}

/************************************************* ******************************
* Function Name: void mTimer3Setup()
* Description: CH559 timer counter 3 initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void mTimer3Setup()
{
    T3_CTRL |= bT3_CLR_ALL; //Empty FIFO and count
    T3_CTRL &= ~bT3_CLR_ALL;
    T3_SETUP |= bT3_IE_END; //Enable timer/counter 3FIFO overflow interrupt
    T3_CTRL &= ~bT3_MOD_CAP; //Set to work in timer mode
    T3_STAT |= bT3_IF_DMA_END | bT3_IF_FIFO_OV | bT3_IF_FIFO_REQ | bT3_IF_ACT | bT3_IF_END;//Clear all interrupts in the register
}

/************************************************* ******************************
* Function Name: mTimer3PWMSetup()
* Description: CH559 timer counter PWM3 initialization
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void mTimer3PWMSetup()
{
    PORT_CFG &= ~bP1_DRV;
    P1_DIR |= bPWM3; //Set the PWM3 pin to push-pull mode
    T3_CTRL |= bT3_CLR_ALL; //Empty FIFO and count
    T3_CTRL &= ~bT3_CLR_ALL;
    T3_SETUP |= bT3_IE_END | bT3_IE_ACT;
    T3_CTRL |= bT3_OUT_EN;
}

/************************************************* ******************************
* Function Name: mTimer3PWMPolarSel(UINT8 mode)
* Description:
* Input: UINT8 mode, PWM effective level output selection
* Output: None
* Return: None
************************************************** *****************************/
void mTimer3PWMPolarSel(UINT8 mode)
{
    if(mode)
    {
        T3_CTRL |= bT3_PWM_POLAR; //Set the PWM output polarity, low level
    }
    else
    {
        T3_CTRL &= ~bT3_PWM_POLAR; //Set PWM output polarity, high level
    }
}
/************************************************* ******************************
* Function Name: mTimer3PWMDatInit(UINT16 dat0,UINT16 dat1)
* Description:
* Input: UINT16 dat0, PWM cycle time
                   UINT16 dat1, PWM effective level output time
* Output: None
* Return: None
************************************************** *****************************/
void mTimer3PWMDatInit(UINT16 dat0,UINT16 dat1)
{
    T3_END_L = dat0 & 0xff; //Set the duty cycle
    T3_END_H = (dat0 >> 8) & 0xff;
    T3_FIFO_L = dat1 & 0xff;
    T3_FIFO_H = (dat1 >> 8) & 0xff;
}

/************************************************* ******************************
* Function Name: mTimer3CaptureSetup(UINT16 dat0,UINT8 mode)
* Description: CH559 timer counter 3 capture function initialization CAP3
* Input: UINT16 dat0, capture timeout time setting
UINT8 mode, capture mode selection
0: Close capture
1: Capture from any edge to any edge
2: Capture from falling edge to falling edge
3: Capture from rising edge to rising edge
* Output: None
* Return: None
************************************************** *****************************/
void mTimer3CaptureSetup(UINT16 dat0,UINT8 mode)
{
    T3_CTRL = 0;
    T3_CTRL |= bT3_CLR_ALL; //Clear T3 related registers
    T3_CTRL &= ~bT3_CLR_ALL;

    T3_SETUP |= bT3_IE_ACT;
    T3_CTRL |= bT3_MOD_CAP | bT3_CAP_WIDTH;
    T3_CTRL |= mode << 6;
    T3_END_L = dat0 & 0xff; //Maximum capture timeout, timeout time = Pclk*max_time, Pclk is the system clock
    T3_END_H = (dat0 >> 8) & 0xff;
    T3_FIFO_L = 0; //Clear
    T3_FIFO_H = 0;
    T3_STAT = 0xF0; //Clear interrupt
}

/************************************************* ******************************
* Function Name: mTimer3Interrupt()
* Description: CH559 timer counter 3 timer counter interrupt processing function
************************************************** *****************************/
void mTimer3Interrupt( void) interrupt INT_NO_TMR3 using 2 //timer3 interrupt service routine, use register set 1
{
  mTimer3Stop( );
#if TIMER3
    if(T3_STAT & bT3_IF_END) //Count interrupt
    {
        T3_STAT |= bT3_IF_END;
    }
#endif

#if T3CAP3
    if(T3_STAT & bT3_IF_ACT) //Capture interrupt
    {
        printf("Cap:%04X ",(UINT16)T3_FIFO);
// Cap[FLAG++] = T3_FIFO;
        T3_STAT |= bT3_IF_ACT;
    }
#endif

// if(T3_STAT & bT3_IF_DMA_END)
// {
// T3_STAT |= bT3_IF_DMA_END;
//}
mTimer3Start();
}

main()
{
    UINT8 i,tmp;
// CfgFsys( );
    mDelaymS(5); //Wait for external crystal oscillator to stabilize

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
To
    mSetTimer3Clk(0x02); //T3 clock configuration
/* Counting timer */
#if TIMER3
    mTimer3Setup( ); //Basic timer settings
    mTimer3Init( 0x2000 ); //T3 timer assign initial value
    mTimer3Start( ); //Start timer/counter 3
    IE_TMR3 = 1;
    EA = 1;
    while(1);
#endif
To
/* PWM3 demo */
#if T3PWM3
    PORT_CFG &= ~bP1_OC; //P1.2 sets the PWM port to push-pull output mode
    P1_DIR |= bPWM3;
    P1_PU |= bPWM3;
    mTimer3PWMSetup( ); //PWM3 basic configuration
    mTimer3PWMPolarSel(1); //PWM3 output high and low polarity selection
    mTimer3PWMDatInit(12,3); //PWM output duty cycle setting
    mTimer3Start( );
    IE_TMR3 = 1;
    EA = 1;
    while(1);
#endif

 /* CAP3 demo */
#if T3CAP3
    PIN_FUNC |= bTMR3_PIN_X;
    PORT_CFG |= bP1_OC; //CAP3 port is quasi-bidirectional mode
    P1_DIR |= bCAP3;
    P1_PU |= bCAP3;
To
    FLAG = 0;
    mTimer3CaptureSetup(0x4000,1); //CAP3 capture function
    mTimer3Start( ); //Start timer/counter 3
    EA = 1;
    IE_TMR3 = 1;
while(1);
To
    for(i=0; i<6; i++)
    {
        mDelayuS(300);
        CAP3 = !CAP3; //Analog CAP3 pin level change
    }
To
    mTimer3Stop();
    IE_TMR3 = 0;
    EA = 1;
To
    printf("FLAG:%02X \n",(UINT16)FLAG);
    for(i = 0;i <FLAG;i++)
    {
        printf("Cap[%02X]:%04X ",(UINT16)i,(UINT16)Cap[i]);
    }
    printf("\n");
    tmp = T3_STAT&MASK_T3_FIFO_CNT;
    for(i = 0;i <tmp;i++)
    {
        printf("%04X ",T3_FIFO);
    }
    printf("T3_STAT:%02X \n",(UINT16)T3_STAT);
#endif
    while(1);
}

/*
1. The current TIME3 timing/counting function is to count the time from 0 to T3_END, and then enter the interrupt;
2. In PWM mode, duty cycle=T3_FIFO/T3_END
3. Capture mode, the current value in FIFO is the edge time to be captured;
*/
