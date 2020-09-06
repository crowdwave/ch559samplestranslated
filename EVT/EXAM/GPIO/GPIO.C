/********************************** (C) COPYRIGHT *********** ********************
* File Name: PWM.C
* Author: WCH
* Version: V1.3
* Date: 2016/6/24
* Description: CH559 PWM interface function
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

#define SetPWMClk(CK_SE) (PWM_CK_SE = CK_SE) //Frequency division, default clock Fsys
#define SetPWMCycle(Cycle) (PWM_CYCLE = Cycle) //Set the cycle period
#define SetPWM1Dat(dat) (PWM_DATA = dat) //Set PWM output duty cycle
#define SetPWM2Dat(dat) (PWM_DATA2 = dat)
#define PWMPINAlter() {P4_DIR |= bPWM2_ | bPWM1_;PIN_FUNC |= bPWM1_PIN_X;} //Set PWM pin mapping

/************************************************* ******************************
* Function Name: InitPWM1(UINT8 polar)
* Description: PWM1 initialization function
* Input: polar=0 selects the default low level, high level output is valid;
                   polar=1 selects the default high level, and the low level output is valid;
* Output: None
* Return: None
************************************************** *****************************/
void InitPWM1(UINT8 polar)
{
    PWM_CTRL &= ~bPWM_CLR_ALL; //Clear FIFO and count
    PWM_CTRL &= ~bPWM_MOD_MFM;
    PWM_CTRL |= bPWM_IE_END; //Enable PWM counting cycle completion interrupt
    PWM_CTRL |= bPWM_OUT_EN; //PWM1 output enable
    PWM_CTRL |= bPWM_IF_END; //Clear all interrupt flags
    if(polar){
        PWM_CTRL |= bPWM_POLAR; //active low
    }
    else{
        PWM_CTRL &= ~bPWM_POLAR; //High level active
    }
}

/************************************************* ******************************
* Function Name: InitPWM2(UINT8 polar)
* Description: PWM initialization function
* Input: polar=0 selects the default low level, high level output is valid;
                   polar=1 selects the default high level, the low level output is valid;
* Output: None
* Return: None
************************************************** *****************************/
void InitPWM2(UINT8 polar)
{
    PWM_CTRL &= ~bPWM_CLR_ALL; //Clear FIFO and count
    PWM_CTRL &= ~bPWM_MOD_MFM;
    PWM_CTRL |= bPWM_IE_END; //Enable PWM counting cycle completion interrupt
    PWM_CTRL |= bPWM2_OUT_EN; //PWM2 output enable
    PWM_CTRL |= bPWM_IF_END; //Clear all interrupt flags
    if(polar){
        PWM_CTRL |= bPWM2_POLAR; //active low
    }
    else{
        PWM_CTRL &= ~bPWM2_POLAR; //High level active
    }
}

/************************************************* ******************************
* Function Name: PWMInterrupt(void)
* Description: PWM interrupt service routine
************************************************** *****************************/
void PWMInterrupt( void) interrupt INT_NO_PWM1 using 1 //PWM1&2 interrupt service routine, use register set 1
{
    if(PWM_CTRL & bPWM_IF_END)
    {
        PWM_CTRL |= bPWM_IF_END;
        printf("PWM_DATA %02X\n",(UINT16)PWM_DATA);
    }
}

main()
{
    mDelaymS(30); //Power-on delay, wait for the internal crystal oscillator to stabilize, must be added
// CfgFsys( );
    PORT_CFG &= ~bP2_OC;
    P2_DIR |= bPWM1 | bPWM2; //It is recommended to set the pin to push-pull output when turning on PWM

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
#if 0
    PWMPINAlter( ); //Pin mapping
#endif
    SetPWMClk(12); //Set the clock division factor of PWM1&2 to 12
    InitPWM1(1); //PWM1 initialization, active low
    InitPWM2(0); //PWM2 initialization, active high
    SetPWMCycle(100); //Set the cycle period to 100
    IE_PWM1 = 1; //Enable PWM1 interrupt
    SetPWM1Dat(50); //PWM1 duty cycle setting 50/100
    SetPWM2Dat(50); //PWM1 duty cycle setting 50/100
    EA = 1; //Interrupt the main switch
    while(1);
}

