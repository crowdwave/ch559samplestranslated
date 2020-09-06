/********************************** (C) COPYRIGHT *********** ********************
* File Name: WDOG.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 WatchDOG DEMO
                     (1) Provide watchdog initialization interface function
                     (2), the calculation formula of the watch timing period 262144*(0x100-WDOG_COUNT)/Fsys
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

/************************************************* ******************************
* Function Name: WDOGInit(UINT8 n,UINT8 value)
* Description: Watchdog interrupt initialization
* Input: UINT8 n, timing duration
                   UINT8 value, select the operation after the watchdog timer is completed
                   value=1 chip reset;
                   value=0 generates a watchdog interrupt;
* Output: None
* Return: None
************************************************** *****************************/
void WDOGInit(UINT8 n,UINT8 value)
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA; //Enable safe mode
    GLOBAL_CFG |= value;
    SAFE_MOD = 0xFF; //Close the safe mode

    WDOG_COUNT = n; //Watchdog timeout
    if(!value)
    {
        IE_WDOG = 1; //Enable watchdog interrupt
        EA = 1; //Open total interrupt
    }
}

void CheckResetValue()
{
    printf("PCON %02X\n",(UINT16)PCON);
    printf("PLL_CFG %02X\n",(UINT16)PLL_CFG);
    printf("CLOCK_CFG %02X\n",(UINT16)CLOCK_CFG);
    printf("SLEEP_CTRL %02X\n",(UINT16)SLEEP_CTRL);
    printf("WAKE_CTRL %02X\n",(UINT16)WAKE_CTRL);
    printf("RESET_KEEP %02X\n",(UINT16)RESET_KEEP);
    printf("WDOG_COUNT %02X\n",(UINT16)WDOG_COUNT);
    printf("PLL_CFG %02X\n",(UINT16)PLL_CFG);
    printf("CLOCK_CFG %02X\n",(UINT16)CLOCK_CFG);
}

/************************************************* ******************************
* Function Name: WDogInterrupt(void)
* Description: Watchdog interrupt service routine
************************************************** *****************************/
void WDogInterrupt( void) interrupt INT_NO_WDOG using 1 //Watchdog interrupt service routine, use register set 1
{
    printf("PCON %02X\n",(UINT16)PCON);
    WDOG_COUNT = 0x20;
    P4_OUT = ~P4_OUT;
}

main()
{
// CfgFsys( );
    mDelaymS(5); //Wait for the external crystal oscillator to stabilize
    P4_DIR |= 0x0f; //Enable P40-P44 output
    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
To
    CheckResetValue( );
    WDOGInit(0x23,0);
    while(1);
}
