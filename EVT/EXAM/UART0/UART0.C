/********************************** (C) COPYRIGHT *********** ********************
* File Name: UART0.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: CH559 serial port 0 spontaneous sending and receiving demo
                     (1) The serial port 0 sends and receives data, the baud rate is adjustable;
************************************************** *****************************/

#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

 UINT8 DAT,FLAG;

/************************************************* ******************************
* Function Name: CH559UART0Interrupt()
* Description: CH559UART0 interrupt processing function
************************************************** *****************************/
void CH559UART0Interrupt() interrupt INT_NO_UART0 using 1 //Watchdog interrupt service program, use register set 1
{
    if(TI)
    {
        TI = 0; //Clear the transmit interrupt
    }
    if(RI)
    {
        FLAG = 1;
        RI = 0; //clear the receive interrupt
        DAT = SBUF;
    }
}

main()
{
    UINT8 i;
// CfgFsys( ); //CH559 clock selection configuration
    mDelaymS(5); //Wait for the external crystal oscillator to stabilize
// CH559UART0Alter();
    FLAG = 0; //Flag bit is cleared
    mInitSTDIO( ); //Serial port 0 initialization function
    ES = 1; //Enable UART0 interrupt
    EA = 1; //Total interrupt is on
    while(1)
{
if(FLAG == 1)
{
SBUF = DAT;
FLAG = 0;
}
}
}

