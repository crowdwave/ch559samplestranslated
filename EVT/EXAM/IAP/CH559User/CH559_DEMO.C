/********************************** (C) COPYRIGHT *********** *******************
* File Name: CH559_DEMO.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: IAP download main program
                       Demonstrate the user program running, when P13 inputs low level, the program jumps to the IAP program area to upgrade the user program
************************************************** *****************************/

#include "../../CH559.H"
#include <string.h>
#include <intrins.h>

#define IAP_ProgrameStartAddr (0xE800) //The starting address where the IAP program is stored, which is at least 4 bytes smaller than the actual IAP address
sbit EnableIAP = P1^3; //IAP jump detection pin

typedef void( *pTaskFn)( void );
pTaskFn tasksArr[1];

#pragma NOAREGS

/************************************************* ******************************
* Function Name: mDelay20us(UNIT16 n)
* Description: 20us delay function, the main frequency is 12MHz, the delay is not accurate, other main frequencies refer to the delay function of DEBUG.C
* Input: UNIT16 n
* Output: None
* Return: None
************************************************** *****************************/
void mDelay20us( UINT16 n)
{
for( n <<= 3;n;--n ){
_nop_( );
}
}

/************************************************* ******************************
* Function Name: main
* Description: main function
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void main( void)
{
UINT16 i=0;

P4_DIR = 0x00;
P4_DIR |= 0x0f; //P40-P43 output enable
P4_OUT = 0x0A; //P41 P43 output high level
tasksArr[0] = (pTaskFn)(IAP_ProgrameStartAddr+0x00); //IAP program address
while(1)
  {
if( EnableIAP == 0) //Check whether the P13 pin is low
    {
P4_OUT = 0x0f;
P4_OUT = ~(1<<1); //The LED connected to P4 is used for status indication
mDelay20us(60000);
P4_OUT = 0x0f;
(tasksArr[0])( ); //Jump to IAP program area
}
i++;
if( i == 200) i = 0; //User program can do other things, here is just flashing light
if( i == 0) P4_OUT = 0x0f;
if( i == 100) P4_OUT = ~(1<<2);
mDelay20us(1000);
}
}
