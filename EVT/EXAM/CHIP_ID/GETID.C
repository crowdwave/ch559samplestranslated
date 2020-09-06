/********************************** (C) COPYRIGHT *********** ********************
* File Name :GETID.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: Get the chip's unique ID number and ID number verification function
                      ROM_CHIP_ID_ADDR starts with 4 bytes ID number, the next 2 bytes are ID and check
************************************************** *****************************/

#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"
#include <stdio.h>
#include <string.h>

#pragma NOAREGS

#define ROM_CHIP_ID_ADDR 0x20

/************************************************* ******************************
* Function Name: GetChipID(void)
* Description: Get ID number and ID number and check
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
UINT32 GetChipID( void)
{
UINT8 d0, d1;
UINT16 xl, xh;
E_DIS = 1; //Avoid entering interrupt
d0 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 0 );
d1 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 1 ); //ID number low word
xl = (d1 << 8) | d0;
d0 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 2 );
d1 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 3 ); //ID number high word
xh = (d1 << 8) | d0;
d0 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 6 );
d1 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 7 ); //ID checksum
E_DIS = 0;
if ((UINT16)( xl + xh) != (UINT16)( (d1 << 8) | d0)) return( 0xFFFFFFFF );//Check ID number
return( ((UINT32)xh << 16) | xl );
}

/************************************************* ******************************
* Function Name: CopyChipID(void)
* Description: Get the ID number, because the Flash is double-byte access, the low byte is first, pay attention when using it
* Input: PUINT32X buf
* Output: None
* Return: None
************************************************** *****************************/
void CopyChipID( PUINT32X buf)
{
E_DIS = 1;
*( (PUINT16X)buf + 0) = *(const unsigned short code *)( ROM_CHIP_ID_ADDR + 0 );
*( (PUINT16X)buf + 1) = *(const unsigned short code *)( ROM_CHIP_ID_ADDR + 2 );
E_DIS = 0;
}

void main()
{
    UINT32 x;
// CfgFsys( );
// mDelaymS(5); //Wait for the external crystal oscillator to stabilize

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf( "Start @ChipID=%02X\n", (UINT16)CHIP_ID );
    printf("ID+CRC:%lx\n",GetChipID());
    CopyChipID(&x);
    printf("ID:%lx\n",x);
    while(1);
}
