/* Debug */
/* Provide printf subroutine and delay function */

#ifndef __DEBUG_H__
#define __DEBUG_H__

//Define the function return value
#ifndef SUCCESS
#define SUCCESS 1
#endif
#ifndef FAIL
#define FAIL 0
#endif

//Define the start of the timer
#ifndef START
#define START 1
#endif
#ifndef STOP
#define STOP 0
#endif

void CfgFsys( ); //CH559 clock selection and configuration
void mDelayuS( UINT16 n ); // Delay in uS
void mDelaymS( UINT16 n ); // Delay in mS
void CH559UART0Alter(); //CH559 serial port 0 pin is mapped to P0.2/P0.3
void mInitSTDIO( ); //T1 as a baud rate generator
UINT8 CH559UART0RcvByte( ); //CH559 UART0 receiving a byte in query mode
void CH559UART0SendByte(UINT8 SendDat); //CH559UART0 sends a byte
#endif
