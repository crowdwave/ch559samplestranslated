/********************************** (C) COPYRIGHT *********** ********************
* File Name: UART1.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: Provide UART1 configuration, transceiver function definition
                       Slightly slower, but easy to use and understand
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"

#pragma NOAREGS

#define CH559UART1_BPS 115200 /*Define CH559 serial port 1 communication baud rate*/

UINT8 Num;
UINT8 buffer[20];

/************************************************* ******************************
* Function Name: UART1RegCfgValue()
* Description: CH559UART1 readable register value after correct configuration
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void UART1RegCfgValue()
{
    printf("SER1_IER %02X\n",(UINT16)SER1_IER); //0x27/0x17/0x37 are all possible
    printf("SER1_IIR %02X\n",(UINT16)SER1_IIR); //0xc1/0xC2 no interrupt or empty interrupt, "C" means FIFO is on
    printf("SER1_LCR %02X\n",(UINT16)SER1_LCR); //0x03 data format configuration, indicating wireless path interval, no parity, 1 stop bit, 8 data bits
    printf("SER1_MCR %02X\n",(UINT16)SER1_MCR); //0x08 interrupt output enable, excluding other functions such as flow control on
    printf("SER1_LSR %02X\n",(UINT16)SER1_LSR); //0x60, FIFO and line status
    printf("SER1_MSR %02X\n",(UINT16)SER1_MSR);
}

/************************************************* ******************************
* Function Name: ResetUART1()
* Description: CH559UART1 port soft reset
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
void ResetUART1()
{
    SER1_IER |= bIER_RESET; //This bit can be automatically cleared to reset the serial port register
}

/************************************************* ******************************
* Function Name: CH559UART1Init(UINT8 DIV,UINT8 mode,UINT8 pin)
* Description: CH559 UART1 initialization setting
* Input:
                   UINT8 DIV sets the frequency division coefficient, clock frequency=Fsys/DIV, DIV cannot be 0
                   UINT8 mode, mode selection, 1: normal serial port mode; 0: 485 mode
                   UINT8 pin, serial port pin selection;
                   When mode=1
                   0: RXD1=P4.0, TXD1 is off;
                   1: RXD1&TXD1=P4.0&P4.4;
                   2: RXD1&TXD1=P2.6&P2.7;
                   3: RXD1&TXD1&TNOW=P2.6&P2.7&P2.5;
                   When mode=0
                   0: meaningless
                   1: Connect P5.4&P5.5 to 485, TNOW=P4.4;
                   2: P5.4&P5.5 connect to 485;
                   3: P5.4&P5.5 connect to 485, TNOW=P2.5;
* Output: None
* Return: None
************************************************** *****************************/
void CH559UART1Init(UINT8 DIV,UINT8 mode,UINT8 pin)
{
    UINT32 x;
    UINT8 x2;

    SER1_LCR |= bLCR_DLAB; // DLAB bit is 1, write DLL, DLM and DIV registers
    SER1_DIV = DIV; // Prescaler
    x = 10 * FREQ_SYS *2 / DIV / 16 / CH559UART1_BPS;
    x2 = x% 10;
    x /= 10;
    if (x2 >= 5) x ++; //Rounding
    SER1_DLM = x>>8;
    SER1_DLL = x&0xff;
    SER1_LCR &= ~bLCR_DLAB; //DLAB bit is 0 to prevent modification of UART1 baud rate and clock
    if(mode == 1) //Close RS485 mode RS485_EN = 0, can not be omitted
    {
XBUS_AUX |= bALE_CLK_EN;
    }
    else if(mode == 0) //Enable RS485 mode RS485_EN = 1;
    {
        UHUB1_CTRL |= bUH1_DISABLE;
        PIN_FUNC &= ~bXBUS_CS_OE;
        PIN_FUNC |= bXBUS_AL_OE;
        XBUS_AUX &= ~bALE_CLK_EN;
        SER1_MCR |= bMCR_HALF; //485 mode can only use half-duplex mode
    }
    SER1_LCR |= MASK_U1_WORD_SZ; //Line control
    SER1_LCR &= ~(bLCR_PAR_EN | bLCR_STOP_BIT); //Wireless path interval, no parity, 1 stop bit, 8 data bits

    SER1_IER |= ((pin << 4) & MASK_U1_PIN_MOD); //Serial port mode configuration
    SER1_IER |= bIER_MODEM_CHG | bIER_LINE_STAT | bIER_THR_EMPTY | bIER_RECV_RDY;//Interrupt enable configuration

    SER1_FCR |= MASK_U1_FIFO_TRIG | bFCR_T_FIFO_CLR | bFCR_R_FIFO_CLR | bFCR_FIFO_EN;//FIFO controller
                                                                               //Empty the receive and transmit FIFO, 7-byte receive trigger, FIFO enable
    SER1_MCR |= bMCR_OUT2; //MODEM control register
                                                                               //Interrupt request output, no actual interrupt
    SER1_ADDR |= 0xff; //Close multi-machine communication
}

/************************************************* ******************************
* Function Name: CH559UART1RcvByte()
* Description: CH559UART1 receives one byte
* Input: None
* Output: None
* Return: correct: UINT8 Rcvdat; receive data
************************************************** *****************************/
UINT8 CH559UART1RcvByte()
{
    while((SER1_LSR & bLSR_DATA_RDY) == 0); //Wait for the data to be ready
    return SER1_RBR;
}


/************************************************* ******************************
* Function Name: CH559UART1Rcv(PUINT8 buf)
* Description: CH559UART1 receives multiple bytes
* Input: PUINT8 buf
* Output: UINT8 RcvNum//Returns the number of received data
* Return: correct:
                   Error: None
************************************************** *****************************/
UINT8 CH559UART1Rcv(PUINT8 buf)
{
    UINT8 RcvNum;
    RcvNum = 0;
    while((SER1_LSR & bLSR_DATA_RDY) == 0); //Wait for the data to be ready
    while(SER1_LSR & bLSR_DATA_RDY)
    {
        buf[RcvNum] = SER1_RBR;
        RcvNum++;
    }
    return RcvNum; //Return to receive count
}

/************************************************* ******************************
* Function Name: CH559UART1SendByte(UINT8 SendDat)
* Description: CH559UART1 sends one byte
* Input: UINT8 SendDat; the data to be sent
* Output: None
* Return: None
************************************************** *****************************/
void CH559UART1SendByte(UINT8 SendDat)
{
    SER1_THR = SendDat;
    while((SER1_LSR & bLSR_T_FIFO_EMP) == 0); //Wait for the data to be sent
}

/************************************************* ******************************
* Function Name: CH559UART1SendStr(PUINT8 SendStr)
* Description: CH559UART1 sends multiple bytes
* Input: UINT8 SendStr; the first address of the data to be sent
* Output: None
* Return: None
************************************************** *****************************/
void CH559UART1SendStr(PUINT8 SendStr)
{
    while( *SendStr !='\0')
    {
        CH559UART1SendByte( *SendStr++ );
    }
}

/************************************************* ******************************
* Function Name: UART1Interrupt(void)
* Description: UART1 interrupt service routine
************************************************** *****************************/
void UART1Interrupt( void) interrupt INT_NO_UART1 using 1 //UART1 interrupt service routine, use register set 1
{
    UINT8 InterruptStatus,i,tmp;
    InterruptStatus = SER1_IIR & 0x0f; //Get interrupt status
// printf("InterruptStatus %02X\n",(UINT16)InterruptStatus);
    switch(InterruptStatus)
    {
        case U1_INT_RECV_RDY: //Receive data can be interrupted, you can read the specified number of bytes to trigger the interrupt data number
           Num = CH559UART1Rcv(buffer);
           tmp = Num;
           while(Num--)
           {
               printf("%02X ",(UINT16)buffer[Num]);
           }
// for(i = 0;i <tmp;i++)
// {
// CH559UART1SendByte(buffer[i]);
//}
break;
        case U1_INT_RECV_TOUT: //Receive timeout interrupt
             Num = CH559UART1Rcv(buffer);
           tmp = Num;
             while(Num--)
             {
                 printf("%02X ",(UINT16)buffer[Num]);
             }
// for(i = 0;i <tmp;i++)
// {
// CH559UART1SendByte(buffer[i]);
//}
break;
        case U1_INT_LINE_STAT: //Line status interrupt
break;
        case U1_INT_SLV_ADDR: //Device address match interrupt
break;
        case U1_INT_NO_INTER: //No interrupt
          break;
        case U1_INT_MODEM_CHG: //MODEM interrupt
i = SER1_MSR;
         break;
        case U1_INT_THR_EMPTY: //Send empty interrupt, you can start the next transmission or wait for reception
break;
        default:
         break;
}
}

main()
{
    UINT8 i;
// CfgFsys( ); //Clock configuration
// mDelaymS(5); //Wait for the external crystal oscillator to stabilize

    mInitSTDIO( ); //Serial port 0, can be used for debugging
    printf("start ...\n");
To
    CH559UART1Init(1,1,2);
// UART1RegCfgValue( ); //UART1 register configuration
    P4_DIR |= 0x10; //When using P4 port, you must set the direction, TXD1 is set as output

/*Interrupt mode*/
    IE_UART1 = 1; //UART1 interrupt enable
    EA = 1; //Enable global interrupt
    Num = 0;
    while(1);

/*inquiry mode*/
// while(1)
// {
// Num = CH559UART1Rcv(buffer);
// for(i = 0;i <Num;i++)
// {
// CH559UART1SendByte(buffer[i]);
//}
//}
}
