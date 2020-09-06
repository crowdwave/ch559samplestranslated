/********************************** (C) COPYRIGHT *********** ********************
* File Name: DataFlash.C
* Author: WCH
* Version: V1.3
* Date: 2016/6/24
* Description: Read and write DataFlash of CH559, the internal Flash of CH55X is double-byte write (big endian), single-byte read
************************************************** *****************************/
#include "..\DEBUG.C" //Print debugging information
#include "..\DEBUG.H"
#include <intrins.h>

#pragma NOAREGS

/************************************************* ******************************
* Function Name: EraseBlock(UINT16 Addr)
* Description: CodeFlash block erase (1KB), write 1 for all data bits
* Input: UINT16 Addr
* Output: None
* Return: None
************************************************** *****************************/
UINT8 EraseBlock( UINT16 Addr)
{
ROM_ADDR = Addr;
if (ROM_STATUS & bROM_ADDR_OK) {// The operation address is valid
ROM_CTRL = ROM_CMD_ERASE;
return( (ROM_STATUS ^ bROM_ADDR_OK) & 0x7F ); // return status, 0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
}
else return( 0x40 );
}

/************************************************* ******************************
* Function Name: ProgWord( UINT16 Addr, UINT16 Data)
* Description: write EEPROM, double-byte write
* Input: UNIT16 Addr, write address
                   UINT16 Data, data
* Output: None
* Return: SUCESS
************************************************** *****************************/
UINT8 ProgWord( UINT16 Addr, UINT16 Data)
{
ROM_ADDR = Addr;
ROM_DATA = Data;
if (ROM_STATUS & bROM_ADDR_OK) {// The operation address is valid
ROM_CTRL = ROM_CMD_PROG;
return( (ROM_STATUS ^ bROM_ADDR_OK) & 0x7F ); // return status, 0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
}
else return( 0x40 );
}

/************************************************* ******************************
* Function Name: EraseDataFlash(UINT16 Addr)
* Description: DataFlash block erase (1KB), write 1 for all data bits
* Input: UINT16 Addr
* Output: None
* Return: UINT8 status
************************************************** *****************************/
UINT8 EraseDataFlash(UINT16 Addr)
{
    UINT8 status;

    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA; //Enter safe mode
    GLOBAL_CFG |= bDATA_WE; //Enable DataFlash writing
    SAFE_MOD = 0;
    status = EraseBlock(Addr);
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA; //Enter safe mode
    GLOBAL_CFG &= ~bDATA_WE; //Enable DataFlash write protection
    SAFE_MOD = 0;
    return status;
}

/************************************************* ******************************
* Function Name: WriteDataFlash(UINT16 Addr,PUINT8 buf,UINT16 len)
* Description: Write in DataFlash
* Input: UINT16 Addr, PUINT16 buf, UINT16 len
* Output: None
* Return:
************************************************** *****************************/
void WriteDataFlash(UINT16 Addr,PUINT8 buf,UINT16 len)
{
    UINT16 j,tmp;

    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA; //Enter safe mode
    GLOBAL_CFG |= bDATA_WE; //Enable DataFlash writing
    SAFE_MOD = 0;
    for(j=0;j<len;j=j+2)
    {
        tmp = buf[j+1];
        tmp <<= 8;
        tmp += buf[j];
        ProgWord(Addr,tmp);
        Addr = Addr + 2;
    }
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA; //Enter safe mode
    GLOBAL_CFG &= ~bDATA_WE; //Enable DataFlash write protection
    SAFE_MOD = 0;
}

void main()
{
    UINT8X buf[512];
    UINT8 ret;
    UINT16 j,i;
    i = 0;

// CfgFsys( ); //CH559 clock selection configuration
// mDelaymS(5); //Wait for the external crystal oscillator to stabilize
    mInitSTDIO( ); //Serial port 0, can be used for debugging
// CH559UART0Alter(); //Serial port 0 is mapped to P02 P03, the default is P30 P31
while(1){

    ret = EraseDataFlash(0xF000);
    if(ret == 0){
        printf("Erase DataFlash SUCCESS...\n");
    }
    else{
        printf("ERR %02X\n",(UINT16)ret);
    }
To
    printf("Write DataFlash ...\n"); //Write DataFlash
    for(j=0;j<512;j++){
        buf[j] = j%512;
    }
    WriteDataFlash(0xF000,buf,512);

    for(j=0;j<512;j++){
        printf("%02X ",(UINT16)*((PUINT8C)(0xF000+j))); //Read DataFlash
    }
    printf("\nRead DataFlash SUCCESS...\n");
    mDelaymS(100);
    while(1);
 }
}
