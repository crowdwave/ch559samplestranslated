/********************************** (C) COPYRIGHT *********** ********************
* File Name: Encryption.C
* Author: WCH
* Version: V1.3
* Date: 2016/06/24
* Description: Use numerical substitution or interpolation to flexibly realize MCU HEX encryption
                       For example, use Encoded_0() and Encoded_1() to replace 0 and 1, respectively, and other similar methods can be used to encrypt the key
************************************************** *****************************/

#include "./DEBUG.C"
#include "./DEBUG.H"
#define PI 3.141592657

#pragma NOAREGS

UINT8 ID;
UINT8 IDX;

/************************************************* ******************************
* Function Name: EraseBlock(UNIT16 Addr)
* Description: Dataflash block erase function
* Input: UINT16 Addr
* Output: None
* Return: status
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
* Description: Dataflash write function
* Input: UNIT16 Addr,UINT16 Data
* Output: None
* Return: status
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
* Function Name: EncodedID_AndWR_ToDataflash()
* Description: ID conversion function, use irreversible calculation (here you can choose other more complex algorithms) to operate ID,
                   Obtain an integer IDX and store it in Dataflash
* Input: None
* Output: None
* Return: None
************************************************** *****************************/
UINT8 EncodedID_AndWR_ToDataflash()
{
double i;
UINT8 status;
i=(double)ID/PI;
IDX=(UINT8)i;
To
SAFE_MOD = 0x55;
SAFE_MOD = 0xAA;/*Enter safe mode*/
GLOBAL_CFG |= bDATA_WE;/*Dataflash write enable*/
status=EraseBlock(0xF000);/*Erase 1K Dataflash*/
SAFE_MOD = 0x55;
SAFE_MOD = 0xAA;
  GLOBAL_CFG &= ~ bDATA_WE;/*Dataflash write enable close*/
SAFE_MOD = 0xFF;/*Exit safe mode*/
To
SAFE_MOD = 0x55;
SAFE_MOD = 0xAA;/*Enter safe mode*/
GLOBAL_CFG |= bDATA_WE;/*Dataflash write enable*/
status=ProgWord( 0xF000,(UINT16)IDX);/*Write the decoding base to Dataflash*/
SAFE_MOD = 0x55;
SAFE_MOD = 0xAA;
GLOBAL_CFG &= ~ bDATA_WE;/*Dataflash write enable close*/
SAFE_MOD = 0xFF;/*Exit safe mode*/
To
return IDX;
}

/************************************************* ******************************
* Function Name: GetIDXFromDataflash()
* Description: IDX acquisition function
* Input: None
* Output: None
* Return: IDX
************************************************** *****************************/
UINT8 GetIDXFromDataflash()
{
return (UINT16)*((PUINT8C)(0xF000));
}

/************************************************* ******************************
* Function Name: Encoded_0()
* Description: Value 0 alternative function
* Input: None
* Output: None
* Return: 0
************************************************** *****************************/
UINT8 Encoded_0()
{
return (GetIDXFromDataflash()-EncodedID_AndWR_ToDataflash());
}

/************************************************* ******************************
* Function Name: Encoded_1()
* Description: Value 1 alternative function
* Input: None
* Output: None
* Return: 1
************************************************** *****************************/
UINT8 Encoded_1()
{
return (GetIDXFromDataflash()-EncodedID_AndWR_ToDataflash()+1);
}

void main()
{
UINT8 i;
mDelaymS(50);
mInitSTDIO( );
printf("start...\n");
  ID=CHIP_ID;/*Get chip ID*/

/*For debugging*/
// i=Encoded_0();
// printf("Replacement value of 0%02X\n",(UINT16)i);
// i=Encoded_1();
// printf("Replacement value of 1 %02X\n",(UINT16)i);
To
//Functional part, for loop using numerical value substitution
for(i= Encoded_0();i<10*Encoded_1();i=i+Encoded_1())
{
printf("%02X\n",(UINT16)i);
}
To
while(1);
}
