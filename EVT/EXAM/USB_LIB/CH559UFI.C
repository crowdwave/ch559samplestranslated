/* 2014.09.09
*****************************************
** Copyright (C) W.ch 1999-2015 **
** Web: http://wch.cn **
*****************************************
** USB-flash File Interface for CH559 **
** KC7.0@MCS51 **
*****************************************
*/
/* CH559 U disk host file system interface, support: FAT12/FAT16/FAT32 */

//#define DISK_BASE_BUF_LEN 512 /* The default disk data buffer size is 512 bytes (you can choose 2048 or even 4096 to support some large-sector U disks). If it is 0, it is forbidden to define the buffer in this file and Specified by the application in pDISK_BASE_BUF*/
/* If you need to reuse the disk data buffer to save RAM, you can define DISK_BASE_BUF_LEN as 0 to prohibit the definition of the buffer in this file, and the start address of the buffer that will be shared with other programs before calling CH559LibInit Put the pDISK_BASE_BUF variable*/

//#define NO_DEFAULT_ACCESS_SECTOR 1 /* The default disk sector read and write subroutine is forbidden, and replace it with a self-written program below */
//#define NO_DEFAULT_DISK_CONNECT 1 /* The default check disk connection subroutine is forbidden, replace it with a self-written program below */
//#define NO_DEFAULT_FILE_ENUMER 1 /* The default file name enumeration callback program is forbidden, and replace it with a self-written program below */

//#include "CH559.H"
#include "CH559UFI.H"

CMD_PARAM_I mCmdParam; /* Command parameter */
#if DISK_BASE_BUF_LEN> 0
UINT8X DISK_BASE_BUF[ DISK_BASE_BUF_LEN] _at_ 0x0080; /* External RAM disk data buffer, the buffer length is the length of one sector */
#endif

/* The following program can be modified as needed */

#ifndef NO_DEFAULT_ACCESS_SECTOR /* Defining NO_DEFAULT_ACCESS_SECTOR in the application program can prohibit the default disk sector reading and writing subroutine, and then replace it with a self-written program */
//if (use_external_interface) {// Replace the read and write subroutines at the bottom of the U disk sector
// CH559vSectorSizeH=512>>8; // Set the actual sector size, which must be a multiple of 512. This value is the high byte of the disk sector size, the low byte is always 0x00, and the value 2 replaces 0x200
// CH559vSectorSizeB=9; // Set the actual sector size displacement number, 512 corresponds to 9, 1024 corresponds to 10, 2048 corresponds to 11
// CH559DiskStatus=DISK_MOUNTED; // Force the block device to connect successfully (only to analyze the file system)
//}

UINT8 CH559ReadSector( UINT8 SectCount, PUINT8X DataBuf) /* Read data of multiple sectors from the disk into the buffer */
{
UINT8 retry;
// if (use_external_interface) return( extReadSector( CH559vLbaCurrent, SectCount, DataBuf) ); /* External interface */
for( retry = 0; retry <3; retry ++) {/* Retry on error */
pCBW -> mCBW_DataLen0 = 0; /* Data transmission length */
pCBW -> mCBW_DataLen1 = SectCount << CH559vSectorSizeB-8;
pCBW -> mCBW_DataLen2 = 0;
pCBW -> mCBW_DataLen3 = 0;
pCBW -> mCBW_Flag = 0x80;
pCBW -> mCBW_LUN = CH559vCurrentLun;
pCBW -> mCBW_CB_Len = 10;
pCBW -> mCBW_CB_Buf[ 0] = SPC_CMD_READ10;
pCBW -> mCBW_CB_Buf[ 1] = 0x00;
pCBW -> mCBW_CB_Buf[ 2] = *(PUINT8)&CH559vLbaCurrent;
pCBW -> mCBW_CB_Buf[ 3] = *( (PUINT8)&CH559vLbaCurrent + 1 );
pCBW -> mCBW_CB_Buf[ 4] = *( (PUINT8)&CH559vLbaCurrent + 2 );
pCBW -> mCBW_CB_Buf[ 5] = *( (PUINT8)&CH559vLbaCurrent + 3 );
pCBW -> mCBW_CB_Buf[ 6] = 0x00;
pCBW -> mCBW_CB_Buf[ 7] = 0x00;
pCBW -> mCBW_CB_Buf[ 8] = SectCount;
pCBW -> mCBW_CB_Buf[ 9] = 0x00;
CH559BulkOnlyCmd( DataBuf ); /* Execute commands based on BulkOnly protocol */
if (CH559IntStatus == ERR_SUCCESS) {
return( ERR_SUCCESS );
}
CH559IntStatus = CH559AnalyzeError( retry );
if (CH559IntStatus != ERR_SUCCESS) {
return( CH559IntStatus );
}
}
return( CH559IntStatus = ERR_USB_DISK_ERR ); /* Disk operation error */
}

#ifdef EN_DISK_WRITE
UINT8 CH559WriteSector( UINT8 SectCount, PUINT8X DataBuf) /* Write data blocks of multiple sectors in the buffer to disk */
{
UINT8 retry;
// if (use_external_interface) return( extWriteSector( CH559vLbaCurrent, SectCount, DataBuf) ); /* External interface */
for( retry = 0; retry <3; retry ++) {/* Retry on error */
pCBW -> mCBW_DataLen0 = 0; /* Data transmission length */
pCBW -> mCBW_DataLen1 = SectCount << CH559vSectorSizeB-8;
pCBW -> mCBW_DataLen2 = 0;
pCBW -> mCBW_DataLen3 = 0;
pCBW -> mCBW_Flag = 0x00;
pCBW -> mCBW_LUN = CH559vCurrentLun;
pCBW -> mCBW_CB_Len = 10;
pCBW -> mCBW_CB_Buf[ 0] = SPC_CMD_WRITE10;
pCBW -> mCBW_CB_Buf[ 1] = 0x00;
pCBW -> mCBW_CB_Buf[ 2] = *(PUINT8)&CH559vLbaCurrent;
pCBW -> mCBW_CB_Buf[ 3] = *( (PUINT8)&CH559vLbaCurrent + 1 );
pCBW -> mCBW_CB_Buf[ 4] = *( (PUINT8)&CH559vLbaCurrent + 2 );
pCBW -> mCBW_CB_Buf[ 5] = *( (PUINT8)&CH559vLbaCurrent + 3 );
pCBW -> mCBW_CB_Buf[ 6] = 0x00;
pCBW -> mCBW_CB_Buf[ 7] = 0x00;
pCBW -> mCBW_CB_Buf[ 8] = SectCount;
pCBW -> mCBW_CB_Buf[ 9] = 0x00;
CH559BulkOnlyCmd( DataBuf ); /* Execute commands based on BulkOnly protocol */
if (CH559IntStatus == ERR_SUCCESS) {
mDelayuS( 200 ); /* Delay after write operation */
return( ERR_SUCCESS );
}
CH559IntStatus = CH559AnalyzeError( retry );
if (CH559IntStatus != ERR_SUCCESS) {
return( CH559IntStatus );
}
}
return( CH559IntStatus = ERR_USB_DISK_ERR ); /* Disk operation error */
}
#endif
#endif // NO_DEFAULT_ACCESS_SECTOR

#ifndef NO_DEFAULT_DISK_CONNECT /* Defining NO_DEFAULT_DISK_CONNECT in the application program can prohibit the default check disk connection subroutine, and then replace it with a self-written program */
/* Check if the disk is connected */
UINT8 CH559DiskConnect( void)
{
USB_DEV_AD &= 0x7F;
if (USB_DEV_AD == USB_DEVICE_ADDR || USB_DEV_AD == USB_DEVICE_ADDR + 1) {/* Built-in USB device under Root-HUB */
if ((USB_DEV_AD == USB_DEVICE_ADDR? UHUB0_CTRL: UHUB1_CTRL) & bUH_PORT_EN) {/* The USB device under the built-in Root-HUB exists and is not plugged in */
return( ERR_SUCCESS ); /* The USB device has been connected and not plugged in */
}
else if (USB_HUB_ST & (UINT8)( USB_DEV_AD == USB_DEVICE_ADDR? bUHS_H0_ATTACH: bUHS_H1_ATTACH)) {/* The USB device under the built-in Root-HUB exists */
mDiskConnect:
CH559DiskStatus = DISK_CONNECT; /* Disconnected before */
return( ERR_SUCCESS ); /* The external HUB or USB device has been connected or disconnected and then reconnected */
}
else {/* USB device disconnect */
mDiskDisconn:
CH559DiskStatus = DISK_DISCONNECT;
return( ERR_USB_DISCON );
}
}
#ifndef FOR_ROOT_UDISK_ONLY
else if (USB_DEV_AD> 0x10 && USB_DEV_AD <= 0x14 || USB_DEV_AD> 0x20 && USB_DEV_AD <= 0x24) {/* USB device under the port of external HUB */
if ((USB_DEV_AD & 0x20? UHUB1_CTRL: UHUB0_CTRL) & bUH_PORT_EN) {/* The external HUB under the built-in Root-HUB exists and is not plugged in */
TxBuffer[ MAX_PACKET_SIZE-1] = USB_DEV_AD; /* Backup */
USB_DEV_AD = USB_DEVICE_ADDR-1 + (USB_DEV_AD >> 4 ); /* Set the USB address of the USB host to the HUB */
CH559IntStatus = HubGetPortStatus( TxBuffer[ MAX_PACKET_SIZE-1] & 0x0F ); /* Query the status of the HUB port and return it in TxBuffer */
if (CH559IntStatus == ERR_SUCCESS) {
if (TxBuffer[2] & (1<<(HUB_C_PORT_CONNECTION-0x10))) {/* A plug event on the HUB port is detected */
CH559DiskStatus = DISK_DISCONNECT; /* It is assumed that the USB device on the HUB port is disconnected */
HubClearPortFeature( TxBuffer[ MAX_PACKET_SIZE-1] & 0x0F, HUB_C_PORT_CONNECTION ); /* Clear HUB port connection event status */
}
USB_DEV_AD = TxBuffer[ MAX_PACKET_SIZE-1 ]; /* Set the USB address of the USB host to the USB device */
if (TxBuffer[0] & (1<<HUB_PORT_CONNECTION)) {/* Connection status */
if (CH559DiskStatus <DISK_CONNECT) {
CH559DiskStatus = DISK_CONNECT; /* Disconnected before */
}
return( ERR_SUCCESS ); /* The USB device has been connected or disconnected and then reconnected */
}
else {
// CH559DiskStatus = DISK_DISCONNECT;
// return( ERR_USB_DISCON );
CH559DiskStatus = DISK_CONNECT;
return( ERR_HUB_PORT_FREE ); /* HUB has been connected but HUB port has not been connected to the disk */
}
}
else {
USB_DEV_AD = TxBuffer[ MAX_PACKET_SIZE-1 ]; /* Set the USB address of the USB host to the USB device */
if (CH559IntStatus == ERR_USB_DISCON) {
// CH559DiskStatus = DISK_DISCONNECT;
// return( ERR_USB_DISCON );
goto mDiskDisconn;
}
else {
CH559DiskStatus = DISK_CONNECT; /* HUB operation failed */
return( CH559IntStatus );
}
}
}
else if (USB_HUB_ST & (UINT8)( USB_DEV_AD & 0x20? bUHS_H1_ATTACH: bUHS_H0_ATTACH)) {/* The USB device under the built-in Root-HUB exists, and the external HUB or USB device has been connected or disconnected and then reconnected */
// CH559DiskStatus = DISK_CONNECT; /* Disconnected before */
// return( ERR_SUCCESS ); /* The external HUB or USB device has been connected or disconnected and then reconnected */
goto mDiskConnect;
}
else {/* External HUB disconnected */
CH559DiskStatus = DISK_DISCONNECT;
}
}
#endif
else {
// CH559DiskStatus = DISK_DISCONNECT;
// return( ERR_USB_DISCON );
goto mDiskDisconn;
}
}
#endif // NO_DEFAULT_DISK_CONNECT

#ifndef NO_DEFAULT_FILE_ENUMER /* Defining NO_DEFAULT_FILE_ENUMER in the application program can disable the default file name enumeration callback program, and then replace it with a program written by yourself */
void xFileNameEnumer( void) /* File name enumeration callback subroutine */
{
/* If you call FileOpen after specifying the enumeration number CH559vFileSize as 0xFFFFFFFF, then this callback program will be called every time a file FileOpen is searched.
   After the callback program xFileNameEnumer returns, FileOpen decrements CH559vFileSize and continues to enumerate until no files or directories are found. The recommended approach is,
   Before calling FileOpen, define a global variable as 0. After FileOpen calls back to this program, this program gets the structure FAT_DIR_INFO from CH559vFdtOffset,
   Analyze the DIR_Attr and DIR_Name in the structure to determine whether it is the required file name or directory name, record the relevant information, and increment the global variable count,
   When FileOpen returns, it is judged that if the return value is ERR_MISS_FILE or ERR_FOUND_NAME, the operation is considered to be successful, and the global variable is the number of valid files found.
   If CH559vFileSize is set to 1 in this callback program xFileNameEnumer, then FileOpen can be notified to end the search early. The following is an example of a callback program */
#if 0
UINT8 i;
UINT16 FileCount;
PX_FAT_DIR_INFO pFileDir;
PUINT8 NameBuf;
pFileDir = (PX_FAT_DIR_INFO)( pDISK_BASE_BUF + CH559vFdtOffset ); /* The starting address of the current FDT */
FileCount = (UINT16)( 0xFFFFFFFF-CH559vFileSize ); /* The enumeration number of the current file name, the initial value of CH559vFileSize is 0xFFFFFFFF, and it will decrease after finding the file name */
if (FileCount <sizeof( FILE_DATA_BUF) / 12) {/* Check whether the buffer is sufficient for storage, assuming that each file name takes up 12 bytes to store */
NameBuf = & FILE_DATA_BUF[ FileCount * 12 ]; /* Calculate the buffer address for saving the current file name */
for (i = 0; i <11; i ++) NameBuf[ i] = pFileDir -> DIR_Name[ i ]; /* Copy the file name, the length is 11 characters, no spaces are processed */
// if (pFileDir -> DIR_Attr & ATTR_DIRECTORY) NameBuf[ i] = 1; /* Determine the name of the directory */
NameBuf[ i] = 0; /* End of file name */
}
#endif
}
#endif // NO_DEFAULT_FILE_ENUMER

UINT8 CH559LibInit( void) /* Initialize CH559 library, return 0 when operation is successful */
{
if (CH559GetVer() <CH559_LIB_VER) return( 0xFF ); /* Get the version number of the current subroutine library, if the version is too low, an error will be returned */
#if DISK_BASE_BUF_LEN> 0
pDISK_BASE_BUF = & DISK_BASE_BUF[0]; /* Point to the disk data buffer of external RAM */
pDISK_FAT_BUF = & DISK_BASE_BUF[0]; /* Disk FAT data buffer pointing to external RAM, can be combined with pDISK_BASE_BUF to save RAM */
/* If you want to improve the file access speed, you can re-point pDISK_FAT_BUF to another independently allocated buffer with the same size as pDISK_BASE_BUF after calling CH559LibInit in the main program */
#endif
CH559DiskStatus = DISK_UNKNOWN; /* Unknown status */
CH559vSectorSizeB = 9; /* The default physical disk sector is 512B */
CH559vSectorSizeH = 512 >> 8; // The default physical disk sector is 512B, this value is the high byte of the disk sector size, the low byte is always 0x00, and the value 2 replaces 0x200
CH559vStartLba = 0; /* The default is to automatically analyze FDD and HDD */
return( ERR_SUCCESS );
}
