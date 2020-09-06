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

//#include "CH559BAS.H"

#ifndef __CH559UFI_H__
#define __CH559UFI_H__

#define CH559_LIB_VER 0x13

//#define DISK_BASE_BUF_LEN 512 /* The default disk data buffer size is 512 bytes (you can choose 2048 or even 4096 to support some large-sector U disks). If it is 0, it is forbidden to define the buffer in this file and Specified by the application in pDISK_BASE_BUF*/
/* If you need to reuse the disk data buffer to save RAM, you can define DISK_BASE_BUF_LEN as 0 to prohibit the definition of the buffer in this file, and the start address of the buffer that will be shared with other programs before calling CH559LibInit Put the pDISK_BASE_BUF variable*/

//#define NO_DEFAULT_ACCESS_SECTOR 1 /* The default disk sector read and write subroutine is forbidden, and replace it with a self-written program below */
//#define NO_DEFAULT_DISK_CONNECT 1 /* The default check disk connection subroutine is forbidden, replace it with a self-written program below */
//#define NO_DEFAULT_FILE_ENUMER 1 /* The default file name enumeration callback program is forbidden, and replace it with a self-written program below */

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************ ************************************************** ******************* */

/* FILE: CH559UF.H */

/* error code */
#ifndef ERR_SUCCESS
#define ERR_SUCCESS 0x00 /* The operation was successful */
#endif
#ifndef ERR_DISK_DISCON
#define ERR_CH559_ERROR 0x81 /* CH559 hardware error, may need to reset CH559 */
//#define ERR_DISK_DISCON 0x82 /* The disk is not connected yet, it may be disconnected */
#define ERR_STATUS_ERR 0x83 /* The disk status is wrong, it may be connecting or disconnecting the disk */
#define ERR_HUB_PORT_FREE 0x84 /* The USB-HUB has been connected but the HUB port has not been connected to the disk, and the disk may have been disconnected */
#define ERR_MBR_ERROR 0x91 /* The master boot record of the disk is invalid, the disk may not be partitioned or formatted */
#define ERR_TYPE_ERROR 0x92 /* The disk partition type is not supported, only FAT12/FAT16/BigDOS/FAT32 is supported, and it needs to be repartitioned by the disk management tool */
#define ERR_BPB_ERROR 0xA1 /* The disk has not been formatted, or the parameters are wrong, and need to be reformatted by WINDOWS using the default parameters */
#define ERR_TOO_LARGE 0xA2 /* The disk is formatted abnormally and the capacity is greater than 4GB, or the capacity is greater than 250GB, it needs to be reformatted by WINDOWS with default parameters */
#define ERR_FAT_ERROR 0xA3 /* The file system of the disk does not support, only FAT12/FAT16/FAT32 is supported, and it needs to be reformatted by WINDOWS with default parameters */
#define ERR_DISK_FULL 0xB1 /* The disk file is too full, the remaining space is too little or there is no more, it needs disk defragment */
#define ERR_FDT_OVER 0xB2 /* There are too many files in the directory, no free directory entries, the number of files in the FAT12/FAT16 root directory should be less than 500, and disk defragmentation is required */
#define ERR_MISS_DIR 0xB3 /* A subdirectory of the specified path is not found, it may be the directory name is wrong */
#define ERR_FILE_CLOSE 0xB4 /* The file has been closed, if you need to use it, you should reopen the file */
#define ERR_OPEN_DIR 0x41 /* The directory of the specified path is opened */
#define ERR_MISS_FILE 0x42 /* The file in the specified path was not found, it may be the wrong file name */
#define ERR_FOUND_NAME 0x43 /* The file name that matches the wildcard is searched. The file name and its full path are in the command buffer. If you need to use it, you should open the file */
#endif
/* Code 2XH-3XH is used for the communication failure code of USB host mode, and the return of CH375 is imitated by the CH559 subroutine */
/* Code 1XH is used for the operation status code of USB host mode, and the return of CH375 is imitated by the CH559 subroutine */
#ifndef ERR_USB_CONNECT
#define ERR_USB_CONNECT_LS 0x13 /* Low-speed USB device connection event detected */
#define ERR_USB_CONNECT 0x15 /* A USB device connection event was detected, and the disk has been connected */
#define ERR_USB_DISCON 0x16 /* A USB device disconnection event was detected, and the disk has been disconnected */
#define ERR_USB_BUF_OVER 0x17 /* The data transmitted by USB is wrong or there is too much data and buffer overflow */
#define ERR_USB_DISK_ERR 0x1F /* USB storage operation failed, it may be that the USB storage is not supported during initialization, and the disk may be damaged or disconnected during read and write operations */
#define ERR_USB_TRANSFER 0x20 /* NAK/STALL and other error codes are 0x20~0x2F */
#endif

/* Disk and file status */
#define DISK_UNKNOWN 0x00 /* Not yet initialized, unknown state */
#define DISK_DISCONNECT 0x01 /* The disk is not connected or has been disconnected */
#define DISK_CONNECT 0x02 /* The disk has been connected, but it has not been initialized or the disk cannot be recognized */
#define DISK_USB_ADDR 0x04 /* The disk has been assigned the USB device address, but the USB and the initialization disk have not been configured */
#define DISK_MOUNTED 0x05 /* The disk has been initialized successfully, but the file system has not been analyzed or the file system is not supported */
#define DISK_READY 0x10 /* The file system of the disk has been analyzed and can support */
#define DISK_OPEN_ROOT 0x12 /* The root directory has been opened, in sector mode, the contents of the directory can only be read and written in units of sectors. It must be closed after use. Note that the FAT12/FAT16 root directory is a fixed length */
#define DISK_OPEN_DIR 0x13 /* The subdirectory has been opened, in sector mode, the contents of the directory can only be read and written in units of sectors */
#define DISK_OPEN_FILE 0x14 /* The file has been opened, in sector mode, data can be read and written in units of sectors */
#define DISK_OPEN_FILE_B 0x15 /* The file has been opened, byte mode, data can be read and written in bytes */

/* FAT type mark */
#ifndef DISK_FAT16
#define DISK_FS_UNKNOWN 0 /* Unknown file system */
#define DISK_FAT12 1 /* FAT12 file system */
#define DISK_FAT16 2 /* FAT16 file system */
#define DISK_FAT32 3 /* FAT32 file system */
#endif

/* File directory information in FAT data area */
typedef struct _FAT_DIR_INFO {
UINT8 DIR_Name[11]; /* 00H, file name, a total of 11 bytes, fill in spaces for deficiencies */
UINT8 DIR_Attr; /* 0BH, file attribute, refer to the description below */
UINT8 DIR_NTRes; /* 0CH */
UINT8 DIR_CrtTimeTenth; /* 0DH, file creation time, counted in 0.1 second unit */
UINT16 DIR_CrtTime; /* 0EH, the time of file creation */
UINT16 DIR_CrtDate; /* 10H, the date of file creation */
UINT16 DIR_LstAccDate; /* 12H, the date of the last access operation */
UINT16 DIR_FstClusHI; /* 14H */
UINT16 DIR_WrtTime; /* 16H, file modification time, refer to the macro MAKE_FILE_TIME */
UINT16 DIR_WrtDate; /* 18H, file modification date, refer to macro MAKE_FILE_DATA */
UINT16 DIR_FstClusLO; /* 1AH */
UINT32 DIR_FileSize; /* 1CH, file length */
} FAT_DIR_INFO; /* 20H */

typedef FAT_DIR_INFO xdata *PX_FAT_DIR_INFO;

/* File attributes */
#define ATTR_READ_ONLY 0x01 /* The file is read-only */
#define ATTR_HIDDEN 0x02 /* The file is a hidden attribute */
#define ATTR_SYSTEM 0x04 /* The file is a system attribute */
#define ATTR_VOLUME_ID 0x08 /* Volume label */
#define ATTR_DIRECTORY 0x10 /* subdirectory */
#define ATTR_ARCHIVE 0x20 /* File is archive attribute */
#define ATTR_LONG_NAME (ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID)
/* File attribute UINT8 */
/* bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7 */
/* Only hide the volume catalog, undefined */
/* Read the Tibetan standard record file */
/* File time UINT16 */
/* Time = (Hour<<11) + (Minute<<5) + (Second>>1) */
#define MAKE_FILE_TIME( h, m, s) ((h<<11) + (m<<5) + (s>>1)) /* Generate file time data for specified hours, minutes and seconds */
/* File date UINT16 */
/* Date = ((Year-1980)<<9) + (Month<<5) + Day */
#define MAKE_FILE_DATE( y, m, d) (((y-1980)<<9) + (m<<5) + d) /* Generate file date data of specified year, month and day */

/* file name */
#define PATH_WILDCARD_CHAR 0x2A /* Wildcard for path name'*' */
#define PATH_SEPAR_CHAR1 0x5C /* Path name separator'\' */
#define PATH_SEPAR_CHAR2 0x2F /* Path name separator'/' */
#ifndef MAX_PATH_LEN
#define MAX_PATH_LEN 32 /* Maximum path length, including all slash separators and decimal separators, and the path terminator 00H */
#endif

/* External command parameters */
typedef union _CMD_PARAM {
struct {
UINT8 mBuffer[ MAX_PATH_LEN ];
} Other;
// struct {
// UINT32 mTotalSector; /* Returns: the total number of sectors of the current logical disk */
// UINT32 mFreeSector; /* Returns: the number of remaining sectors of the current logical disk */
// UINT32 mSaveValue;
//} Query; /* CMD_DiskQuery, query disk information */
struct {
UINT8 mPathName[ MAX_PATH_LEN ]; /* Input parameters: Path: [drive letter, colon, slash, directory name or file name and extension..., terminator 00H], where the drive letter and colon can be omitted, such as "C :\DIR1.EXT\DIR2\FILENAME.EXT",00H */
} Open; /* CMD_FileOpen, open file */
// struct {
// UINT8 mPathName[ MAX_PATH_LEN ]; /* Input parameters: Path: [drive letter, colon, slash, directory name or file name and extension (including wildcard *)..., terminator 00H], where the drive letter and The colon can be omitted, for example "C:\DIR1.EXT\DIR2\FILE*",00H */
//} Open; /* CMD_FileOpen, enumerate files, the highest bit of CH559vFileSize is 1, each call xFileNameEnumer, if it is 0, the file name of the specified serial number is returned */
struct {
UINT8 mUpdateLen; /* Input parameters: Whether to allow the update length: 0 prohibits, 1 allows */
} Close; /* CMD_FileClose, close the current file */
// struct {
// UINT8 mPathName[ MAX_PATH_LEN ]; /* Input parameters: Path: [drive letter, colon, slash, directory name or file name and extension..., terminator 00H], where the drive letter and colon can be omitted, for example "C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
//} Create; /* CMD_FileCreate, create a new file and open it, if the file already exists, delete it and then create a new one */
// struct {
// UINT8 mPathName[ MAX_PATH_LEN ]; /* Input parameters: Path: [drive letter, colon, slash, directory name or file name and extension..., terminator 00H], where the drive letter and colon can be omitted, for example "C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
//} Erase; /* CMD_FileErase, delete the file and close */
struct {
UINT32 mFileSize; /* Input parameter: new file length, if it is 0FFFFFFFFH, it will not be modified, return: original length */
UINT16 mFileDate; /* Input parameter: the new file date, if it is 0FFFFH, it will not be modified, return: the original date */
UINT16 mFileTime; /* Input parameter: the new file time, if it is 0FFFFH, it will not be modified, return: the original time */
UINT8 mFileAttr; /* Input parameter: new file attribute, if it is 0FFH, it will not be modified, return: original attribute */
} Modify; /* CMD_FileQuery, query the information of the current file; CMD_FileModify, query or modify the information of the current file */
// struct {
// UINT32 mSaveCurrClus;
// UINT32 mSaveLastClus;
//} Alloc; /* CMD_FileAlloc, adjust the disk space allocated for the file according to the file length */
// struct {
// UINT32 mSectorOffset; /* Input parameters: sector offset, 0 is moved to the beginning of the file, 0FFFFFFFFH is moved to the end of the file, return: the absolute linear sector number corresponding to the current file pointer, 0FFFFFFFFH is the end of the file */
// UINT32 mLastOffset;
//} Locate; /* CMD_FileLocate, move the current file pointer */
// struct {
// UINT8 mSectorCount; /* Input parameter: read sector number, return: actual read sector number */
// UINT8 mActCnt;
// UINT8 mLbaCount;
// UINT8 mRemainCnt;
// PUINT8X mDataBuffer; /* Input parameter: start address of buffer, return: current address of buffer */
// UINT32 mLbaStart;
//} Read; /* CMD_FileRead, read data from the current file */
// struct {
// UINT8 mSectorCount; /* Input parameter: number of sectors written, return: number of sectors actually written */
// UINT8 mActCnt;
// UINT8 mLbaCount;
// UINT8 mAllocCnt;
// PUINT8X mDataBuffer; /* Input parameter: start address of buffer, return: current address of buffer */
// UINT32 mLbaStart;
// UINT32 mSaveValue;
//} Write; /* CMD_FileWrite, write data to the current file */
struct {
UINT32 mDiskSizeSec; /* Returns: the total number of sectors of the entire physical disk, only the first call */
} DiskReady; /* CMD_DiskReady, query disk readiness */
// struct {
// UINT32 mByteOffset; /* Input parameters: the offset in bytes, the file pointer in bytes, return: the absolute linear sector number corresponding to the current file pointer, 0FFFFFFFFH has reached the end of the file */
// UINT32 mLastOffset;
//} ByteLocate; /* CMD_ByteLocate, move the current file pointer in bytes */
struct {
UINT16 mByteCount; /* Input parameter: the number of bytes to be read, return: the number of bytes actually read */
PUINT8X mByteBuffer; /* Input parameter: point to the buffer for storing the read data block */
UINT16 mActCnt;
} ByteRead; /* CMD_ByteRead, read the data block from the current file in bytes */
// struct {
// UINT16 mByteCount; /* Input parameter: the number of bytes to be written, return: the number of bytes actually written */
// PUINT8X mByteBuffer; /* Input parameter: Point to the buffer that stores the read data block */
// UINT16 mActCnt;
//} ByteWrite; /* CMD_ByteWrite, write a data block to the current file in bytes */
// struct {
// UINT8 mSaveVariable; /* Input parameters: 0 to restore the variables of a single U disk, 0x80 to restore the variables of multiple U disks, and other values ​​to backup/save variables */
// UINT8 mReserved[3];
// PUINT8X mBuffer; /* Input parameter: Point to the backup buffer of the variable of the subroutine library, the length is not less than 80 bytes */
//} SaveVariable; /* CMD_SaveVariable, backup/save/restore subroutine library variables */
} CMD_PARAM;

typedef CMD_PARAM idata CMD_PARAM_I;
//typedef CMD_PARAM data *P_CMD_PARAM;

/* SCSI command code */
#ifndef SPC_CMD_INQUIRY
#define SPC_CMD_INQUIRY 0x12
#define SPC_CMD_READ_CAPACITY 0x25
#define SPC_CMD_READ10 0x28
#define SPC_CMD_WRITE10 0x2A
#define SPC_CMD_TEST_READY 0x00
#define SPC_CMD_REQUEST_SENSE 0x03
#define SPC_CMD_MODESENSE6 0x1A
#define SPC_CMD_MODESENSE10 0x5A
#define SPC_CMD_START_STOP 0x1B
#endif

/* FILE: CH559UFI.C */

//#define EN_DISK_WRITE 1

#ifndef DISK_BASE_BUF_LEN
#define DISK_BASE_BUF_LEN 512 /* The default disk data buffer size is 512 bytes, it is recommended to choose 2048 or even 4096 to support some large-sector U disks, and 0 to prohibit the definition of the buffer in the .H file and the application The program is specified in pDISK_BASE_BUF*/
#endif

/* Variables provided in the subroutine library */
extern UINT8DV CH559IntStatus; /* Interrupt status of CH559 operation */
extern UINT8DV CH559DiskStatus; /* Disk and file status */
extern UINT8X CH559vDiskFat; /* FAT flag of logical disk: 1=FAT12, 2=FAT16, 3=FAT32 */
extern UINT8D CH559vSecPerClus; /* The number of sectors per cluster of the logical disk */
extern UINT8D CH559vSectorSizeB; /* log2(CH559vSectorSizeH) */
extern UINT32X CH559vStartLba; /* The absolute starting sector number of the logical disk LBA */
extern UINT32X CH559vDiskRoot; /* For FAT16 disks, it is the number of sectors occupied by the root directory, and for FAT32 disks, it is the starting cluster number of the root directory */
extern UINT32X CH559vDataStart; /* The starting LBA of the data area of ​​the logical disk */
extern UINT32X CH559vStartCluster; /* The starting cluster number of the current file or directory */
extern UINT32D CH559vFileSize; /* The length of the current file */
extern UINT32D CH559vCurrentOffset; /* Current file pointer, byte offset of current reading and writing position */
extern UINT32X CH559vFdtLba; /* The current LBA address of FDT */
extern UINT32D CH559vLbaCurrent; /* The starting LBA address of the disk currently read and written */
extern UINT16X CH559vFdtOffset; /* The offset address of the current FDT in the sector */
extern UINT8D CH559vSectorSizeH; /* The high byte of the disk sector size, the low byte is always 0x00, and the value 2 replaces 0x200 */
extern UINT8X CH559vCurrentLun; /* Logical unit number of the current disk operation */
extern BOOL CH559vSubClassIs6; /* The subclass of USB storage device is 6, 0 is not 6 */
extern PUINT8X data pDISK_BASE_BUF; /* Point to the disk data buffer of external RAM, the length of the buffer is not less than CH559vSectorSizeH, initialized by the application */
extern PUINT8X xdata pDISK_FAT_BUF; /* Point to the disk FAT data buffer of the external RAM, the buffer length is not less than CH559vSectorSizeH, initialized by the application */

extern CMD_PARAM_I mCmdParam; /* Command parameter */

extern UINT8X RxBuffer[ MAX_PACKET_SIZE ]; // IN, must even address
extern UINT8X TxBuffer[ MAX_PACKET_SIZE ]; // OUT, must even address

#ifndef pSetupReq
#define pSetupReq ((PXUSB_SETUP_REQ)TxBuffer)
#endif
#ifndef pCBW
#define pCBW ((PXUDISK_BOC_CBW)TxBuffer)
#define pCSW ((PXUDISK_BOC_CSW)RxBuffer)
#endif
#ifndef pBOC_buf
#define pBOC_buf (TxBuffer+((USB_BO_CBW_SIZE+4)&0xFE))
#endif

#if DISK_BASE_BUF_LEN> 0
extern UINT8X DISK_BASE_BUF[ DISK_BASE_BUF_LEN ]; /* The disk data buffer of the external RAM, the length of the buffer is the length of a sector */
#endif
extern UINT8 CH559ReadSector( UINT8 SectCount, PUINT8X DataBuf ); /* Read multiple sectors of data from the disk to the buffer */
#ifdef EN_DISK_WRITE
extern UINT8 CH559WriteSector( UINT8 SectCount, PUINT8X DataBuf ); /* Write data blocks of multiple sectors in the buffer to disk */
#endif

extern UINT8 CH559DiskConnect( void ); /* Check whether the disk is connected and update the disk status */
extern void xFileNameEnumer( void ); /* Call externally defined subroutine, file name enumeration callback subroutine */

extern UINT8 CH559LibInit( void ); /* Initialize CH559 library, return 0 if operation is successful */

/* Subroutines provided in the subroutine library */
/* In the following subprograms, both the file operation subprogram CH559File* and the disk query subprogram CH559DiskQuery may use the disk data buffer pDISK_BASE_BUF,
   And it is possible to save disk information in pDISK_BASE_BUF, so it must be ensured that pDISK_BASE_BUF is not used for other purposes.
   If there is less RAM, pDISK_BASE_BUF should be temporarily used for other purposes, then CH559DirtyBuffer must be called to clear the disk buffer */
extern UINT8 CH559GetVer( void ); /* Get the version number of the current subroutine library */
extern void CH559DirtyBuffer( void ); /* Clear disk buffer */
extern UINT8 CH559BulkOnlyCmd( PUINT8X DataBuf ); /* Execute commands based on BulkOnly protocol */
extern UINT8 CH559DiskReady( void ); /* Query whether the disk is ready */
extern UINT8 CH559AnalyzeError( UINT8 iMode ); /* USB operation failure analysis CH559IntStatus returns error status */
extern UINT8 CH559FileOpen( void ); /* Open file or enumerate file */
extern UINT8 CH559FileClose( void ); /* Close the current file */
#ifdef EN_DISK_WRITE
extern UINT8 CH559FileErase( void ); /* Delete the file and close */
extern UINT8 CH559FileCreate( void ); /* Create a new file and open it, if the file already exists, delete it and then create a new one */
extern UINT8 CH559FileAlloc( void ); /* Adjust the disk space allocated for the file according to the file length */
#endif
//extern UINT8 CH559FileModify( void ); /* Query or modify the information of the current file */
extern UINT8 CH559FileQuery( void ); /* Query the information of the current file */
//extern UINT8 CH559FileLocate( void ); /* Move the current file pointer */
//extern UINT8 CH559FileRead( void ); /* Read data from the current file to the specified buffer */
#ifdef EN_DISK_WRITE
extern UINT8 CH559FileWrite( void ); /* Write the data of the specified buffer to the current file */
#endif
//extern UINT8 CH559ByteLocate( void ); /* Move the current file pointer in bytes */
extern UINT8 CH559ByteRead( void ); /* Read the data block from the current position in bytes */
#ifdef EN_DISK_WRITE
extern UINT8 CH559ByteWrite( void ); /* Write a data block to the current position in bytes */
#endif
//extern UINT8 CH559DiskQuery( void ); /* Query disk information */
//extern void CH559SaveVariable( void ); /* Backup/save/restore the variables of the subroutine library, used for the subroutine library to switch between multiple chips or U disk */

extern void mDelayuS( UINT16 n ); // Delay in uS
extern void mDelaymS( UINT16 n ); // Delay in mS
extern UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout ); // CH559 transfer transaction, input destination endpoint address/PID token, synchronization flag, NAK retry time, return 0 success, timeout/error retry
extern UINT8 HostCtrlTransfer( PUINT8X DataBuf, PUINT8I RetLen ); // Perform control transfer, the 8-byte request code is in pSetupReq, DataBuf is an optional transmit and receive buffer, and the actual transmit and receive length is returned in the variable pointed to by ReqLen
extern void CopySetupReqPkg( PUINT8C pReqPkt ); // copy control transmission request packet
//extern UINT8 CtrlGetDeviceDescr( void ); // Get the device descriptor and return it in TxBuffer
extern UINT8 CtrlGetConfigDescr( void ); // Get the configuration descriptor and return it in TxBuffer
//extern UINT8 CtrlSetUsbAddress( UINT8 addr ); // Set USB device address
extern UINT8 CtrlSetUsbConfig( UINT8 cfg ); // Set USB device configuration
extern UINT8 CtrlClearEndpStall( UINT8 endp ); // Clear endpoint STALL
//extern UINT8 CtrlGetHubDescr( void ); // Get the HUB descriptor and return it in TxBuffer
//extern UINT8 HubGetPortStatus( UINT8 HubPortIndex ); // Query the status of the HUB port and return it in TxBuffer
//extern UINT8 HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt ); // Set HUB port characteristics
//extern UINT8 HubClearPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt ); // Clear HUB port characteristics

#ifdef __cplusplus
}
#endif

#endif
