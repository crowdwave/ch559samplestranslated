// 2003.09.08, 2003.12.28, 2004.10.15, 2004.12.05, 2004.12.10, 2005.01.20, 2005.02.23, 2005.07.15, 2005.08.17
//****************************************
//** Copyright (C) W.ch 1999-2005 **
//** Web: http://www.winchiphead.com **
//****************************************
//** DLL for USB interface chip CH375 **
//** C, VC5.0 **
//****************************************
//
// Application layer interface library V2.2 of USB bus interface chip CH375
// Nanjing Qin Heng Electronics Co., Ltd. Author: W.ch 2005.08
// CH375-DLL V2.2, Support: Ctrl/Bulk/Int
// Operating environment: Windows 98/ME, Windows 2000/XP
// support USB chip: CH372/CH375
//

#ifndef _CH375_DLL_H
#define _CH375_DLL_H

#ifdef __cplusplus
extern "C" {
#endif

#define mOFFSET( s, m) ((ULONG) & (((s *) 0) -> m)) // Define the macro to obtain the relative offset address of the structure member

#ifndef max
#define max( a, b) (((a)> (b))? (a): (b)) // larger value
#endif

#ifndef min
#define min( a, b) (((a) <(b))? (a): (b)) // smaller value
#endif

#ifdef ExAllocatePool
#undef ExAllocatePool // Delete memory allocation with TAG
#endif

#ifndef NTSTATUS
typedef LONG NTSTATUS; // return status
#endif


typedef struct _USB_SETUP_PKT {// Data request packet structure in the establishment phase of USB control transmission
UCHAR mUspReqType; // 00H request type
UCHAR mUspRequest; // 01H request code
union {
struct {
UCHAR mUspValueLow; // 02H value parameter low byte
UCHAR mUspValueHigh; // 03H value parameter high byte
};
USHORT mUspValue; // 02H-03H value parameter
};
union {
struct {
UCHAR mUspIndexLow; // 04H index parameter low byte
UCHAR mUspIndexHigh; // 05H index parameter high byte
};
USHORT mUspIndex; // 04H-05H index parameter
};
USHORT mLength; // 06H-07H data length of data phase
} mUSB_SETUP_PKT, *mPUSB_SETUP_PKT;


#define mCH375_PACKET_LENGTH 64 // Length of data packet supported by CH375
#define mCH375_PKT_LEN_SHORT 8 // The length of the short data packet supported by CH375


typedef struct _WIN32_COMMAND {// Define WIN32 command interface structure
union {
ULONG mFunction; // Specify function code or pipe number when input
NTSTATUS mStatus; //Return to operation status when output
};
ULONG mLength; // Access length, return the length of subsequent data
union {
mUSB_SETUP_PKT mSetupPkt; // Data request in the establishment phase of USB control transmission
UCHAR mBuffer[ mCH375_PACKET_LENGTH ]; // Data buffer, length is 0 to 255B
};
} mWIN32_COMMAND, *mPWIN32_COMMAND;


// WIN32 application layer interface command
#define IOCTL_CH375_COMMAND (FILE_DEVICE_UNKNOWN << 16 | FILE_ANY_ACCESS << 14 | 0x0f37 << 2 | METHOD_BUFFERED) // dedicated interface

#define mWIN32_COMMAND_HEAD mOFFSET( mWIN32_COMMAND, mBuffer) // Header length of WIN32 command interface

#define mCH375_MAX_NUMBER 16 // Maximum number of simultaneously connected CH375

#define mMAX_BUFFER_LENGTH 0x1000 // The maximum length of the data buffer is 4096

#define mMAX_COMMAND_LENGTH (mWIN32_COMMAND_HEAD + mMAX_BUFFER_LENGTH) // The maximum data length plus the length of the command structure header

#define mDEFAULT_BUFFER_LEN 0x0400 // The default length of the data buffer is 1024

#define mDEFAULT_COMMAND_LEN (mWIN32_COMMAND_HEAD + mDEFAULT_BUFFER_LEN) // The default data length plus the length of the command structure header


// CH375 endpoint address
#define mCH375_ENDP_INTER_UP 0x81 // The address of the interrupt data upload endpoint of CH375
#define mCH375_ENDP_AUX_DOWN 0x01 // CH375's auxiliary data download endpoint address
#define mCH375_ENDP_DATA_UP 0x82 // The address of the data block upload endpoint of CH375
#define mCH375_ENDP_DATA_DOWN 0x02 // CH375's data block download endpoint address


// The pipeline operation command provided by the device layer interface
#define mPipeDeviceCtrl 0x00000004 // Integrated control pipeline of CH375
#define mPipeInterUp 0x00000005 // CH375 interrupt data upload pipeline
#define mPipeDataUp 0x00000006 // CH375 data block upload pipeline
#define mPipeDataDown 0x00000007 // CH375 data block download pipeline
#define mPipeAuxDown 0x00000008 // CH375's auxiliary data download pipeline

// Function code of application layer interface
#define mFuncNoOperation 0x00000000 // No operation
#define mFuncGetVersion 0x00000001 // Get the driver version number
#define mFuncGetConfig 0x00000002 // Get USB device configuration descriptor
#define mFuncSetExclusive 0x0000000b // Set exclusive use
#define mFuncResetDevice 0x0000000c // Reset the USB device
#define mFuncResetPipe 0x0000000d // Reset the USB pipe
#define mFuncAbortPipe 0x0000000e // Cancel the data request of the USB pipe
#define mFuncSetTimeout 0x0000000f // Set USB communication timeout
#define mFuncBufferMode 0x00000010 // Set the buffer upload mode and query the data length in the buffer
#define mFuncBufferModeDn 0x00000011 // Set the buffer download mode and query the data length in the buffer


// USB device standard request code
#define mUSB_CLR_FEATURE 0x01
#define mUSB_SET_FEATURE 0x03
#define mUSB_GET_STATUS 0x00
#define mUSB_SET_ADDRESS 0x05
#define mUSB_GET_DESCR 0x06
#define mUSB_SET_DESCR 0x07
#define mUSB_GET_CONFIG 0x08
#define mUSB_SET_CONFIG 0x09
#define mUSB_GET_INTERF 0x0a
#define mUSB_SET_INTERF 0x0b
#define mUSB_SYNC_FRAME 0x0c

// The manufacturer-specific request type of CH375 control transmission
#define mCH375_VENDOR_READ 0xc0 // CH375 vendor-specific read operation realized by control transmission
#define mCH375_VENDOR_WRITE 0x40 // CH375 vendor-specific write operation realized by control transmission

// Vendor-specific request code for CH375 control transmission
#define mCH375_SET_CONTROL 0x51 // Output control signal
#define mCH375_GET_STATUS 0x52 // Input status signal

// bit definition of register
#define mBitInputRxd 0x02 // Read only, RXD# pin input status, 1: high level, 0: low level
#define mBitInputReq 0x04 // Read only, REQ# pin input status, 1: high level, 0: low level


// The bit definition of the status signal input directly
#define mStateRXD 0x00000200 // RXD# pin input state, 1: high level, 0: low level
#define mStateREQ 0x00000400 // REQ# pin input state, 1: high level, 0: low level

#define MAX_DEVICE_PATH_SIZE 128 // The maximum number of characters in the device name
#define MAX_DEVICE_ID_SIZE 64 // The maximum number of characters in the device ID


typedef VOID (CALLBACK * mPCH375_INT_ROUTINE) (// Interrupt service callback program
PUCHAR iBuffer ); // Point to a buffer to provide current interrupt characteristic data


HANDLE WINAPI CH375OpenDevice( // Open the CH375 device and return the handle, invalid if it is wrong
ULONG iIndex ); // Specify the serial number of the CH375 device, 0 corresponds to the first device, -1 automatically searches for a device that can be opened and returns the serial number


VOID WINAPI CH375CloseDevice( // Close CH375 device
ULONG iIndex ); // Specify the serial number of the CH375 device


ULONG WINAPI CH375GetVersion( ); // Get the DLL version number, return the version number


ULONG WINAPI CH375DriverCommand( // Pass the command directly to the driver, return 0 if error occurs, otherwise return the data length
ULONG iIndex, // Specify the serial number of the CH375 device, the DLL above V1.6 can also be the handle after the device is opened
mPWIN32_COMMAND ioCommand ); // pointer to command structure
// The program returns the data length after the call, and still returns the command structure, if it is a read operation, the data is returned in the command structure,
// The length of the returned data is 0 when the operation fails, and the length of the entire command structure when the operation succeeds. For example, if one byte is read, mWIN32_COMMAND_HEAD+1 is returned,
// Before calling the command structure, provide: pipe number or command function code, length of access data (optional), data (optional)
// After the command structure is called, it returns: the operation status code, the length of the subsequent data (optional),
// The operation status code is a code defined by WINDOWS, you can refer to NTSTATUS.H,
// The length of the subsequent data refers to the length of the data returned by the read operation. The data is stored in the subsequent buffer. For write operations, it is generally 0


ULONG WINAPI CH375GetDrvVersion( ); // Get the driver version number, return the version number, and return 0 if error occurs


BOOL WINAPI CH375ResetDevice( // Reset USB device
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375GetDeviceDescr( // read device descriptor
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID oBuffer, // Point to a buffer large enough to save the descriptor
PULONG ioLength ); // Point to the length unit, the length to be read when input, and the actual length after return


BOOL WINAPI CH375GetConfigDescr( // read configuration descriptor
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID oBuffer, // Point to a buffer large enough to save the descriptor
PULONG ioLength ); // Point to the length unit, the length to be read when input, and the actual length after return


BOOL WINAPI CH375SetIntRoutine( // Set interrupt service routine
ULONG iIndex, // Specify the serial number of the CH375 device
mPCH375_INT_ROUTINE iIntRoutine ); // Specify the interrupt service callback program, NULL will cancel the interrupt service, otherwise the program will be called when interrupted


BOOL WINAPI CH375ReadInter( // Read interrupt data
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID oBuffer, // Point to a buffer large enough to save the interrupt data read
PULONG ioLength ); // Point to the length unit, the length to be read when input, and the actual length after return


BOOL WINAPI CH375AbortInter( // Abandon interrupt data reading operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375ReadData( // read data block
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID oBuffer, // Point to a buffer large enough to save the read data
PULONG ioLength ); // Point to the length unit, the length to be read when input, and the actual length after return


BOOL WINAPI CH375AbortRead( // Abandon the data block read operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375WriteData( // write data block
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID iBuffer, // Point to a buffer to place the data to be written
PULONG ioLength ); // Point to the length unit, the length to be written when input, and the actual length after returning


BOOL WINAPI CH375AbortWrite( // Abandon data block write operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375WriteRead( // First write the standard data block (command), then read the standard data block (response)
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID iBuffer, // Point to a buffer, place the data to be written, the length is not greater than mCH375_PACKET_LENGTH
PVOID oBuffer, // points to a sufficiently large buffer, the length is not less than mCH375_PACKET_LENGTH, used to save the read data
PULONG ioLength ); // Point to the length unit, not greater than mCH375_PACKET_LENGTH, the length to be written when input, and the length actually read after returning


BOOL WINAPI CH375GetStatus( // Input data and status directly through CH375
ULONG iIndex, // Specify the serial number of the CH375 device
PULONG iStatus ); // Point to a double word unit to save status data
// Bit 7-bit 0 corresponds to the D7-D0 pins of CH375, bit 9 corresponds to the RXD# pin of CH375, and bit 10 corresponds to the REQ# pin of CH375


BOOL WINAPI CH375SetTimeout( // Set the timeout of USB data reading and writing
ULONG iIndex, // Specify the serial number of the CH375 device
ULONG iWriteTimeout, // Specify the timeout time for the USB to write data blocks, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)
ULONG iReadTimeout ); // Specify the timeout time for USB to read the data block, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)


BOOL WINAPI CH375WriteAuxData( // Write auxiliary data
ULONG iIndex, // Specify the serial number of the CH375 device
PVOID iBuffer, // Point to a buffer to place the data to be written
PULONG ioLength ); // Point to the length unit, the length to be written when input, and the actual length after returning


BOOL WINAPI CH375SetExclusive( // Set exclusive use of the current CH375 device
ULONG iIndex, // Specify the serial number of the CH375 device
ULONG iExclusive ); // If it is 0, the device can be shared, and if it is not 0, it can be used exclusively


ULONG WINAPI CH375GetUsbID( // Get the USB device ID, in the returned data, the low 16 bits are the manufacturer ID, the high 16 bits are the product ID, and all 0s (invalid ID) are returned when an error occurs
ULONG iIndex ); // Specify the serial number of the CH375 device


PVOID WINAPI CH375GetDeviceName( // Returns the buffer pointing to the CH375 device name, or NULL if error occurs
ULONG iIndex ); // Specify the serial number of the CH375 device, 0 corresponds to the first device


BOOL WINAPI CH375SetBufUpload( // Set internal buffer upload mode
ULONG iIndex, // Specify the serial number of the CH375 device, 0 corresponds to the first device
ULONG iEnableOrClear ); // 0 to disable the internal buffer upload mode, use direct upload, non-zero to enable the internal buffer upload mode and clear the existing data in the buffer
// If the internal buffer upload mode is enabled, the CH375 driver creates a thread to automatically receive the USB upload data to the internal buffer, and at the same time clear the existing data in the buffer, when the application calls CH375ReadData, it will immediately return to the existing data in the buffer data


LONG WINAPI CH375QueryBufUpload( // Query the number of existing data packets in the internal upload buffer, return the number of data packets successfully, return -1 on error
ULONG iIndex ); // Specify the serial number of the CH375 device, 0 corresponds to the first device


BOOL WINAPI CH375SetBufDownload( // Set internal buffer download mode
ULONG iIndex, // Specify the serial number of the CH375 device, 0 corresponds to the first device
ULONG iEnableOrClear ); // 0 to disable the internal buffer download mode, use direct download, non-zero to enable the internal buffer download mode and clear the existing data in the buffer
// If the internal buffer download mode is enabled, then when the application calls CH375WriteData, it will only put the USB download data into the internal buffer and return immediately, and the thread created by the CH375 driver will automatically send it until the end


LONG WINAPI CH375QueryBufDownload( // Query the number of remaining data packets in the internal download buffer (not yet sent), return the number of data packets successfully, return -1 if error occurs
ULONG iIndex ); // Specify the serial number of the CH375 device, 0 corresponds to the first device


BOOL WINAPI CH375ResetInter( // Reset interrupt data read operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375ResetAux( // Reset auxiliary data write operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375ResetRead( // Reset data block read operation
ULONG iIndex ); // Specify the serial number of the CH375 device


BOOL WINAPI CH375ResetWrite( // Reset data block write operation
ULONG iIndex ); // Specify the serial number of the CH375 device


typedef VOID (CALLBACK * mPCH375_NOTIFY_ROUTINE) (// Device event notification callback program
ULONG iEventStatus ); // Device event and current status (defined in the lower line): 0=device unplug event, 3=device plug-in event

#define CH375_DEVICE_ARRIVAL 3 // Device insertion event, inserted
#define CH375_DEVICE_REMOVE_PEND 1 // The device will be unplugged
#define CH375_DEVICE_REMOVE 0 // Device unplug event, already unplugged


BOOL WINAPI CH375SetDeviceNotify( // Set device event notification program
ULONG iIndex, // Specify the serial number of the CH375 device, 0 corresponds to the first device
PCHAR iDeviceID, // Optional parameter, point to a string, specify the ID of the monitored device, the string is terminated by \0
mPCH375_NOTIFY_ROUTINE iNotifyRoutine ); // Specify the device event callback program, NULL will cancel the event notification, otherwise the program will be called when the event is detected


BOOL WINAPI CH375SetTimeoutEx( // Set the timeout of USB data reading and writing
ULONG iIndex, // Specify the serial number of the CH375 device
ULONG iWriteTimeout, // Specify the timeout time for the USB to write data blocks, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)
ULONG iReadTimeout, // Specify the timeout time for USB to read the data block, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)
ULONG iAuxTimeout, // Specify the timeout time of USB auxiliary download data, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)
ULONG iInterTimeout ); // Specify the timeout period of USB interrupt upload data, in milliseconds mS, 0xFFFFFFFF specifies no timeout (default value)


#ifdef __cplusplus
}
#endif

#endif // _CH375_DLL_H
