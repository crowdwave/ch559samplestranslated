/*
Module Name:Main.cpp
*****************************************
** Copyright (C) WCH 2001-2015 **
** Web: http://wch.cn **
*****************************************
Environment:
    user mode only,VC6.0
Revision History:
    9/10/2015: TECH
Descriptor:
    WCH MCU IAP Windows download sample interface code
Support CH56X, CH55X MCU
*/

#include "resource.h"
#include <stdio.h>
#include "Main.h"
#include "IAP.H"
#include "CH375DLL.H"
#pragma comment (lib,"CH375DLL")

//Global variables
HWND AfxMainHwnd; //Main form handle
HINSTANCE AfxMainIns; //Process instance

DEFINE_GUID(GUID_DEVINTERFACE_COMPORT, 0x86e0d1e0L, 0x8089, 0x11d0, 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);

#define mDnDev_MAX_NUMBER 16
DnDevInforS AfxDnDev[mDnDev_MAX_NUMBER] = {0}; //Device list
UCHAR AfxDnDevCnt;
ULONG AfxDnInterface;
BOOL IsDownloading, IsDeviceChanged;

//Function declaration
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

//Application entrance
int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR lpCmdLine,
                     int nCmdShow)
{
AfxMainIns = hInstance;
return DialogBox(hInstance, (LPCTSTR)IDD_IAPDEMO_DIALOG, 0, (DLGPROC)WndProc);
}

//Find the download device interface and display it
ULONG ScanDnDeviceAndShow()
{
ULONG i, DevID;
CHAR FmtStr[128] = "";

AfxDnDevCnt = 0;
if( AfxDnInterface == 0) //USB interface
{
for (i = 0;i <mDnDev_MAX_NUMBER;++i)
{
if( CH375OpenDevice(i) != INVALID_HANDLE_VALUE )//Find a device
{
DevID = CH375GetUsbID(i);
if( (LOWORD(DevID) == 0x4348) || //VID number of the downloaded USB device, corresponding to the MCU
(HIWORD(DevID) == 0x55E0)) //PID number of the downloaded USB device, corresponding to the MCU
{
strcpy(&AfxDnDev[AfxDnDevCnt].DevName[0],(PCHAR)CH375GetDeviceName(i));
if( strlen(AfxDnDev[AfxDnDevCnt].DevName) == 0) //Invalid device name
continue;
AfxDnDev[AfxDnDevCnt].iIndex = AfxDnDevCnt;
AfxDnDevCnt++;
}
CH375CloseDevice(i);
}
}
if( AfxDnDevCnt)
DbgPrint("%d USB devices have been found", AfxDnDevCnt);
else
DbgPrint("USB device not found, please check whether the MCU has downloaded the IAP code, and whether the MCU has entered IAP mode", AfxDnDevCnt);
}
else //Enumerate serial port
{
SP_DEVINFO_DATA DeviceInfoData;
HDEVINFO COMDeviceInfoSet = INVALID_HANDLE_VALUE;
CHAR PortName[24];
DWORD dwType, dwComNameLen;
HKEY hDeviceKey;
To
COMDeviceInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT,0,AfxMainHwnd, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT ); // COM Port class present on system
//Enumerate through all Devices.
DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
for (i=0;SetupDiEnumDeviceInfo(COMDeviceInfoSet,i,&DeviceInfoData);i++)
{
PortName[0] = 0;
hDeviceKey = SetupDiOpenDevRegKey(COMDeviceInfoSet,
&DeviceInfoData,DICS_FLAG_GLOBAL,0,DIREG_DEV,KEY_ALL_ACCESS);
if (hDeviceKey != INVALID_HANDLE_VALUE)
{
dwComNameLen = sizeof(PortName);
RegQueryValueEx(hDeviceKey,REGSTR_VAL_PORTNAME,0,&dwType,(PBYTE)PortName,&dwComNameLen);
RegCloseKey( hDeviceKey );
}
if( strlen(PortName)) //Find a serial port
{
strcpy(&AfxDnDev[AfxDnDevCnt].DevName[0],PortName);
AfxDnDev[AfxDnDevCnt].iIndex = AfxDnDevCnt;
AfxDnDevCnt++;
}
}
if( COMDeviceInfoSet != INVALID_HANDLE_VALUE)
SetupDiDestroyDeviceInfoList(COMDeviceInfoSet);
DbgPrint("%d serial ports have been found", AfxDnDevCnt);
}

//Clear the contents of the device list
SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_RESETCONTENT,0,0);
for(i=0;i<AfxDnDevCnt;i++)
{
if( AfxDnInterface == 0)
sprintf(FmtStr,"%d number equipment",i); //display name can be customized
else
sprintf(FmtStr,"%s",AfxDnDev[i].DevName); //Display name can be customized
SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)FmtStr);
}
SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_SETCURSEL,0,0);

return AfxDnDevCnt;
}

// Monitor the plugging and unplugging of USB devices
VOID CALLBACK NotifyUsbDnDeviceRoutine(ULONG iEventStatus) // Device event and current status (defined in the lower line): 0=device unplug event, 3=device plug-in event
{
if( AfxDnInterface != 0) //Non-USB download, no need to update the device list
return;
if( (iEventStatus == CH375_DEVICE_ARRIVAL) || (iEventStatus == CH375_DEVICE_REMOVE) )//device insertion
{
if( IsDownloading) //The program is being downloaded, pause and refresh the device list
IsDeviceChanged = TRUE; //Refresh the device list after downloading
else
{
IsDeviceChanged = FALSE;
PostMessage(GetDlgItem(AfxMainHwnd,IDC_ScanDev),BM_CLICK,0,0); //Refresh the device list
}
}
}

//Main form process
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
int wmId, wmEvent;
AfxMainHwnd = hWnd;

switch (message)
{
case WM_INITDIALOG:
//Initial download method
SendDlgItemMessage(hWnd,IDC_DnInterface,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)"USB");
SendDlgItemMessage(hWnd,IDC_DnInterface,CB_ADDSTRING,0,(LPARAM)(LPCTSTR)"serial port");
SendDlgItemMessage(hWnd,IDC_DnInterface,CB_SETCURSEL,0,0);
AfxDnInterface = SendDlgItemMessage(hWnd,IDC_DnInterface,CB_GETCURSEL,0,0);

CH375SetDeviceNotify(0,NULL,NotifyUsbDnDeviceRoutine); //Start device plug-in monitoring
IsDownloading = IsDeviceChanged = FALSE;

PostMessage(GetDlgItem(AfxMainHwnd,IDC_ScanDev),BM_CLICK,0,0); //Refresh the device list
break;
case WM_COMMAND:
wmId = LOWORD(wParam);
wmEvent = HIWORD(wParam);
// Parse the menu selections:
switch (wmId)
{
case IDC_DnInterface:
if( (wmEvent == CBN_SELCHANGE)) //Update the interface device list
{
//Refresh the device list
AfxDnInterface = SendDlgItemMessage(hWnd,IDC_DnInterface,CB_GETCURSEL,0,0);
ScanDnDeviceAndShow(); //Refresh the device list
}
break;
case IDC_SelectFile: //Select download file
To
{// Get the file name to be sent
CHAR FmtStr[256] = "",FileName[512] = "";
OPENFILENAME mOpenFile={0};

sprintf(FmtStr,"Select download file");
// Fill in the OPENFILENAME structure to support a template and hook.
mOpenFile.lStructSize = sizeof(OPENFILENAME);
mOpenFile.hwndOwner = AfxMainHwnd;
mOpenFile.hInstance = AfxMainIns;
mOpenFile.lpstrFilter = "*.HEX\0*.HEX\0*.BIN\0*.BIN\0";
mOpenFile.lpstrCustomFilter = NULL;
mOpenFile.nMaxCustFilter = 0;
mOpenFile.nFilterIndex = 0;
mOpenFile.lpstrFile = FileName;
mOpenFile.nMaxFile = sizeof(FileName);
mOpenFile.lpstrFileTitle = NULL;
mOpenFile.nMaxFileTitle = 0;
mOpenFile.lpstrInitialDir = NULL;
mOpenFile.lpstrTitle = FmtStr;
To
mOpenFile.nFileOffset = 0;
mOpenFile.nFileExtension = 0;
mOpenFile.lpstrDefExt = NULL;
mOpenFile.lCustData = 0;
mOpenFile.lpfnHook = NULL;
mOpenFile.lpTemplateName = NULL;
mOpenFile.Flags = OFN_SHOWHELP | OFN_EXPLORER | OFN_READONLY | OFN_FILEMUSTEXIST;
if (!GetOpenFileName(&mOpenFile))
{
DbgPrint("Error selecting file to send");
}
else
{
SetDlgItemText(AfxMainHwnd,IDC_DownloadFile,FileName); //Save the downloaded file
SendDlgItemMessage(AfxMainHwnd,IDC_DownloadFile,EM_SETSEL,0xFFFFFFFE,0xFFFFFFFE);//Move the caret to the end of the text box
}
}
break;
case IDC_ScanDev: //Scan device
ScanDnDeviceAndShow();
break;
case IDC_ClearResult: //Clear the result information
SetDlgItemText(AfxMainHwnd,IDC_ResultShow,"");
break;
case IDC_Download: //Download file
DWORD ThreadID;

if( SendDlgItemMessage(hWnd,IDC_DeviceList,CB_GETCURSEL,0,0) == CB_ERR)
{
MessageBox(AfxMainHwnd,"Please select a download device first or no download device is currently found","IAP DEMO",MB_ICONERROR);
DbgPrint("Please select the download device first or no download device is currently found.");
break;
}
{//Determine whether the downloaded file exists
CHAR FileName[MAX_PATH] = "";
OFSTRUCT lpReOpenBuff = {0};

if( GetDlgItemText(AfxMainHwnd,IDC_DownloadFile,FileName,sizeof(FileName)) <1)
{
MessageBox(AfxMainHwnd,"Please select the download file first","IAP DEMO",MB_ICONERROR);
DbgPrint("Download failed! Please select download file first");
break;
}
if (OpenFile(FileName,&lpReOpenBuff,OF_EXIST) == HFILE_ERROR)
{
MessageBox(AfxMainHwnd,"The downloaded file does not exist or is in use","IAP DEMO",MB_ICONERROR);
DbgPrint("Download failed! The downloaded file does not exist or is in use");
break;
}
}
CloseHandle(CreateThread(NULL,0,IAPFlashDownloadThread,NULL,0,&ThreadID)); //Start USB download

break;
case WM_DESTROY:
CH375SetDeviceNotify(0,NULL,NULL); //Cancel device plug detection
DestroyWindow(hWnd);
break;
default:
return DefWindowProc(hWnd, message, wParam, lParam);
}
break;
case WM_DESTROY:
PostQuitMessage(0);
break;
}
return 0;
}
