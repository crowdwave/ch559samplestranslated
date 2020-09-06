#ifndef _MAIN_H
#define _MAIN_H

// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

#include <windows.h>
#include <stdio.h>
#include <initguid.h>
#include <regstr.h>
#include <setupapi.h>
#pragma comment(lib,"setupapi")
#include "CH375DLL.H"
//#include <afxdlgs.h>
#pragma comment (lib,"CH375DLL")
#include "DbgFunc.h"


//Download device information structure
typedef struct _DnDeviceInfor
{
UCHAR iIndex; //equipment serial number
CHAR DevName[128]; //Device name, used for communication
CHAR DevDispName[128]; //The name displayed in the device manager
}DnDevInforS,*PDnDevInforS;

#endif // MAIN_H
