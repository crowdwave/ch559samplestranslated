Nanjing Qinheng Electronics Co., Ltd. 2015.04
Please check our website for the latest information
http://wch.cn
http://winchiphead.com


File Directory
  |
  |-- README.TXT: catalog file description
  |-- BRIEF
  | |--CH559DS0.PDF: Introduction to CH559
  |-- DS
  | |--CH559DS1.PDF: CH559 Application Manual
  |-- EVT
  | |-- EXAM: C example
  | | |--CH559.uvproj: CH559 keil project file
  | | |--CH559.H: CH559 C language header file, SFR definition
  | | |--CH559.INC: CH559 assembly header file, SFR definition
  | | |--DEBUG.C: UART0 initialization, commonly used delay functions, UART0 data transceiver sub-functions
  | | |--DEBUG.H: UART0 initialization, commonly used delay functions, UART0 data transceiver sub-function declaration
  | | |--CHIP_ID
  | | | |-- GETID.C: Get the chip's unique ID number
  | | | |-- GETID.hex:
  | | |--Encryption
  | | | |-- Encryption.C: Chip user program encryption routine
  | | | |-- Encryption.hex:
  | | | |-- Chip encryption suggestions.PDF
  | | |--ADC:
  | | | |-- ADCAUTODMA.C: DMA mode automatic AD sampling
  | | | |-- ADCAUTODMA.hex: DMA mode automatic AD sampling hex file
  | | | |-- ADCManual.C: Manual AD sampling
  | | | |-- ADCManual.hex: Manual AD sampling hex file
  | | |--DataFlash
  | | | |-- DataFlash.C: DATA Flash read and write, erase routines
  | | | |-- DataFlash.hex: DATA Flash operation hex file
  | | |--GPIO
  | | | |-- GPIO.C: IO port configuration, drive capability setting and IO interrupt setting
  | | | |-- GPIO.hex: IO operation hex file
  | | |--LED_CTL
  | | | |-- LED_Udisk_Flash_DMA
  | | | | |--Complete demo project, need to add file system LIB
  | | |--PWM
  | | | |-- PWM.C: PWM1 and PWM2 operating procedures
  | | | |-- PWM.hex: PWM1 and PWM2 operation hex file
  | | |--SPI0
  | | | |-- SPI0Flash.C: SPI0 operation serial Flash
  | | | |-- SPI0Flash.hex: hex file for SPI0 operation serial Flash
  | | | |-- SPI0Master.C: SPI0 master sending and receiving program
  | | | |-- SPI0Master.hex: SPI0 host interface function hex file
  | | | |-- SPI0Slv.C: SPI0 slave initialization program
  | | | |-- SPI0Slv.hex: SPI0 slave interface function hex file
  | | |--SPI1
  | | | |-- SPI1Master.C: SPI1 master sending and receiving program
  | | | |-- SPI1Master.hex: SPI1 host interface function hex file
  | | |--Timer0
  | | | |-- Timer0.C: Timer0 interrupt, timer setting
  | | | |-- Timer0.hex: Timer0 operation hex file
  | | |--Timer1
  | | | |-- Timer1.C: Timer1 set interrupt, timer setting
  | | | |-- Timer1.hex: Timer1 operation hex file
  | | |--Timer2
  | | | |-- Timer2.C: Timer1 interrupt, timer setting and capture
  | | | |-- Timer2.hex: Timer2 operation hex file
  | | |--Timer3
  | | | |-- Timer3.C: Timer1 interrupt, timer setting, capture and PWM3
  | | | |-- Timer3.hex: Timer3 operation hex file
  | | |--UART1
  | | | |-- UART1.C:
  | | | |-- UART1.hex:
  | | | |-- UART1_485.C: UART1 485 mode data transceiver
  | | | |-- UART1_485.hex:
  | | | |-- UART1_INT.C: UART1 interrupt mode data transmission and reception
  | | | |-- UART1.hex:
  | | | |-- UART1_Query.C: UART1 query mode data transmission and reception
  | | | |-- UART1.hex:
  | | |--WDOG
  | | | |-- WDOG.C: Watchdog operation
  | | | |-- WDOG.hex: Watchdog operation hex file
  | | |--XBUS
  | | | |-- XBUS.C: Directly and multiplex address mode to operate external RAM
  | | | |-- XBUS.hex: XBUS operation hex file
  | | |--IAP
  | | | |--CH559 IAP function instruction.doc
  | | | |--CH559IAP
  | | | | |-- CH559_IAP.uvproj: CH559 IAP project file
  | | | | |-- CH559_IAP.C: IAP function C program, support serial port and USB mode to upgrade user program
  | | | | |-- CH559_IAP.H: IAP function header file
  | | | | |-- CH559_IAP.hex: IAP function hex file
  | | | |--CH559User
  | | | | |-- CH559_User.uvproj: CH559 IAP user program project file
  | | | | |-- CH559_Demo.C: CH559 cooperates with IAP application, enter IAP by pressing the key
  | | | | |-- CH559_Demo.hex: User program hex file with IAP function
  | | | |--CH559WINUser
  | | | | |-- WCHMCUIAP_WINAPP.exe:CH559 PC tool with routine IAP, supports serial port and USB download
  | | |--USB
  | | | |-- DEVICE
  | | | | |-- Compound Dev.C: simulate keyboard and mouse routines, support some commands
  | | | | |-- Compound Dev.hex: Analog keyboard and mouse hex file
  | | | | |-- USBDEV.C: simulates the manufacturer's custom device, needs to install the CH372 driver, and can be debugged through 372test.exe (batch data sending and receiving)
  | | | | |-- USBDEV.hex: Simulate the hex file of the manufacturer's custom device
  | | | |-- HOST
  | | | | |-- USBHOST.C: Initialize and enumerate USB devices connected to DP/DM port in host mode ROOT-HUB, you can operate U disk or operate USB mouse, does not include HID command processing
  | | | | |-- USBHOST.hex:
  | | | | |-- USBH_HUB_KM.C: USB host application example, initialize and enumerate the devices connected to the USB port, support up to 2 USB devices at the same time, support an external HUB, can operate USB keyboard and mouse and HUB, including HID Class command processing
  | | | | |-- USBH_HUB_KM.hex:
  | | | | |-- USBH_KM.C: USB host application example, only supports the operation of HID devices such as keyboard and mouse hooked on ROOTHUB.
  | | | | |-- USBH_KM.hex:
  | | | | |-- USBH_AOA.C: USB host application example, support ROOTHUB to connect Android devices to communicate with APP.
  | | | | |-- USBH_AOA.hex:
  | | | | |-- TOOL
  | | | | | |-- UARTDemo.apk: AOA debugging supporting APK test file
  | | | |-- U_DISK
  | | | | | |-- EXAM1.C: C example source program, read and write files in bytes, including file creation, deletion, modification of file attributes, and modification of file names
  | | | | | |-- EXAM1.hex: Byte mode file read and write hex file
  | | | | | |-- EXAM6.C: C example source program, read and write files in units of sectors, including file creation, deletion, and modification of file attributes
  | | | | | |-- EXAM6.hex: Byte mode file read and write hex file
  | | | | | |-- EXAM9.C: C example source program, create a new folder, create files and subdirectories under the file
  | | | | | |-- EXAM9.hex: Create directory hex file
  | | | | | |-- EXAM11.C: C example source program, enumerate the root directory or files under the specified directory
  | | | | | |-- EXAM11.hex:
  | | |-- USB_LIB
  | | | | |-- CH559UFI.C: CH559 U Disk File System Library Common Interface Functions
  | | | | |-- CH559UFI.H: CH559 U disk file system library variable and function declaration header file
  | | | | |-- CH559UFI.LIB: CH559 U disk file system library file (compiler memory mode set to: small)
  | | | | |-- CH559UFX.LIB: U disk file system library file (compiler memory mode is set to: large)
  | | | | |-- CH559UF_README.TXT: CH559 U Disk File System Change Document
  | |-- PUB: Evaluation board description
  | | |-- CH559EVT.PDF: Evaluation board description
  | | |-- CH559SCH.PDF: Evaluation board schematic diagram
  | | |-- CH55X instruction cycle.PDF: CH55X assembly instruction cycle table
  | | |--- CH55X offline writer V130
  | | | | |--- Offline burner instructions. PDF
  | | | | |-- Structure diagram of offline burner. PDF
  | | | | |-- OffLineWCH_ISP configuration tool
  | | | | | |--Offline burner configuration tool.exe
  | | | | | | |--SV00xISPDLL.dll


2015-08-21 change description:
(1) Update DataFlash operation (read overflow defines buffer size).
(2). Correct the operation that does not point to the storage buffer during the EXAM1 file read operation.
(3) Revise FlashMaster.C SPI initialization repeated operation bit bS0_MST_CLK.
(4). Added the interface function of PWM pin mapping.
(5) Modify the problem that the Timer2.C file needs to be delayed after capturing, and the last sampled data will be lost without delay.
(6), 2015.06.27 CH559.H updated, changed the LED interface DMA data transmission mode, the original multi-channel quantile transmission was changed to the multi-channel byte mode,
The original 8 data channels are changed to 4 data channels, and up to 2 DMAs can be used to send data at the same time;
In addition, the watchdog timeout flag bit (read only) has been added.
(7) Modify the error sent in the UART1.C interrupt, the receiving length is self-decreasing, and the data length is 0 when self-transmitting and self-receiving.
(8) When modifying the CAP3 of the TIMER3.C file, the problem of not being able to obtain the first data without clearing the interrupt.
(9) Modify Mouse.C, the original program file has errors, use the same address host register to operate the device and the full speed\low speed setting problem, increase the simulation keyboard function, simulate the composite device, and support some types of commands.
(10). The schematic diagram of CH559MINIEVT is added to CH559SCH.PDF.
(11), EVT\PCB update, add CH559MINIEVT schematic diagram and PCB.
(12), add EXAM11 file, U disk file enumeration function.
(13). The XBUS.C file is modified. The original file has the problem of variable overflow.
(14) Delete LED4~LED7 used in the .C file, and increase the power-on delay 30ms operation for each .C file.
(15) The problem that the preset number is sent first when SPI0 slave mode data is sent is changed.
(16). EVT\PUB\CH559EVT.PDF is updated, the original instructions are adjusted, and the MINI board instructions and IAP function instructions are added.
(17). Delete the original LED control program and change it to a standard drive mode, which can read U disk display information, store it in Flash, and then read Flash display information.
(18) A 16-bit variable CH559vSectorSize has been added to EXAM6 to specify the actual sector size. Now the default is 512, which solves the problem of incorrect operation of CH559vSectorSizeH (default 2) using the high byte of the sector size.
(19). Add IAP function data package.


2015-11-26 change description:
(1) Schematic diagram changes, the matching resistance between XA and XB used for 485 is changed from 120Ω to 0Ω;
(2) UART1 routine separates 485 and TTL, highlighting the use of serial port FIFO;
(3) Add EXAM9, use CH559 to create new folders, folder file operations;

2015-12-28 change description:
(1) CH559DS1 release version 1B, the changes mainly include LED control instructions and main frequency;
(2) CH559.H: Fxt is changed from 4-13MHz to 4-20MHz; Fpll is changed from 24-320MHz to 24-350MHz; Fsys is changed from 1-60MHz to 1-57MHz;
(3) CH559.INC: Fxt is changed from 4-13MHz to 4-20MHz; Fpll is changed from 24-320MHz to 24-350MHz; Fsys is changed from 1-60MHz to 1-57MHz;
   typedef USB_SETUP_REQ xdata PXUSB_SETUP_REQ; changed to typedef USB_SETUP_REQ xdata *PXUSB_SETUP_REQ; corrected the error;
(4) CH559UFI.H: CH559_LIB_VER changed from 0x10 to 0x11;
(5). Modify the original UHUB0_CTRL to UDEV_CTRL register in the device initialization function in USBDEV.C. The two addresses are the same, but the former is used for the host and the latter is used for the device to avoid misunderstanding;
(6), CH559UFI(small)/CH559UFX(large) V1.1 latest update 2015.12.23
   1. Fix BUG
    1. CH559FileWrite failed to write root directory length limit
   1. Updates and additions
    1. Update DMA to support large sectors of 4096 bytes
(7) ADC sampling of ADCManual.C adds a channel selection array to prevent customer channel assignment errors;
(8) The DataFlash.C chip has improved DataFlash reading and writing, the old erasing and writing are compatible with the new version, and the new version will cause problems when operating the old film;
(9) Timer0.C Timer1.C Timer2.C changed the way of timer assignment;
(10). Added the example program Encryption.C for chip user program encryption and the documentation of general encryption methods;
(11). Added CH55X series MCU assembly instruction periodic table;

2016-2-18 change description:
(1) The main frequency of CH559DS1 is changed to 56MHz, and some pin descriptions and USB descriptions are clarified;
(2) The main frequency part involved in CH559.H and CH559.INC is changed to the highest 56MHz, and the MASK_LED_CHAN_MOD is changed from 0x80 to 0xC0;
(3) CH559UFI.H: CH559_LIB_VER changed from 0x11 to 0x12;
(4), CH559UFI(small)/CH559UFX(large) V1.2 latest update 2016.2.18
     1. Fix BUG
     1. CH559FileModify file date and time are wrong;
(5). Delete the two redundant example programs in USBHOST. Now it is changed to an example that only operates HID under ROOT_HUB and an example that supports operation of 2 devices under ROOT_HUB and HID under level 1 HUB, and supports HID commands ；
(6) Unify the file style and file structure, mainly operating the LIB library of the U disk to change a location, and call public files from the EXAM root directory;

2016-3-1 change description:
(1), Compound Dev.C modifies the judgment condition of NUM_LOCK;

2016-3-3 change description:
(1), GPIO.C modify how to configure a single IO state.
(2) Add #pragma NOAREGS to all .C files.

2016-3-22 change description:
(1), Compound Dev.C Note: The length of the buffer for receiving data >= min (the maximum packet length that may be received + 2 bytes, 64 bytes)
     USB bus reset adds UEP0/1/2/3_CTRL cleaning
     CH559 interrupt clearing, only supports bit-by-bit clearing, or writing 1 to clear
     Support standard request USB_CLEAR_FEATURE and USB_GET_STATUS and USB_SET_FEATURE

2016-6-24 change description:
(1) ADCManual.C adds a channel switching sampling function, and the first sampled value is taken out and discarded
(2) DataFlash.C modified the write and erase functions of DataFlash
(3) Compond_Dev.C and USBDEV.C add while(( UEP1_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK) when uploading data at endpoints 1 and 2; ensure that the DMA buffer is not read and written at the same time
(4) USBH_HUB_KM.C Before the modification, the HUB cannot be recognized when the device is inserted first, and it must be re-plugged to recognize the problem. The HUB operation part is corrected, and the USB printer part is added, and USB printer commands are not supported.
(5), EXAM.6 read only a few sectors of the file before the modification, read the entire file instead, emphasize how to read less than 1 sector
(6) Add the example of GETID.C to obtain the unique ID number of the chip
(7) Add CH55X_MINIEVT_V20.DDB, CH559&CH558 L and T package, CH557 L&Q package PCB, schematic diagram
(8), update CH559SCH.pdf
(9) EXAM1 adds functions to modify the file name, all U disk routines have changed from supporting HUB0 to HUB1 and HUB0
(10), add offline burner instructions and PCB under the PUB folder
(11). Modified the waiting time for port re-enablement after the device is connected to all USB host programs, from the original 100ms to 20-30ms

2016-8-25 change description:
(1) Instructions for use of CH55X offline burner.pdf Add USB connection debounce delay, USB download LED status change, D3 as connection status indicator, D4 as download start indicator

2016-11-29 Change description:
(1). Modify the BOM file of CH55X_MINIEVTV13, and change the serial port download port to 0 ohm consistent with the schematic.

2017-2-10 11:14
(1), modify the typos of CH559.H explanation.

20172017-3-6 16:40
(1). Modify the description of CH559EVT and CH559SCH.PDF.
(2) Modify the CH55X instruction cycle table, and add the description of the DB command and CH551/2/3/4.

2018-3-2 9:56
(1), modify CH559EVT.PDF.
(2). Delete the EVT/PCB folder.
(3) Modify the CH559 IAP function instruction.pdf and update it to version V1.2.
(4) Modify USBH_HUB_KM.C to increase the number of retries in the enumeration part to ensure that the USB device enumeration is successful.
(5), delete the EVT/PUB/CH55X offline programmer under CH55X_OFF_LINE_LOAD_V10.DDB
(6), add EVT/PUB/CH55X offline programmer/CH55X_OFF_LINE_LOAD_SCH.pdf

2018-08-01
(1). The USBH_KM.C file is added under the USB/Host directory, which is an example separated from USBH_HUB_KM.C and only used for keyboard and mouse operation.
(2) Modify the SelectHubPort function in the USBH_HUB_KM.C file to support the normal operation of low-speed devices under the HUB.
(3) Modify the main function code structure in the USBH_HUB_KM.C file, plug in the device to automatically operate without using getkey as a trigger.
(4) Delete part of the code for the HUB operation of the InitRootDevice function under the USBH_HUB_KM.C file.
(5) CH559EVT.PDF updates the V1.5 version, mainly adding the choice of CH559 device when creating a project.
(6) Add USBH_AOA.C file to support Android accessory mode under ROOTHUB.
(7). Delete the related files of the old version of the offline burner, and add the new version of the offline burner description, structure diagram and configuration software.

2019-7-22
1. Modify the SPI0_Master.c, SPI0_Slv.c, SPI1_Master.c files:
Host: send data, the data is incremented from 0 to 255, and detect whether the received data is the inverse of the sent data;
Slave: Receive data, and put the received data into SPI_DATA after inverting, waiting for the host to fetch data.
