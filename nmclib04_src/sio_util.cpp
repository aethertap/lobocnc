//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#include <vcl\vcl.h>
#include <stdio.h>
#pragma hdrstop

#include "sio_util.h"
//---------------------------------------------------------------------------//
// Global variables                                                          //
//---------------------------------------------------------------------------//
int printerrors = 1;


//---------------------------------------------------------------------------//
//  Function Name:  SimpleMsgBox (Internal Library Function)                 //
//  Return Value:   0 on failure, non-zero on success                        //
//  Parameters:     msgstr: pointer to null terminated string                //
//  Description:    Displays a simple Windows message box.                   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) int SimpleMsgBox(char *msgstr)
{
return MessageBox(NULL, msgstr,"",MB_TASKMODAL | MB_SETFOREGROUND);
}


//---------------------------------------------------------------------------//
//  Function Name:  ErrorPrinting (Internal Library Function)                //
//  Return Value:   None                                                     //
//  Parameters:     f: 0=disable error printing, 1=enable error printing     //
//  Description:    Controls whether low-level error messages printed by     //
//                  ErrorMsgBox() are printed or suppressed.                 //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) void ErrorPrinting(int f)
{
printerrors = f;
}


//---------------------------------------------------------------------------//
//  Function Name:  ErrorMsgBox (Internal Library Function)                  //
//  Return Value:   0 on failure or printing disabled, non-zero on success   //
//  Parameters:     msgstr: pointer to null terminated string                //
//  Description:    Displays a Windows message box for error messages under  //
//                  display control using ErrorPrinting().                   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) int ErrorMsgBox(char *msgstr)
{
if (printerrors)
  return MessageBox(NULL, msgstr,"",MB_TASKMODAL | MB_SETFOREGROUND);
else return(0);
}


//---------------------------------------------------------------------------//
//  Function Name:  SioOpen (Internal Library Function)                      //
//  Return Value:   Returns a handle to a COM port stream                    //
//  Parameters:     portname:  name of COM port ("COMn:", where n=1-8)       //
//                  baudrate: 9600,19200,38400,57600,115200,230400           //
//  Description:    Opens a COM port at the specified baud rate.  Set up     //
//                  read and write timeouts.                                 //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) HANDLE SioOpen(char *name, unsigned int baudrate)
{
BOOL RetStat;
COMMCONFIG cc;
COMMTIMEOUTS ct;
HANDLE ComHandle;
DWORD winrate;
char msgstr[50];

//Open COM port as a file
ComHandle = CreateFile(name, GENERIC_READ | GENERIC_WRITE ,0, NULL,
			           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL , NULL);


while (TRUE)
	{
	if (ComHandle == INVALID_HANDLE_VALUE)
		{
        sprintf(msgstr,"%s failed to open",name);
		ErrorMsgBox(msgstr);
        break;
    	}

    switch (baudrate) {
		case 9600: 		winrate = CBR_9600; break;
		case 19200: 	winrate = CBR_19200; break;
		case 38400: 	winrate = CBR_38400; break;
		case 57600: 	winrate = CBR_57600; break;
		case 115200: 	winrate = CBR_115200; break;
		case 230400: 	winrate = 230400; break;
    	default:		ErrorMsgBox("Baud rate not supported - using default of 19200");
    					winrate = CBR_19200;
    	}

    //Fill in COM port config. structure & set config.
    cc.dwSize = sizeof(DCB) + sizeof(WCHAR) + 20;
    cc.wVersion = 1;

    cc.dcb.DCBlength = sizeof(DCB);
    cc.dcb.BaudRate = winrate;
    cc.dcb.fBinary = 1;
    cc.dcb.fParity = 0;
    cc.dcb.fOutxCtsFlow = 0;
    cc.dcb.fOutxDsrFlow = 0;
    cc.dcb.fDtrControl = DTR_CONTROL_DISABLE;
    cc.dcb.fDsrSensitivity = 0;
    cc.dcb.fTXContinueOnXoff = 0;
    cc.dcb.fOutX = 0;
    cc.dcb.fInX = 0;
    cc.dcb.fErrorChar = 0;
    cc.dcb.fNull = 0;
    cc.dcb.fRtsControl = RTS_CONTROL_DISABLE;
    cc.dcb.fAbortOnError = 0;
    cc.dcb.XonLim = 100;
    cc.dcb.XoffLim = 100;
    cc.dcb.ByteSize = 8;
    cc.dcb.Parity = NOPARITY;
    cc.dcb.StopBits = ONESTOPBIT;
    cc.dcb.XonChar = 'x';
    cc.dcb.XoffChar = 'y';
    cc.dcb.ErrorChar = 0;
    cc.dcb.EofChar = 0;
    cc.dcb.EvtChar = 0;

    cc.dwProviderSubType = PST_RS232;
    cc.dwProviderOffset = 0;
    cc.dwProviderSize = 0;

    RetStat = SetCommConfig(ComHandle, &cc, sizeof(cc));
	if (RetStat == 0)
    	{
    	ErrorMsgBox("Failed to set COMM configuration");
        break;
        }

    //Set read/write timeout values for the file
    ct.ReadIntervalTimeout = 0;  		//ignore interval timing
    ct.ReadTotalTimeoutMultiplier = 2; 	//2 msec per char
    ct.ReadTotalTimeoutConstant = 50;  		//plus add'l 50 msec
    ct.WriteTotalTimeoutMultiplier = 2;	//Set max time per char written
    ct.WriteTotalTimeoutConstant = 50;	//plus additional time

	RetStat = SetCommTimeouts(ComHandle, &ct);
    if (RetStat == 0)
    	{
        ErrorMsgBox("Failed to set Comm timeouts");
        break;
        }

    break;
	}

return(ComHandle);
}


//---------------------------------------------------------------------------//
//  Function Name:  SioChangeBaud (Internal Library Function)                //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     ComPort: COM port handle                                 //
//                  baudrate: 9600,19200,38400,57600,115200,230400           //
//  Description:    Change the baud rate to the specified values.            //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL SioChangeBaud(HANDLE ComPort, unsigned int baudrate)
{
BOOL RetStat;
DWORD winrate;
DCB cs;

RetStat = GetCommState(ComPort, &cs);
if (RetStat == false) return RetStat;
switch (baudrate) {
	case 9600: 		winrate = CBR_9600; break;
	case 19200: 	winrate = CBR_19200; break;
	case 38400: 	winrate = CBR_38400; break;
	case 57600: 	winrate = CBR_57600; break;
	case 115200: 	winrate = CBR_115200; break;
	case 230400: 	winrate = 230400; break;
    default:		ErrorMsgBox("Baud rate not supported");
    				return false;
    }
cs.BaudRate = winrate;
RetStat = SetCommState(ComPort, &cs);
if (RetStat == false) return RetStat;
return true;
}


//---------------------------------------------------------------------------//
//  Function Name:  SioPutChars (Internal Library Function)                  //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     ComPort: COM port handle                                 //
//                  stuff: array containing character data to send           //
//                  n: number of characters to send                          //
//  Description:    Write out n chars to the ComPort, returns only after     //
//                  chars have been sent.                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL SioPutChars(HANDLE ComPort, char *stuff, int n)
{
BOOL RetStat;
DWORD nums;

RetStat = WriteFile(ComPort, stuff,n, &nums, NULL);
if (RetStat == 0) ErrorMsgBox("SioPutChars failed");
return RetStat;
}


//---------------------------------------------------------------------------//
//  Function Name:  SioGetChars (Internal Library Function)                  //
//  Return Value:   Returns the number of characters actually read           //
//  Parameters:     ComPort: COM port handle                                 //
//                  stuff: array to store characters read                    //
//                  n: number of characters to read                          //
//  Description:    Read n chars into the array stuff.                       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) DWORD SioGetChars(HANDLE ComPort, char *stuff, int n)
{
BOOL RetStat;
DWORD numread;

RetStat = ReadFile(ComPort, stuff, n, &numread, NULL);
if (RetStat == 0) ErrorMsgBox("SioReadChars failed");

return numread;
}


//---------------------------------------------------------------------------//
//  Function Name:  SioTest (Internal Library Function)                      //
//  Return Value:   Returns the number of characters in ComPort's input buf  //
//  Parameters:     ComPort: COM port handle                                 //
//  Description:    Returns the number of chars in a port's input buffer.    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) DWORD SioTest(HANDLE ComPort)
{
COMSTAT cs;
DWORD Errors;
BOOL RetStat;

RetStat = ClearCommError(ComPort, &Errors, &cs);
if (RetStat == 0) ErrorMsgBox("SioTest failed");
return cs.cbInQue;
}


//---------------------------------------------------------------------------//
//  Function Name:  SioClrInBuf (Internal Library Function)                  //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     ComPort: COM port handle                                 //
//  Description:    Purge all chars from a port's input buffer.              //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL SioClrInbuf(HANDLE ComPort)
{
BOOL RetStat;

RetStat = PurgeComm(ComPort, PURGE_RXCLEAR);
if (RetStat == 0) ErrorMsgBox("SioClrInbuf failed");

return RetStat;
}


//---------------------------------------------------------------------------//
//  Function Name:  SioClose (Internal Library Function)                     //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     ComPort: COM port handle                                 //
//  Description:    Close a previously opened COM port.                      //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL SioClose(HANDLE ComPort)
{
return(CloseHandle(ComPort));
}
//---------------------------------------------------------------------------

