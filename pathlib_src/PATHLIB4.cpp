//---------------------------------------------------------------------------
#include <vcl\vcl.h>
#pragma hdrstop
//---------------------------------------------------------------------------
//   Important note about DLL memory management:
//
//   If your DLL exports any functions that pass String objects (or structs/
//   classes containing nested Strings) as parameter or function results,
//   you will need to add the library BCBMM.LIB to both the DLL project and any
//   EXE projects that use the DLL.  This will change the DLL and its calling
//   EXE's to use the BCBMM.DLL as their memory manager.  In these cases,
//   the file BCBMM.DLL should be deployed along with your DLL.
//
//   To avoid using BCBMM.DLL, pass string information using "char *" or
//   ShortString parameters.
//---------------------------------------------------------------------------
USERES("PATHLIB4.res");
USEUNIT("path.cpp");
USELIB("nmclib04.lib");
//---------------------------------------------------------------------------
int WINAPI DllEntryPoint(HINSTANCE hinst, unsigned long reason, void*)
{
	return 1;
}
//---------------------------------------------------------------------------
