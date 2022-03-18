//---------------------------------------------------------------------------
#include <vcl\vcl.h>
#pragma hdrstop
//---------------------------------------------------------------------------
USEFORM("main.cpp", MainForm);
USERES("PSCNC.res");
USEFORM("warning.cpp", WarningForm);
USEFORM("home.cpp", HomeForm);
USELIB("nmclib04.lib");
USELIB("pathlib4.lib");
USEFORM("positionedit.cpp", PositionEditForm);
//---------------------------------------------------------------------------
WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
	try
	{
		Application->Initialize();
		Application->CreateForm(__classid(TMainForm), &MainForm);
                 Application->CreateForm(__classid(TWarningForm), &WarningForm);
                 Application->CreateForm(__classid(THomeForm), &HomeForm);
                 Application->CreateForm(__classid(TPositionEditForm), &PositionEditForm);
                 Application->Run();
	}
	catch (Exception &exception)
	{
		Application->ShowException(&exception);
	}
	return 0;
}
//---------------------------------------------------------------------------
