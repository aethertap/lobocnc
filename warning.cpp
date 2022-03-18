//---------------------------------------------------------------------------
#include <vcl\vcl.h>
#pragma hdrstop

#include "warning.h"
#include "main.h"
//---------------------------------------------------------------------------
#pragma resource "*.dfm"
TWarningForm *WarningForm;
//---------------------------------------------------------------------------
__fastcall TWarningForm::TWarningForm(TComponent* Owner)
	: TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TWarningForm::OKButtonClick(TObject *Sender)
{
Close();
MainForm->StartupTimer->Enabled = true;
}
//---------------------------------------------------------------------------
void __fastcall TWarningForm::Button1Click(TObject *Sender)
{
MainForm->Close();        
}
//---------------------------------------------------------------------------
