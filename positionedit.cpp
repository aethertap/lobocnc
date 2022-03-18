//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "positionedit.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TPositionEditForm *PositionEditForm;
//---------------------------------------------------------------------------
__fastcall TPositionEditForm::TPositionEditForm(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TPositionEditForm::OKButtonClick(TObject *Sender)
{
ModalResult = 1;
}
//---------------------------------------------------------------------------
void __fastcall TPositionEditForm::CancelButtonClick(TObject *Sender)
{
ModalResult = -1;
}
//---------------------------------------------------------------------------
void __fastcall TPositionEditForm::FormShow(TObject *Sender)
{
PosEdit->SetFocus();        
}
//---------------------------------------------------------------------------
