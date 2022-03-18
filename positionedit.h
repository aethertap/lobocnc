//---------------------------------------------------------------------------

#ifndef positioneditH
#define positioneditH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
//---------------------------------------------------------------------------
class TPositionEditForm : public TForm
{
__published:	// IDE-managed Components
        TButton *OKButton;
        TButton *CancelButton;
        TEdit *PosEdit;
        TLabel *Label1;
        void __fastcall OKButtonClick(TObject *Sender);
        void __fastcall CancelButtonClick(TObject *Sender);
        void __fastcall FormShow(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TPositionEditForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TPositionEditForm *PositionEditForm;
//---------------------------------------------------------------------------
#endif
