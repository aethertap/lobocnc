//---------------------------------------------------------------------------
#ifndef warningH
#define warningH
//---------------------------------------------------------------------------
#include <vcl\Classes.hpp>
#include <vcl\Controls.hpp>
#include <vcl\StdCtrls.hpp>
#include <vcl\Forms.hpp>
//---------------------------------------------------------------------------
class TWarningForm : public TForm
{
__published:	// IDE-managed Components
	TMemo *WarningMemo;
	TButton *OKButton;
        TButton *Button1;
	void __fastcall OKButtonClick(TObject *Sender);
        void __fastcall Button1Click(TObject *Sender);
private:	// User declarations
public:		// User declarations
	__fastcall TWarningForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern TWarningForm *WarningForm;
//---------------------------------------------------------------------------
#endif
