//---------------------------------------------------------------------------
#ifndef homeH
#define homeH
//---------------------------------------------------------------------------
#include <vcl\Classes.hpp>
#include <vcl\Controls.hpp>
#include <vcl\StdCtrls.hpp>
#include <vcl\Forms.hpp>
#include <vcl\ExtCtrls.hpp>
//---------------------------------------------------------------------------
class THomeForm : public TForm
{
__published:	// IDE-managed Components
	TPanel *Panel1;
	TLabel *Label1;
	TLabel *Label2;
private:	// User declarations
public:		// User declarations
	__fastcall THomeForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern THomeForm *HomeForm;
//---------------------------------------------------------------------------
#endif
