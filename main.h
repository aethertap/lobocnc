//---------------------------------------------------------------------------
#ifndef mainH
#define mainH
//---------------------------------------------------------------------------
#include <vcl\Classes.hpp>
#include <vcl\Controls.hpp>
#include <vcl\StdCtrls.hpp>
#include <vcl\Forms.hpp>
#include <vcl\Menus.hpp>
#include <vcl\ExtCtrls.hpp>
#include <vcl\ComCtrls.hpp>
#include <vcl\Buttons.hpp>
#include <vcl\Dialogs.hpp>
//---------------------------------------------------------------------------
class TMainForm : public TForm
{
__published:	// IDE-managed Components
	TBevel *Bevel1;
	TTimer *PathTimer;
	TGroupBox *GroupBox1;
	TRadioButton *MachineRB;
	TRadioButton *ProgramRB;
	TEdit *XPosEdit;
	TLabel *Label1;
	TEdit *YPosEdit;
	TEdit *ZPosEdit;
	TLabel *Label2;
	TLabel *Label3;
	TButton *XOrgButton;
	TButton *YOrgButton;
	TButton *ZOrgButton;
	TButton *SetOrgButton;
	TButton *AllOnButton;
	TButton *AllOffButton;
	TGroupBox *JogGroupBox;
	TButton *YPlusButton;
	TButton *YMinusButton;
	TButton *XMinusButton;
	TButton *XPlusButton;
	TButton *ZPlusButton;
	TButton *ZMinusButton;
	TTrackBar *JogTrackBar;
	TLabel *Label4;
	TLabel *Label5;
	TCheckBox *XOnCB;
	TCheckBox *YOnCB;
	TCheckBox *ZOnCB;
	TTimer *StartupTimer;
	TUpDown *RapidUD;
	TEdit *RapidEdit;
	TLabel *Label6;
	TLabel *Label7;
	TUpDown *FeedrateUD;
	TEdit *FeedrateEdit;
	TLabel *Label8;
	TLabel *Label9;
	TEdit *ImmediateEdit;
	TButton *ExecuteButton;
	TButton *FeedholdButton;
	TButton *ResumeButton;
	TButton *OpenButton;
	TButton *ResetButton;
	TBitBtn *BitBtn1;
	TBitBtn *HelpButton;
	TButton *StartButton;
	TOpenDialog *OpenDialog;
	TListBox *GList;
	TButton *StepButton;
	TButton *StopButton;
	TLabel *Label10;
	TLabel *Label11;
	TEdit *ToolNumEdit;
	TEdit *ToolLenEdit;
	TCheckBox *ContouringCB;
	TButton *AbortButton;
	
	
	void __fastcall PathTimerTimer(TObject *Sender);
	void __fastcall ExitButtonClick(TObject *Sender);

	
	void __fastcall StartupTimerTimer(TObject *Sender);
	void __fastcall XOnCBClick(TObject *Sender);
	void __fastcall YOnCBClick(TObject *Sender);
	void __fastcall ZOnCBClick(TObject *Sender);
	void __fastcall AllOnButtonClick(TObject *Sender);
	void __fastcall AllOffButtonClick(TObject *Sender);
	void __fastcall FormClose(TObject *Sender, TCloseAction &Action);
	void __fastcall XPlusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	
	void __fastcall XPlusButtonMouseUp(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall XMinusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	
	void __fastcall YPlusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall YMinusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall YPlusButtonMouseUp(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	
	void __fastcall ZPlusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall ZMinusButtonMouseDown(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall ZPlusButtonMouseUp(TObject *Sender, TMouseButton Button,
	TShiftState Shift, int X, int Y);
	void __fastcall XOrgButtonClick(TObject *Sender);
	void __fastcall YOrgButtonClick(TObject *Sender);
	void __fastcall ZOrgButtonClick(TObject *Sender);
	void __fastcall SetOrgButtonClick(TObject *Sender);
	void __fastcall FeedrateUDClick(TObject *Sender, TUDBtnType Button);
	
	void __fastcall BitBtn1Click(TObject *Sender);
	void __fastcall OpenButtonClick(TObject *Sender);

	void __fastcall RapidUDClick(TObject *Sender, TUDBtnType Button);
	void __fastcall ExecuteButtonClick(TObject *Sender);
	
	
	void __fastcall ImmediateEditEnter(TObject *Sender);
	void __fastcall ImmediateEditClick(TObject *Sender);
	void __fastcall ResetButtonClick(TObject *Sender);
	void __fastcall StepButtonClick(TObject *Sender);
	void __fastcall StopButtonClick(TObject *Sender);
	void __fastcall StartButtonClick(TObject *Sender);
	void __fastcall FeedholdButtonClick(TObject *Sender);
	void __fastcall ResumeButtonClick(TObject *Sender);
	
	void __fastcall AbortButtonClick(TObject *Sender);
	void __fastcall HelpButtonClick(TObject *Sender);
        void __fastcall XPosEditClick(TObject *Sender);
        void __fastcall YPosEditClick(TObject *Sender);
        void __fastcall ZPosEditClick(TObject *Sender);
	
private:	// User declarations
public:		// User declarations
	__fastcall TMainForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern TMainForm *MainForm;
//---------------------------------------------------------------------------
//Function Prototypes
int ReadGCodeFile(char *name);
int ExecuteGCode(char *codestr, int append);
int StartRapid(float x, float y, float z);
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#endif
