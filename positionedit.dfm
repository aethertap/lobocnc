object PositionEditForm: TPositionEditForm
  Left = 666
  Top = 423
  BorderIcons = []
  BorderStyle = bsDialog
  ClientHeight = 63
  ClientWidth = 148
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  Position = poMainFormCenter
  OnShow = FormShow
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: TLabel
    Left = 8
    Top = 12
    Width = 64
    Height = 13
    Caption = 'New posiiton:'
  end
  object OKButton: TButton
    Left = 80
    Top = 36
    Width = 61
    Height = 21
    Caption = 'OK'
    TabOrder = 0
    OnClick = OKButtonClick
  end
  object CancelButton: TButton
    Left = 8
    Top = 36
    Width = 61
    Height = 21
    Caption = 'Cancel'
    TabOrder = 1
    OnClick = CancelButtonClick
  end
  object PosEdit: TEdit
    Left = 76
    Top = 8
    Width = 65
    Height = 21
    TabOrder = 2
    Text = 'PosEdit'
  end
end
