//---------------------------------------------------------------------------
//Revisions:
// 1/1/13: original release
// 8/25/13:
//      1. Pressing the space bar (or Enter) executes a Feedhold
//      2. Clicking on the X, Y or Z coordinate value lets you set current position
//      3. Added R-mode arcs: R<0.0 invalid, if R <= 1/2 chord len., 180 deg. arc
//      4. G-code filename is displayed at top of window.
//
//---------------------------------------------------------------------------

#include <vcl\vcl.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#pragma hdrstop

#include "warning.h"
#include "home.h"
#include "positionedit.h"
#include "main.h"
#include "path.h"
#include "sio_util.h"
#include "nmccom.h"
#include "picservo.h"
#include "globals.h"
//---------------------------------------------------------------------------
#pragma resource "*.dfm"
TMainForm *MainForm;
//---------------------------------------------------------------------------
//Defines:
#define MAXTOOLS 20

#define PH_PWM 170

//Control modes
#define SERVO_OFF -1      //servos turned off
#define IDLE 0            //motors idle
#define PATH 1            //path mode with path still loading
#define PATH_END 2        //tail end of already loaded path
#define RAPID 3           //trapezoidal move in progress
#define JOG 4             //jog in progress
#define JOG_STOP 5		  //decel to a stop from a jog
#define JOGXP 6			  //jog in X plus direction
#define JOGXM 7			  //jog in X minus direction
#define JOGYP 8			  //jog in Y plus direction
#define JOGYM 9			  //jog in Y minus direction
#define JOGZP 10		  //jog in Z plus direction
#define JOGZM 11		  //jog in Z minus direction

//G-Code file limits
#define MAXCHARS 2500000
#define MAXLINES 100000

//Methods for defining an arc
#define ARC_NONE 	0
#define ARC_R 		1
#define ARC_IJK 	2

//Feedhold states
#define NO_HOLD 0
#define PATHHOLD 1
#define RAPIDHOLD 2

//---------------------------------------------------------------------------
//System control Globals:
int nummod = 0;
char comport[10];
byte xaxis, yaxis, zaxis;
int nbuf, pfreq;
double xacc, yacc, zacc;
double xscale, yscale, zscale;
double xmaxvel, ymaxvel, zmaxvel, maxfr;
double xmin, ymin, zmin;
double xmax, ymax, zmax;
double xorg, yorg, zorg;
GAINVECT xgain, ygain, zgain;
double feedrate = 0.05;  //default feedrate in units/sec
double accel;
double maxang;
int rapidoverride = 100;
int froverride = 100;
byte xpadv, ypadv, zpadv;
byte xpoff, ypoff, zpoff;
double toollen[MAXTOOLS];
int toolnum = 0;
int toollennum = 0;
int toolchanged = 0;
int dwellcounts = 0;
int opmode = SERVO_OFF;
int rungcode = 0;  		//flag to start interpreting G-Code
int feedhold = 0;
int contouring = 0;
int controlsenabled = 0;
double prevrx, prevry, prevrz;
int mill_homed = 0;
char formatstr[20] = "%8.4f";
unsigned char xiotype, yiotype, ziotype;


//G-Code storage globals:
char gcode[MAXCHARS];    //gcode buffer
int line[MAXLINES];		//index to individual lines
int numlines = 0;
int curline;

//Interpreter globals:
int pathappendmode = 0;  //says if we are in the middle of loading segments
double arcnormx = 0.0;    //normal vector for arcs
double arcnormy = 0.0;
double arcnormz = 1.0;
double gx = 0.0;          //interpreter x, y, z command position
double gy = 0.0;
double gz = 0.0;
double oldgx = 0.0;          //prev x, y, z command position
double oldgy = 0.0;
double oldgz = 0.0;
double cmdx = 0.0;   		//current pic-servo command positions
double cmdy = 0.0;
double cmdz = 0.0;
int arcmode = ARC_NONE;
double arci = 0.0;		 //arc center coordinates
double arcj = 0.0;
double arck = 0.0;
double arcr = 0.0;
double dwelltime = 0.0;   	//dwell parameter
double fripm;		     	//feedrate in inches per minute
int mcode = -1;             //current m code
int g0code = -1;            //current group 0 gcode
int g1code = -1;            //current group 1 gcode
int oldg1code = -1;         //previous group 1 gcode
int g2code = -1;            //current group 2 gcode
int g8code = -1;            //current group 8 gcode
int ptoolnum = 0;			//pending tool number
int ptoollennum = 0; 		//pending tool length number
int dgz = 0;				//flags a change in the z coord. in the cur. line

//---------------------------------------------------------------------------
//Usage: n = GetInt(FooEdit);
int GetInt(TEdit *PEdit)
{
  int i;
  char msgstr[40];

  PEdit->GetTextBuf(msgstr, 40);
  sscanf(msgstr,"%d", &i);
  return(i);
}
//---------------------------------------------------------------------------
double GetFloat(TEdit *PEdit)
{
  double x;
  char msgstr[40];

  PEdit->GetTextBuf(msgstr, 40);
  sscanf(msgstr,"%lf", &x);
  return(x);
}
//---------------------------------------------------------------------------
//Returns the magnitude of a floating point vector
double mag(double x, double y, double z)
{
  return(sqrt(x*x + y*y + z*z));
}
//---------------------------------------------------------------------------
//Returns the dot product of two floating point vectors
double dot(double Ax, double Ay, double Az, double Bx, double By, double Bz)
{
  return( Ax*Bx + Ay*By + Az*Bz );
}
//---------------------------------------------------------------------------
//C is returned as the cross product of (A cross B)
void cross(double Ax, double Ay, double Az, double Bx, double By, double Bz,
           double *Cx, double *Cy, double *Cz)
{
  *Cx = Ay*Bz - Az*By;
  *Cy = Az*Bx - Ax*Bz;
  *Cz = Ax*By - Ay*Bx;
}
//---------------------------------------------------------------------------
//Normalize a vector
double normalize(double *x, double *y, double *z)
{
  double a;

  a = mag(*x, *y, *z);
  if (a==0.0) return(a);

  *x /= a;
  *y /= a;
  *z /= a;
  return(a);
}
//---------------------------------------------------------------------------
//Returns -1 if PSCNC.TLL not found, -2 if not all parameters set, 0 on success
int ReadToolFile()
{
  int i;
  char param[20];  //tool name
  FILE *toolfile;


  if ( (toolfile = fopen("pscnc.tll", "r")) == NULL )
    {
      SimpleMsgBox("Could not find PSCNC.TLL");
      return(-1);
    }

  for (i=0; i<MAXTOOLS; i++)
    if ( fscanf(toolfile,"%s%lf",param, &(toollen[i])) != 2 )
      {
        fclose(toolfile);
        return(-2);
      }

  fclose(toolfile);
  return(0);
}
//---------------------------------------------------------------------------
//Returns -1 if PSCNC.INI not found, -2 if not all parameters set, 0 on success
int ReadIniFile()
{
  __int64 parambits;
  int scanres;
  char param[20];  //parameter name
  FILE *inifile;

  parambits = 0;   //say no parameters have been set

  if ( (inifile = fopen("pscnc.ini", "r")) == NULL )
    {
      SimpleMsgBox("Could not find PSCNC.INI");
      return(-1);
    }

  while (1)
    {
      scanres = fscanf(inifile, "%s", param);     //read in a parameter name
      if (scanres == EOF  || scanres == 0) break;

      if (!strcmp(param, "xaxis:"))
        {
          if ( fscanf(inifile, "%d", &xaxis) != 1 ) break;
          parambits |= 0x0000000000000001;
        }
      else if (!strcmp(param, "yaxis:"))
        {
          if ( fscanf(inifile, "%d", &yaxis) != 1 ) break;
          parambits |= 0x0000000000000002;
        }
      else if (!strcmp(param, "zaxis:"))
        {
          if ( fscanf(inifile, "%d", &zaxis) != 1 ) break;
          parambits |= 0x0000000000000004;
        }
      else if (!strcmp(param, "bufsize:"))
        {
          if ( fscanf(inifile, "%d", &nbuf) != 1 ) break;
          parambits |= 0x0000000000000008;
        }
      else if (!strcmp(param, "pfreq:"))
        {
          if ( fscanf(inifile, "%d", &pfreq) != 1 ) break;
          parambits |= 0x0000000000000010;
        }
      else if (!strcmp(param, "xscale:"))
        {
          if ( fscanf(inifile, "%lf", &xscale) != 1 ) break;
          parambits |= 0x0000000000000020;
        }
      else if (!strcmp(param, "yscale:"))
        {
          if ( fscanf(inifile, "%lf", &yscale) != 1 ) break;
          parambits |= 0x0000000000000040;
        }
      else if (!strcmp(param, "zscale:"))
        {
          if ( fscanf(inifile, "%lf", &zscale) != 1 ) break;
          parambits |= 0x0000000000000080;
        }
      else if (!strcmp(param, "xaccel:"))
        {
          if ( fscanf(inifile, "%lf", &xacc) != 1 ) break;
          parambits |= 0x0000000000000100;
        }
      else if (!strcmp(param, "yaccel:"))
        {
          if ( fscanf(inifile, "%lf", &yacc) != 1 ) break;
          parambits |= 0x0000000000000200;
        }
      else if (!strcmp(param, "zaccel:"))
        {
          if ( fscanf(inifile, "%lf", &zacc) != 1 ) break;
          parambits |= 0x0000000000000400;
        }
      else if (!strcmp(param, "xmaxvel:"))
        {
          if ( fscanf(inifile, "%lf", &xmaxvel) != 1 ) break;
          parambits |= 0x0000000000000800;
        }
      else if (!strcmp(param, "ymaxvel:"))
        {
          if ( fscanf(inifile, "%lf", &ymaxvel) != 1 ) break;
          parambits |= 0x0000000000001000;
        }
      else if (!strcmp(param, "zmaxvel:"))
        {
          if ( fscanf(inifile, "%lf", &zmaxvel) != 1 ) break;
          parambits |= 0x0000000000002000;
        }
      else if (!strcmp(param, "xmin:"))
        {
          if ( fscanf(inifile, "%lf", &xmin) != 1 ) break;
          parambits |= 0x0000000000004000;
        }
      else if (!strcmp(param, "ymin:"))
        {
          if ( fscanf(inifile, "%lf", &ymin) != 1 ) break;
          parambits |= 0x0000000000008000;
        }
      else if (!strcmp(param, "zmin:"))
        {
          if ( fscanf(inifile, "%lf", &zmin) != 1 ) break;
          parambits |= 0x0000000000010000;
        }
      else if (!strcmp(param, "xmax:"))
        {
          if ( fscanf(inifile, "%lf", &xmax) != 1 ) break;
          parambits |= 0x0000000000020000;
        }
      else if (!strcmp(param, "ymax:"))
        {
          if ( fscanf(inifile, "%lf", &ymax) != 1 ) break;
          parambits |= 0x0000000000040000;
        }
      else if (!strcmp(param, "zmax:"))
        {
          if ( fscanf(inifile, "%lf", &zmax) != 1 ) break;
          parambits |= 0x0000000000080000;
        }

      else if (!strcmp(param, "xkp:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.kp)) != 1 ) break;
          parambits |= 0x0000000000100000;
        }
      else if (!strcmp(param, "xkd:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.kd)) != 1 ) break;
          parambits |= 0x0000000000200000;
        }
      else if (!strcmp(param, "xki:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.ki)) != 1 ) break;
          parambits |= 0x0000000000400000;
        }
      else if (!strcmp(param, "xil:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.il)) != 1 ) break;
          parambits |= 0x0000000000800000;
        }
      else if (!strcmp(param, "xol:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.ol)) != 1 ) break;
          parambits |= 0x0000000001000000;
        }
      else if (!strcmp(param, "xcl:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.cl)) != 1 ) break;
          parambits |= 0x0000000002000000;
        }
      else if (!strcmp(param, "xel:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.el)) != 1 ) break;
          parambits |= 0x0000000004000000;
        }
      else if (!strcmp(param, "xsr:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.sr)) != 1 ) break;
          parambits |= 0x0000000008000000;
        }
      else if (!strcmp(param, "xdc:"))
        {
          if ( fscanf(inifile, "%d", &(xgain.dc)) != 1 ) break;
          parambits |= 0x0000000010000000;
        }
      else if (!strcmp(param, "xpadv:"))
        {
          if ( fscanf(inifile, "%d", &xpadv) != 1 ) break;
          parambits |= 0x0000000020000000;
        }
      else if (!strcmp(param, "xpoff:"))
        {
          if ( fscanf(inifile, "%d", &xpoff) != 1 ) break;
          parambits |= 0x0000000040000000;
        }
      else if (!strcmp(param, "ykp:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.kp)) != 1 ) break;
          parambits |= 0x0000000080000000;
        }
      else if (!strcmp(param, "ykd:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.kd)) != 1 ) break;
          parambits |= 0x0000000100000000;
        }
      else if (!strcmp(param, "yki:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.ki)) != 1 ) break;
          parambits |= 0x0000000200000000;
        }
      else if (!strcmp(param, "yil:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.il)) != 1 ) break;
          parambits |= 0x0000000400000000;
        }
      else if (!strcmp(param, "yol:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.ol)) != 1 ) break;
          parambits |= 0x0000000800000000;
        }
      else if (!strcmp(param, "ycl:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.cl)) != 1 ) break;
          parambits |= 0x0000001000000000;
        }
      else if (!strcmp(param, "yel:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.el)) != 1 ) break;
          parambits |= 0x0000002000000000;
        }
      else if (!strcmp(param, "ysr:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.sr)) != 1 ) break;
          parambits |= 0x0000004000000000;
        }
      else if (!strcmp(param, "ydc:"))
        {
          if ( fscanf(inifile, "%d", &(ygain.dc)) != 1 ) break;
          parambits |= 0x0000008000000000;
        }
      else if (!strcmp(param, "ypadv:"))
        {
          if ( fscanf(inifile, "%d", &ypadv) != 1 ) break;
          parambits |= 0x0000010000000000;
        }
      else if (!strcmp(param, "ypoff:"))
        {
          if ( fscanf(inifile, "%d", &ypoff) != 1 ) break;
          parambits |= 0x0000020040000000;
        }
      else if (!strcmp(param, "zkp:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.kp)) != 1 ) break;
          parambits |= 0x0000040000000000;
        }
      else if (!strcmp(param, "zkd:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.kd)) != 1 ) break;
          parambits |= 0x0000080000000000;
        }
      else if (!strcmp(param, "zki:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.ki)) != 1 ) break;
          parambits |= 0x0000100000000000;
        }
      else if (!strcmp(param, "zil:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.il)) != 1 ) break;
          parambits |= 0x0000200000000000;
        }
      else if (!strcmp(param, "zol:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.ol)) != 1 ) break;
          parambits |= 0x0000400000000000;
        }
      else if (!strcmp(param, "zcl:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.cl)) != 1 ) break;
          parambits |= 0x0000800000000000;
        }
      else if (!strcmp(param, "zel:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.el)) != 1 ) break;
          parambits |= 0x0001000000000000;
        }
      else if (!strcmp(param, "zsr:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.sr)) != 1 ) break;
          parambits |= 0x0002000000000000;
        }
      else if (!strcmp(param, "zdc:"))
        {
          if ( fscanf(inifile, "%d", &(zgain.dc)) != 1 ) break;
          parambits |= 0x0004000000000000;
        }
      else if (!strcmp(param, "zpadv:"))
        {
          if ( fscanf(inifile, "%d", &zpadv) != 1 ) break;
          parambits |= 0x0008000000000000;
        }
      else if (!strcmp(param, "zpoff:"))
        {
          if ( fscanf(inifile, "%d", &zpoff) != 1 ) break;
          parambits |= 0x0010000000000000;
        }
      else if (!strcmp(param, "maxang:"))
        {
          if ( fscanf(inifile, "%lf", &maxang) != 1 ) break;
          parambits |= 0x0020000000000000;
        }
      else if (!strcmp(param, "comport:"))
        {
          if ( fscanf(inifile, "%s", &comport) != 1 ) break;
          parambits |= 0x0040000000000000;
        }
      else if (!strcmp(param, "xiotype:"))
        {
          if ( fscanf(inifile, "%d", &xiotype) != 1 ) break;
          parambits |= 0x0080000000000000;
        }
      else if (!strcmp(param, "yiotype:"))
        {
          if ( fscanf(inifile, "%d", &yiotype) != 1 ) break;
          parambits |= 0x0100000000000000;
        }
      else if (!strcmp(param, "ziotype:"))
        {
          if ( fscanf(inifile, "%d", &ziotype) != 1 ) break;
          parambits |= 0x0200000000000000;
        }
    } //END while 1

  fclose(inifile);

  if (parambits != 0x03FFFFFFFFFFFFFF)
    return(-2);  //not all parameters set

  return(0);
}
//---------------------------------------------------------------------------
void EnableMotionControls()
{
  MainForm->XPlusButton->Enabled = true;
  MainForm->YPlusButton->Enabled = true;
  MainForm->ZPlusButton->Enabled = true;
  MainForm->XMinusButton->Enabled = true;
  MainForm->YMinusButton->Enabled = true;
  MainForm->ZMinusButton->Enabled = true;
  MainForm->ExecuteButton->Enabled = true;
  MainForm->StartButton->Enabled = true;
  MainForm->StepButton->Enabled = true;
  MainForm->OpenButton->Enabled = true;
  MainForm->ResetButton->Enabled = true;
  controlsenabled = 1;
}
//---------------------------------------------------------------------------
void DisableMotionControls()
{
  MainForm->XPlusButton->Enabled = false;
  MainForm->YPlusButton->Enabled = false;
  MainForm->ZPlusButton->Enabled = false;
  MainForm->XMinusButton->Enabled = false;
  MainForm->YMinusButton->Enabled = false;
  MainForm->ZMinusButton->Enabled = false;
  MainForm->ExecuteButton->Enabled = false;
  MainForm->StartButton->Enabled = false;
  MainForm->StepButton->Enabled = false;
  MainForm->ResetButton->Enabled = false;
  controlsenabled = 0;
}
//---------------------------------------------------------------------------
//Compensates for current tool length
int StartRapid(double x, double y, double z)
{
  long int pos, vel;
  byte mode;

  prevrx = x; prevry = y; prevrz = z;

  mode = LOAD_POS | LOAD_VEL | ENABLE_SERVO | START_NOW;

  vel = (long int)(0x10000*rapidoverride*xscale*xmaxvel/(1953.12*100));
  if (vel<0) vel= -vel;
  pos = (long int)( xscale*(x + xorg));
  ServoLoadTraj(xaxis, mode, pos, vel, 0, 0);

  vel = (long int)(0x10000*rapidoverride*yscale*ymaxvel/(1953.12*100));
  if (vel<0) vel= -vel;
  pos = (long int)( yscale*(y + yorg));
  ServoLoadTraj(yaxis, mode, pos, vel, 0, 0);

  vel = (long int)(0x10000*rapidoverride*zscale*zmaxvel/(1953.12*100));
  if (vel<0) vel= -vel;
  pos = (long int)( zscale*( z + zorg + toollen[toollennum] ));
  ServoLoadTraj(zaxis, mode, pos, vel, 0, 0);

  opmode = RAPID;

  return(0);
}
//---------------------------------------------------------------------------
//Display G code in list box with line n selected
void DisplayGCode(int n)
{
  int i, s, f, sel;  //starting line, finishing line, selected

  MainForm->GList->Items->Clear();

  sel = 3;

  if (n>=3) s = n-3;
  else
    {
      s = 0;
      sel = n;
    }

  f = s+7;
  if (f>numlines) f = numlines;

  for (i=s; i<f; i++)
    MainForm->GList->Items->Add( &(gcode[line[i]]) );

  MainForm->GList->ItemIndex = sel;  //Select the first line
}
//---------------------------------------------------------------------------
//
// G-Code Functions
//
//---------------------------------------------------------------------------
//Returns -1 if file not found, -2 if too many chars, -3 if too many lines
int ReadGCodeFile(char *name)
{
  FILE *gfile;
  int i, testchar;

  i = 0;                //char counter
  numlines = 1;         //line counter
  line[0] = 0;

  if ( (gfile=fopen(name,"r")) == NULL ) return(-1);

  Screen->Cursor = crHourGlass;

  while (1)
    {
      testchar = fgetc(gfile);
      if (testchar == '\r') gcode[i] = '\0';      //convert CR to null
      else if (testchar == '\n') gcode[i] = '\0'; //convert LF to null
      else if (testchar == EOF)                   //convert EOF to null & punt
        {
          gcode[i] = '\0';
          break;
        }
      else
        {
          gcode[i] = (char)testchar;
          if (i>0 && gcode[i-1] == '\0')
            {
              line[numlines] = i;
              numlines++;
            }
        }

      i++;
      if (i >= MAXCHARS) return(-2);
      if (numlines >= MAXLINES) return(-3);
    } //END while 1

  fclose(gfile);

  Screen->Cursor = crDefault;

  return(0);
}
//---------------------------------------------------------------------------
void SetLimitedFeedrate(double fr)
{
  if (fr>maxfr) fr = maxfr;
  SetFeedrate(fr);
}
//---------------------------------------------------------------------------
//Scan a gcode string and fill in the global tokens
//Returns  0 on success
//		   1 on success & some code causes a break in a continuous path
//        -4 on syntax error
//
int TokenScan(char *codestr)
{
  unsigned int i, numtok;
  int tokloc[40];  //token locations
  int gcode;
  int pathbreak;
  int commentmode;

  pathbreak = 0;
  commentmode = 0;

  //Scan for letter codes:
  numtok = 0;
  for (i=0; i<strlen(codestr); i++)
    {
      if ( codestr[i] == '(' ) commentmode++;
      if ( codestr[i] == ')' ) commentmode--;
      if ( codestr[i]>='A' && codestr[i]<='Z' && !commentmode )
        {
          tokloc[numtok] = i;
          numtok++;
        }
    }

  //Fill in the global values
  for (i=0; i<numtok; i++)
    {
      switch (codestr[tokloc[i]])
        {
        case 'G':   if (sscanf(codestr+tokloc[i]+1,"%d",&gcode) != 1) return(-4);
          //assign the gcode to the proper group
          switch (gcode)
            {
            case 4:  g0code = gcode;    //dwell
              pathbreak = 1;
              break;
            case 0:  pathbreak = 1;     //rapid
            case 1:                     //lines & arcs
            case 2:
            case 3:  g1code = gcode;
              break;
            case 17:                    //set arc planes
            case 18:
            case 19: g2code = gcode;
              break;
            case 43: g8code = gcode;    //set tool offset
              pathbreak = 1;
              break;
            }
          break;
        case 'M':   if (sscanf(codestr+tokloc[i]+1,"%d",&mcode) != 1) return(-4);
          pathbreak = 1;
          break;
        case 'X':   if (sscanf(codestr+tokloc[i]+1,"%lf",&gx) != 1) return(-4);
          if (g1code == -1) g1code = oldg1code;
          break;
        case 'Y':   if (sscanf(codestr+tokloc[i]+1,"%lf",&gy) != 1) return(-4);
          if (g1code == -1) g1code = oldg1code;
          break;
        case 'Z':   if (sscanf(codestr+tokloc[i]+1,"%lf",&gz) != 1) return(-4);
          if (g1code == -1) g1code = oldg1code;
          dgz = 1;  //flag change in z coord.
          break;
        case 'I':   if (arcmode == ARC_R) return(-4);
          if (sscanf(codestr+tokloc[i]+1,"%lf",&arci) != 1) return(-4);
          arcmode = ARC_IJK;
          break;
        case 'J':   if (arcmode == ARC_R) return(-4);
          if (sscanf(codestr+tokloc[i]+1,"%lf",&arcj) != 1) return(-4);
          arcmode = ARC_IJK;
          break;
        case 'K':   if (arcmode == ARC_R) return(-4);
          if (sscanf(codestr+tokloc[i]+1,"%lf",&arck) != 1) return(-4);
          arcmode = ARC_IJK;
          break;
        case 'R':   if (arcmode == ARC_IJK) return(-4);
          if (sscanf(codestr+tokloc[i]+1,"%lf",&arcr) != 1) return(-4);
          arcmode = ARC_R;
          break;
        case 'P':   if (sscanf(codestr+tokloc[i]+1,"%lf",&dwelltime)!= 1) return(-4);
          break;
        case 'F':   if (sscanf(codestr+tokloc[i]+1,"%lf",&fripm) != 1) return(-4);
          pathbreak = 1;
          break;
        case 'T':   if (sscanf(codestr+tokloc[i]+1,"%d",&ptoolnum) != 1) return(-4);
          break;
        case 'H':   if (sscanf(codestr+tokloc[i]+1,"%d",&ptoollennum) != 1) return(-4);
          break;
        } //END switch
    } //END fill in global values

  return(pathbreak);
}
//---------------------------------------------------------------------------
//Process arc using current interpreter globals, and adds it to the segment
//list.  Resets the g1code to -1 on completion.
//Returns 0 on success
//       -1 if not tangent to prev. segment
//       -2 if segment list is full
//		 -3 if arc data invalid
//
int ProcessArc()
{
  int res;

  double xc, yc, zc;  //arc center point
  double dx, dy, dz, dl;  //arc endpoint delta vector
  double mx, my, mz;  //arc midpoint
  double px, py, pz, pl;  //midpoint to center vector

  if (arcmode == ARC_IJK)
    {
      xc = oldgx + arci;
      yc = oldgy + arcj;
      zc = oldgz + arck + toollen[toollennum];
    }
  else if (arcmode == ARC_R)
    {
      if (arcr < 0.0) return -3;

      //Create vector from start point to mid-point
      dx = (gx - oldgx)/2; dy = (gy - oldgy)/2; dz = (gz - oldgz)/2;
      dl = mag(dx,dy,dz);
      if (dl > arcr) arcr = dl; //if radius too short, stretch to fit

      //create midpoint
      mx = oldgx + dx; my = oldgy + dy; mz = oldgz + dz + toollen[toollennum];

      //create vector from midpoint to arc center:
      if (g1code == 3) cross(arcnormx,arcnormy,arcnormz,dx,dy,dz,&px,&py,&pz);
      if (g1code == 2) cross(-arcnormx,-arcnormy,-arcnormz,dx,dy,dz,&px,&py,&pz);
      normalize(&px,&py,&pz);
      pl = sqrt(arcr*arcr - dl*dl);

      //create centger point
      xc = mx + pl*px; yc = my + pl*py; zc = mz + pl*pz;
    }
  else return(-3);

  if (g1code == 3)
    res = AddArcSeg( gx, gy, gz+ toollen[toollennum],   //end point
                     xc, yc, zc,     					  //center point
                     arcnormx, arcnormy, arcnormz );    //normal
  else if (g1code == 2)
    res = AddArcSeg( gx, gy, gz+ toollen[toollennum],   //end point
                     xc, yc, zc,     					  //center point
                     -arcnormx, -arcnormy, -arcnormz );    //normal
  else return(-3);

  if (res < 0) return(res);  //pass thru error code on failure
  else return(0);
}
//---------------------------------------------------------------------------
//codestr = null terminated string with G Codes
//append = flag to append segment to existing path
//	if append = 0, clear seg list, add segment, and then execute
//	if append = 1,
//		if not pathappendmode, clear seg list and set pathappend mode
//		add segment
//		if segment not tangent or if feedrate change
//			then execute prev path, clear pathappendmode
//      if mcode, execute gcodes, clear pathappendmode
//Returns 0 on successful execution
//		 -1 Break in continuous path (due to tangency error or mode change)
//       -2 Segment list is full
//       -3 invalid arc data or other data
//       -4 invalid syntax
int ExecuteGCode(char *codestr, int append)
{
  int res;
  int pathbreak;

  MainForm->FeedholdButton->SetFocus();

  //Null out non-modal global data:
  arcmode = ARC_NONE;
  arci = 0.0;
  arcj = 0.0;
  arck = 0.0;
  arcr = 0.0;
  dgz = 0;     //change in z coord.
  fripm = -1.0;
  mcode = -1;
  g0code = -1;
  oldg1code = g1code;
  g1code = -1;
  g2code = -1;
  g8code = -1;
  oldgx = gx; oldgy = gy; oldgz = gz;

  //Fill in the global data for the interpreter:
  res = TokenScan(codestr);
  if (res < 0) return(res);
  pathbreak = res;

  //If pathbreak encountered while building a path, start execution of the path
  //and exit without further processing
  if (pathbreak && pathappendmode)
    {
      InitPath();
      opmode = PATH;
      pathappendmode = 0;
      return(-1);
    }

  //Process feedrate change
  if (fripm>0.0)
    {
      feedrate = fripm/60.0;
      SetLimitedFeedrate(feedrate*froverride/100);
    }

  //Process the G-Codes
  //Process g8
  if (g8code == 43)
    {
      if (ptoollennum<1 || ptoollennum>MAXTOOLS) return(-3);
      //Adjust the default Z coordinate by the difference in tool heights
      //only adjust the current gz if it has not changed from the prev. block
      //ie, gz is interpreted for the new tool length if set in the same block as G43
      if (!dgz) gz -= (toollen[ptoollennum-1] - toollen[toollennum]);
      oldgz -= (toollen[ptoollennum-1] - toollen[toollennum]);
      toollennum = ptoollennum-1;
      toolchanged = 1;
    }

  //Process g2
  if (g2code == 17)
    {
      arcnormx = 0.0;
      arcnormy = 0.0;
      arcnormz = 1.0;
    }
  else if (g2code == 18)
    {
      arcnormx = 0.0;
      arcnormy = 1.0;
      arcnormz = 0.0;
    }
  else if (g2code == 19)
    {
      arcnormx = 1.0;
      arcnormy = 0.0;
      arcnormz = 0.0;
    }

  //Process g0
  if (g0code == 4)     //process dwell as an M code
    {
      if (mcode != -1) return(-4);  //flag error if already an  M code
      mcode = 95;				    //95 = mcode for a HAAS dwell
    }

  //Process g1 code
  if (g1code == 0)      //process a rapid move
    {
      StartRapid(gx, gy, gz);   //tool length handled by StartRapid
      return(0);
    }
  else if (g1code == 1 || g1code == 2 || g1code == 3)  //process a coordinated G01
    {
      if (feedrate == 0.0) return(-5);

      if (!pathappendmode)    //if path not initialized, start at prev. command pos.
        {
          cmdx = (double)(ServoGetPos(xaxis) + ServoGetPError(xaxis))/xscale - xorg;
          cmdy = (double)(ServoGetPos(yaxis) + ServoGetPError(yaxis))/yscale - yorg;
          cmdz = (double)(ServoGetPos(zaxis) + ServoGetPError(zaxis))/zscale - zorg;
          ClearSegList(cmdx, cmdy, cmdz);
        }

      if ( append == 0 )  //runs segments now
        {
          if (g1code == 1)
            res = AddLineSeg(gx, gy, gz + toollen[toollennum]);  //append line segment
          else
            res = ProcessArc();                                  //or append arc segment

          if ( res < 0 )    //punt on error
            return(res);
          InitPath();
          opmode = PATH;
          pathappendmode = 0;
          return(0);
        }
      else     //appends seg to path
        {
          if (g1code == 1)
            res = AddLineSeg(gx, gy, gz + toollen[toollennum]);  //append line segment
          else
            res = ProcessArc();                                  //or append arc segment

          if (res < -1)       //on real error, punt
            return(res);

          if (res ==-1 || pathbreak)   //if segment not tangent, execute the path
            {
              InitPath();
              opmode = PATH;
              pathappendmode = 0;
              if (res == -1)    //put gx, gy, gz back where they were
                {
                  gx = oldgx;
                  gy = oldgy;
                  gz = oldgz;
                }
              return(res);         //returns 0 if a path break, -1 for not tangent
            }

          pathappendmode = 1;
        }
    }  //END G01

  return(0);
} //END ExecuteGCode
//---------------------------------------------------------------------------
int ExecuteMCode(int mcode)
{
  char msgstr[80];

  switch (mcode)
    {
    case 0:	rungcode = 0;		//Halt execution of the program
      break;
    case 3:	rungcode = 0;
      SimpleMsgBox("Turn on spindle");
      break;
    case 5:   rungcode = 0;
      SimpleMsgBox("Turn off spindle");
      break;
    case 6:   rungcode = 0;
      if (ptoolnum<1 || ptoolnum>MAXTOOLS)
        {
          SimpleMsgBox("Invalid tool number");
          return(-3);
        }
      sprintf(msgstr,"Change tool to T%d",ptoolnum);
      SimpleMsgBox(msgstr);
      toolnum = ptoolnum-1;
      toolchanged = 1;
      break;
    case 21:	MainForm->ContouringCB->Checked = true;	 //turn on contouring
      break;
    case 22:	MainForm->ContouringCB->Checked = false; //turn off contouring
      break;
    case 30:	rungcode = 0;		//Halt execution of the program
      curline = 0;
      DisplayGCode(0);
      break;
    case 95:	dwellcounts = (dwelltime*1000)/MainForm->PathTimer->Interval;
      break;
    default:	rungcode = 0;
      sprintf(msgstr,"Unrecognized M code: %d", mcode);
      SimpleMsgBox(msgstr);
    }

  return(0);
}
//---------------------------------------------------------------------------
void DisplayErrorMsg(int n)
{
  switch (n)
    {
    case -2: SimpleMsgBox("Too many segments in a continuous path");
      break;
    case -3: SimpleMsgBox("Invalid arc data");
      break;
    case -4: SimpleMsgBox("Syntax error");
      break;
    case -5: SimpleMsgBox("No feedrate specified");
      break;
    }
}
//---------------------------------------------------------------------------
void ComError()
{
  MainForm->PathTimer->Enabled = false;
  NmcShutdown();
  SimpleMsgBox("NMC Communications Error!");
  MainForm->Close();
}
//---------------------------------------------------------------------------
int AnyKey()
{
  int i, retval;

  retval = 0;
  for (i=0; i<256; i++)
    if (GetAsyncKeyState(i) & 1) retval = 1;

  return(retval);
}
//---------------------------------------------------------------------------
int HomeMill()
{
  TMouseButton Button = mbLeft;
  TShiftState Shift;

  Screen->Cursor = crAppStart;
  HomeForm->Show();
  HomeForm->Repaint();
  AnyKey();

  //
  //  First do Z axis homing:
  //
  //home in the plus direction
  ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);  //first enable servo
  ServoSetGain(zaxis, zgain.kp, zgain.kd, zgain.ki, zgain.il,      //lower output limt
               zgain.ol/2, zgain.cl, 200, zgain.sr, zgain.dc);
  ServoClearBits(zaxis);        //clear pos error bit

  ServoSetHoming(zaxis, ON_POS_ERR | HOME_STOP_ABRUPT);
  MainForm->ZPlusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in plus Z direction
  do  //Wait for homing to complete
    {
      if (!NmcNoOp(zaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( (NmcGetStat(zaxis)&HOME_IN_PROG) && (ServoGetAux(zaxis)&SERVO_ON) );

  ServoResetRelHome(zaxis);
  ServoSetGain(zaxis, zgain.kp, zgain.kd, zgain.ki, zgain.il,      //reset output limit
               zgain.ol, zgain.cl, zgain.el, zgain.sr, zgain.dc);

  //Move 1" off of end stop
  ServoLoadTraj(zaxis, LOAD_POS | ENABLE_SERVO | START_NOW, -zscale, 0, 0, 0);
  do
    {
      if (!NmcNoOp(zaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( !(NmcGetStat(zaxis) & MOVE_DONE) );

  //
  //  Do X axis homing:
  //
  //home in the minus direction
  ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);  //first enable servo
  ServoSetGain(xaxis, xgain.kp, xgain.kd, xgain.ki, xgain.il,      //lower output limt
               xgain.ol/2, xgain.cl, 200, xgain.sr, xgain.dc);
  ServoClearBits(xaxis);        //clear pos error bit

  ServoSetHoming(xaxis, ON_POS_ERR | HOME_STOP_ABRUPT);
  MainForm->XMinusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in minus x direction
  do  //Wait for homing to complete
    {
      if (!NmcNoOp(xaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( (NmcGetStat(xaxis)&HOME_IN_PROG) && (ServoGetAux(xaxis)&SERVO_ON) );

  ServoResetRelHome(xaxis);
  ServoSetGain(xaxis, xgain.kp, xgain.kd, xgain.ki, xgain.il,      //reset output limit
               xgain.ol, xgain.cl, xgain.el, xgain.sr, xgain.dc);

  //Move 1" off of end stop
  ServoLoadTraj(xaxis, LOAD_POS | ENABLE_SERVO | START_NOW, xscale, 0, 0, 0);
  do
    {
      if (!NmcNoOp(xaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( !(NmcGetStat(xaxis) & MOVE_DONE) );


  //
  //  Do Y axis homing:
  //
  //home in the plus direction
  ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);  //first enable servo
  ServoSetGain(yaxis, ygain.kp, ygain.kd, ygain.ki, ygain.il,      //lower output limt
               ygain.ol/2, ygain.cl, 200, ygain.sr, ygain.dc);
  ServoClearBits(yaxis);        //clear pos error bit

  ServoSetHoming(yaxis, ON_POS_ERR | HOME_STOP_ABRUPT);
  MainForm->YPlusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in plus Y direction
  do  //Wait for homing to complete
    {
      if (!NmcNoOp(yaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( (NmcGetStat(yaxis)&HOME_IN_PROG) && (ServoGetAux(yaxis)&SERVO_ON) );

  ServoResetRelHome(yaxis);
  ServoSetGain(yaxis, ygain.kp, ygain.kd, ygain.ki, ygain.il,      //reset output limit
               ygain.ol, ygain.cl, ygain.el, ygain.sr, ygain.dc);

  //Move 1" off of end stop
  ServoLoadTraj(yaxis, LOAD_POS | ENABLE_SERVO | START_NOW, -yscale, 0, 0, 0);
  do
    {
      if (!NmcNoOp(yaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( !(NmcGetStat(yaxis) & MOVE_DONE) );

  mill_homed = 1;

  Screen->Cursor = crDefault;
  opmode = IDLE;
  HomeForm->Close();
  return(0);
}
//---------------------------------------------------------------------------
int HomeMill_switches()
{
  TMouseButton Button = mbLeft;
  TShiftState Shift;

  Screen->Cursor = crAppStart;
  HomeForm->Show();
  HomeForm->Repaint();
  AnyKey();

  MainForm->AllOnButtonClick(NULL);  //First enable all servos

  NmcNoOp(xaxis);  	//get the current switch states
  NmcNoOp(yaxis);
  NmcNoOp(zaxis);

  //
  //  First do Z axis homing:
  //
  //first home in the plus direction if not on limit switch
  if ( !(NmcGetStat(zaxis) & LIMIT1) )
    {
      ServoSetHoming(zaxis, ON_LIMIT1 | HOME_STOP_ABRUPT);
      MainForm->ZPlusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in plus Z direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(zaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(zaxis)&HOME_IN_PROG) && (ServoGetAux(zaxis)&SERVO_ON) );
    }

  if ( !(ServoGetAux(zaxis) & SERVO_ON) )   //make sure we didn't hit a stop
    {
      SimpleMsgBox("Homing procedure failed!");
      MainForm->Close();
    }

  if ( NmcGetStat(zaxis) & LIMIT1 )     //home in the minus direction if on limit
    {
      ServoSetHoming(zaxis, ON_LIMIT1 | HOME_STOP_SMOOTH);
      MainForm->ZMinusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in minus Z direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(zaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(zaxis)&HOME_IN_PROG) && (ServoGetAux(zaxis)&SERVO_ON) );
    }
  ServoResetRelHome(zaxis);

  //
  //  X axis homing:
  //
  //first home in the plus direction if not on limit switch
  if ( !(NmcGetStat(xaxis) & LIMIT1) )
    {
      ServoSetHoming(xaxis, ON_LIMIT1 | HOME_STOP_ABRUPT);
      MainForm->XPlusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in plus X direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(xaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(xaxis)&HOME_IN_PROG) && (ServoGetAux(xaxis)&SERVO_ON) );
    }

  if ( !(ServoGetAux(xaxis) & SERVO_ON) )   //make sure we didn't hit a stop
    {
      SimpleMsgBox("Homing procedure failed!");
      MainForm->Close();
    }

  if ( NmcGetStat(xaxis) & LIMIT1 )     //home in the minus direction if on limit
    {
      ServoSetHoming(xaxis, ON_LIMIT1 | HOME_STOP_SMOOTH);
      MainForm->XMinusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in minus X direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(xaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(xaxis)&HOME_IN_PROG) && (ServoGetAux(xaxis)&SERVO_ON) );
    }
  ServoResetRelHome(xaxis);

  //
  //  Y axis homing:
  //
  //first home in the plus direction if not on limit switch
  if ( !(NmcGetStat(yaxis) & LIMIT1) )
    {
      ServoSetHoming(yaxis, ON_LIMIT1 | HOME_STOP_ABRUPT);
      MainForm->YPlusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in plus y direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(yaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(yaxis)&HOME_IN_PROG) && (ServoGetAux(yaxis)&SERVO_ON) );
    }

  if ( !(ServoGetAux(yaxis) & SERVO_ON) )   //make sure we didn't hit a stop
    {
      SimpleMsgBox("Homing procedure failed!");
      MainForm->Close();
    }

  if ( NmcGetStat(yaxis) & LIMIT1 )     //home in the minus direction if on limit
    {
      ServoSetHoming(yaxis, ON_LIMIT1 | HOME_STOP_SMOOTH);
      MainForm->YMinusButtonMouseDown(NULL, Button, Shift, 0, 0);  //move in minus y direction
      do  //Wait for homing to complete
        {
          if (!NmcNoOp(yaxis)) ComError();
          if (AnyKey()) return(-1);
        }
      while ( (NmcGetStat(yaxis)&HOME_IN_PROG) && (ServoGetAux(yaxis)&SERVO_ON) );
    }
  ServoResetRelHome(yaxis);

  StartRapid(xmax-0.1, ymax-0.1, zmax-0.1);  //move to the edge of the range of motion
  do
    {
      if (!NmcNoOp(xaxis)) ComError();
      if (!NmcNoOp(yaxis)) ComError();
      if (!NmcNoOp(zaxis)) ComError();
      if (AnyKey()) return(-1);
    }
  while ( !(NmcGetStat(xaxis) & MOVE_DONE) ||
          !(NmcGetStat(yaxis) & MOVE_DONE) ||
          !(NmcGetStat(zaxis) & MOVE_DONE) );

  mill_homed = 1;

  Screen->Cursor = crDefault;
  opmode = IDLE;
  HomeForm->Close();
  return(0);
}
//---------------------------------------------------------------------------
//
// Event Functions
//
//---------------------------------------------------------------------------
__fastcall TMainForm::TMainForm(TComponent* Owner)
  : TForm(Owner)
{
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::PathTimerTimer(TObject *Sender)
{
  int res;
  double posx, posy, posz;
  char txtstr[80];

  GetKeyState(VK_LBUTTON);

  if ( ( opmode == JOGXM || opmode == JOGXP ||
         opmode == JOGYM || opmode == JOGYP ||
         opmode == JOGZM || opmode == JOGZP) &&
       ( ((GetKeyState(VK_LBUTTON) & 0x80) == 0) &&
         ((GetKeyState(VK_RBUTTON) & 0x80) == 0) ) )
    {
      if (opmode == JOGXM || opmode == JOGXP)
        ServoLoadTraj(xaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      else if (opmode == JOGYM || opmode == JOGYP)
        ServoLoadTraj(yaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      else if (opmode == JOGZM || opmode == JOGZP)
        ServoLoadTraj(zaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      opmode = JOG_STOP;
    }

  if (!controlsenabled)
    {
      if (opmode == IDLE && !rungcode && !feedhold)
        EnableMotionControls();
    }
  else
    {
      //###SMMOD
      //if (opmode == SERVO_OFF)
      //  DisableMotionControls();
      //###SMMOD
    }

  //Execute path mode or Nop as required to get current data
  //
  if (opmode == PATH)
    {
      res = AddPathPoints();
      if ( res ==-1 )
        {
          opmode = PATH_END;
          NmcNoOp(xaxis);
          NmcNoOp(yaxis);
          NmcNoOp(zaxis);
        }
      else if (res == -2) ComError();
    }
  else
    {
      if (!NmcNoOp(xaxis)) ComError();
      if (!NmcNoOp(yaxis)) ComError();
      if (!NmcNoOp(zaxis)) ComError();
    }


  //Turn off servo if not moving:
  if (opmode == IDLE && rungcode != 1)
    {
      if ( (ServoGetAux(xaxis)& SERVO_ON)  ) ServoStopMotor(xaxis, AMP_ENABLE | MOTOR_OFF);
      if ( (ServoGetAux(yaxis)& SERVO_ON)  ) ServoStopMotor(yaxis, AMP_ENABLE | MOTOR_OFF);
      if ( (ServoGetAux(zaxis)& SERVO_ON)  ) ServoStopMotor(zaxis, AMP_ENABLE | MOTOR_OFF);
    }

  if ((!(ServoGetAux(xaxis)& SERVO_ON)) && opmode==JOGXP) opmode = IDLE;
  if ((!(ServoGetAux(yaxis)& SERVO_ON)) && opmode==JOGYP) opmode = IDLE;
  if ((!(ServoGetAux(zaxis)& SERVO_ON)) && opmode==JOGZP) opmode = IDLE;


  //Check for path end complete
  //
  if (opmode == PATH_END)
    if ( !(ServoGetAux(xaxis)& PATH_MODE) && !(ServoGetAux(yaxis)& PATH_MODE) &&
         !(ServoGetAux(zaxis)& PATH_MODE) )
      {
        opmode = IDLE;
      }

  //Check for completion of jog mode or rapid move
  //
  if (opmode == JOG_STOP || opmode == RAPID)
    {
      if (  (NmcGetStat(xaxis)&MOVE_DONE) && (NmcGetStat(yaxis)&MOVE_DONE) &&
            (NmcGetStat(zaxis)&MOVE_DONE) )
        {
          opmode = IDLE;
        }
    }

  //Check for servo off
  //
  //###SMMOD
  //if ( !(ServoGetAux(xaxis)& SERVO_ON)  )
  //  {
  //  XOnCB->Checked = false;
  //  //shutdown all axes if not idle or off
  //  if (opmode != IDLE && opmode != SERVO_OFF) AllOffButtonClick(Sender);
  //  }
  //if ( !(ServoGetAux(yaxis)& SERVO_ON)  )
  //  {
  //  YOnCB->Checked = false;
  //  //shutdown all axes if not idle or off
  //  if (opmode != IDLE && opmode != SERVO_OFF) AllOffButtonClick(Sender);
  //  }
  //if ( !(ServoGetAux(zaxis)& SERVO_ON)  )
  //  {
  //  ZOnCB->Checked = false;
  //  //shutdown all axes if not idle or off
  //  if (opmode != IDLE && opmode != SERVO_OFF) AllOffButtonClick(Sender);
  //  }
  //###SMMOD

  //get machine position
  posx = (double)ServoGetPos(xaxis)/xscale;
  posy = (double)ServoGetPos(yaxis)/yscale;
  posz = (double)ServoGetPos(zaxis)/zscale;

  //Check if outside range of motion
  if (mill_homed)
    {
      if ( (posx>xmax && opmode==JOGXP) || (posx<xmin && opmode==JOGXM) )
        {
          ServoLoadTraj(xaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
          opmode = JOG_STOP;
        }
      else if ( (posy>ymax && opmode==JOGYP) || (posy<ymin && opmode==JOGYM) )
        {
          ServoLoadTraj(yaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
          opmode = JOG_STOP;
        }
      else if ( (posz>zmax && opmode==JOGZP) || (posz<zmin && opmode==JOGZM) )
        {
          ServoLoadTraj(zaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
          opmode = JOG_STOP;
        }
      else if (opmode==PATH || opmode==PATH_END || opmode==RAPID) //check for programmed modes
        {
          if ( posx>xmax || posx<xmin ||
               posy>ymax || posy<ymin ||
               posz>zmax || posz<zmin )
            {
              ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);
              ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);
              ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);
              opmode = IDLE;
              rungcode = 0;
              pathappendmode = 0;
              SimpleMsgBox("Programmed move exceeds the range of motion");
            }
        }
    }

  //Update position display
  //
  if (ProgramRB->Checked == true) posx -= xorg;
  sprintf(txtstr,formatstr, posx);
  XPosEdit->Text = txtstr;

  if (ProgramRB->Checked == true) posy -= yorg;
  sprintf(txtstr,formatstr, posy);
  YPosEdit->Text = txtstr;

  if (ProgramRB->Checked == true) posz -= (zorg + toollen[toollennum]);
  sprintf(txtstr,formatstr, posz);
  ZPosEdit->Text = txtstr;

  //Update tool info if changed
  //
  if (toolchanged)
    {
      sprintf(txtstr,"%d", toolnum+1);
      ToolNumEdit->Text = txtstr;

      sprintf(txtstr,"%.4f", toollen[toollennum]);
      ToolLenEdit->Text = txtstr;
      toolchanged = 0;
    }

  //Check for dwell
  if (dwellcounts)
    dwellcounts--;  //decrement dwell counter if not already zero

  //Run G-Code if enabled
  //
  if ( (opmode == IDLE) && (rungcode==1) && (dwellcounts==0) && (!feedhold) )   //###fix: added !feedhold
    {
      if (ContouringCB->Checked == true) contouring = 1; //get current contouring state
      else contouring = 0;

      while (1)  //run lines of G-Code until not IDLE (or until an error)
        {
          if (curline>=numlines)  //punt if at end of code
            {
              rungcode = 0;
              pathappendmode = 0;
              SimpleMsgBox("At end of G-Code program.");
              break;
            }

          res = ExecuteGCode(&(gcode[line[curline]]), contouring);

          if (res != 0 && res != -1)
            {
              rungcode = 0;
              pathappendmode = 0;
              DisplayErrorMsg(res);
              break;
            }

          if (res == -1) break;  //break out of while loop without advancing curline

          if (res == 0)    //line of code executed successfully
            {
              curline++;

              if (mcode>=0)
                ExecuteMCode(mcode);

              if (curline == numlines)     //require an M00 at end of program
                {
                  rungcode = 0;
                  pathappendmode = 0;
                  break;
                }

              DisplayGCode(curline);

              //break if not idle or if an M code was executed - otherwise g-codes will
              // continue to execute until an action is started
              if ( (opmode != IDLE) || (mcode>=0) ) break;
            }

        } //END while 1
    } //if IDLE and rungcode

  if (rungcode)
    if ( !(ServoGetAux(xaxis)& SERVO_ON) ||
         !(ServoGetAux(yaxis)& SERVO_ON) ||
         !(ServoGetAux(zaxis)& SERVO_ON) )
      {
        opmode = IDLE;
        rungcode = 0;
        pathappendmode = 0;
      }

}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ExitButtonClick(TObject *Sender)
{
  Close();
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::StartupTimerTimer(TObject *Sender)
{
  long int jogacc;
  int res;

  DisableMotionControls();

  ErrorPrinting(0);  //Turn off error printing

  StartupTimer->Enabled = false;
  SimpleMsgBox("Please turn on Servo Controller Power");

  if (ReadIniFile() != 0)
    {
      SimpleMsgBox("Could not read initialization data in PSCNC.INI");
      Close();
    }
  if (ReadToolFile() != 0)
    {
      SimpleMsgBox("Could not read tool length data in PSCNC.TLL");
      Close();
    }

  if (nummod>0)
    {
      NmcShutdown();
    }

  nummod = NmcInit(comport, 115200);  //Controllers on comport, use 115200 baud

  if (nummod < 3)
    {
      SimpleMsgBox("3 servos not found");
      Close();
    }

  ServoSetIoCtrl(xaxis, xiotype);   //Set the proper I/O control values for
  ServoSetIoCtrl(yaxis, yiotype);   // v5, or for v10 with antiphase or 3 phase
  ServoSetIoCtrl(zaxis, ziotype);   //

  ServoStopMotor(xaxis, MOTOR_OFF | ADV_FEATURE);     //reset amp enable
  ServoStopMotor(yaxis, MOTOR_OFF | ADV_FEATURE);
  ServoStopMotor(zaxis, MOTOR_OFF | ADV_FEATURE);

  ServoSetGain(xaxis, xgain.kp, xgain.kd, xgain.ki, xgain.il,
               xgain.ol, xgain.cl, xgain.el, xgain.sr, xgain.dc);
  ServoSetGain(yaxis, ygain.kp, ygain.kd, ygain.ki, ygain.il,
               ygain.ol, ygain.cl, ygain.el, ygain.sr, ygain.dc);
  ServoSetGain(zaxis, zgain.kp, zgain.kd, zgain.ki, zgain.il,
               zgain.ol, zgain.cl, zgain.el, zgain.sr, zgain.dc);

  //Do phasing for S3-type drivers
  Screen->Cursor = crHourGlass;
  if (NmcGetModVer(xaxis)==20) ServoLoadTraj(xaxis, LOAD_PWM | START_NOW, 0,0,0,0);
  if (NmcGetModVer(yaxis)==20) ServoLoadTraj(yaxis, LOAD_PWM | START_NOW, 0,0,0,0);
  if (NmcGetModVer(zaxis)==20) ServoLoadTraj(zaxis, LOAD_PWM | START_NOW, 0,0,0,0);
  Sleep(2000);
  Screen->Cursor = crDefault;

  ServoStopMotor(xaxis, AMP_ENABLE | MOTOR_OFF);
  ServoStopMotor(yaxis, AMP_ENABLE | MOTOR_OFF);
  ServoStopMotor(zaxis, AMP_ENABLE | MOTOR_OFF);
  ServoClearBits(xaxis);                                        //clear errors
  ServoClearBits(yaxis);                                        //clear errors
  ServoClearBits(zaxis);                                        //clear errors
  opmode = IDLE;

  jogacc = 0x10000*(xacc*xscale/(1953.12*1953.12));
  if (jogacc<0) jogacc = -jogacc;
  ServoLoadTraj(xaxis, LOAD_ACC | START_NOW, 0, 0, jogacc, 0);

  jogacc = 0x10000*(yacc*yscale/(1953.12*1953.12));
  if (jogacc<0) jogacc = -jogacc;
  ServoLoadTraj(yaxis, LOAD_ACC | START_NOW, 0, 0, jogacc, 0);

  jogacc = 0x10000*(zacc*zscale/(1953.12*1953.12));
  if (jogacc<0) jogacc = -jogacc;
  ServoLoadTraj(zaxis, LOAD_ACC | START_NOW, 0, 0, jogacc, 0);

  //Set the required status items the path control module
  NmcDefineStatus(xaxis, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);
  NmcDefineStatus(yaxis, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);
  NmcDefineStatus(zaxis, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);

  //Establish max feed rate as the maximum of the X, Y and Z feed rates
  maxfr = xmaxvel;
  if (ymaxvel>maxfr) maxfr = ymaxvel;
  if (zmaxvel>maxfr) maxfr = zmaxvel;

  //Initialize path control module
  accel = xacc;
  if (accel>yacc) accel = yacc;
  if (accel>zacc) accel = zacc;
  if (SetPathParams2(pfreq, nbuf, xaxis, yaxis, zaxis, 255, 0, xscale, yscale, zscale, accel) != 0)
    Close();
  xorg = 0.0; yorg = 0.0; zorg = 0.0;
  SetOrigin(0.0, 0.0, 0.0);
  SetFeedrate(0.10*maxfr);     //use 10% of max feed rate as default
  SetTangentTolerance(maxang);

  //Set the format for displaying positions
  if ( (xmax - xmin)>=90.0 || (ymax - ymin)>=90.0 || (zmax - zmin)>=90.0 )
    strcpy(formatstr,"%8.3f");
  else strcpy(formatstr,"%8.4f");

  res = MessageBox(NULL, "Do you want to execute the homing procedure?", "", MB_YESNOCANCEL);
  if (res == IDYES)
    if (HomeMill() != 0) Close();
    else if (res == IDCANCEL) Close();

  //Set path timer tick rate to 1/3th the path buffer time
  PathTimer->Interval = (nbuf*1000)/pfreq/3;
  PathTimer->Enabled = 1;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XOnCBClick(TObject *Sender)
{
  if (XOnCB->Checked == true)  //enable the servos
    {
      ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
      ServoClearBits(xaxis);                                        //clear errors
      XPosEdit->Color = clWindow;
      if ( (YOnCB->Checked == true) && (ZOnCB->Checked == true) )
        opmode = IDLE;
    }
  else  //disable the servos
    {
      //leave amp enabled for SS-DRIVE (xpadv > 0)
      if (xpadv>0) ServoStopMotor(xaxis, AMP_ENABLE | MOTOR_OFF);
      else ServoStopMotor(xaxis, MOTOR_OFF);
      XPosEdit->Color = clSilver;
      opmode = SERVO_OFF;
      rungcode = 0;
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YOnCBClick(TObject *Sender)
{
  if (YOnCB->Checked == true)
    {
      ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);
      ServoClearBits(yaxis);                                        //clear errors
      YPosEdit->Color = clWindow;
      if ( (XOnCB->Checked == true) && (ZOnCB->Checked == true) )
        opmode = IDLE;
    }
  else
    {
      //leave amp enabled for SS-DRIVE (ypadv > 0)
      if (ypadv>0) ServoStopMotor(yaxis, AMP_ENABLE | MOTOR_OFF);
      else ServoStopMotor(yaxis, MOTOR_OFF);
      YPosEdit->Color = clSilver;
      opmode = SERVO_OFF;
      rungcode = 0;
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZOnCBClick(TObject *Sender)
{
  if (ZOnCB->Checked == true)
    {
      ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);
      ServoClearBits(zaxis);                                        //clear errors
      ZPosEdit->Color = clWindow;
      if ( (XOnCB->Checked == true) && (YOnCB->Checked == true) )
        opmode = IDLE;
    }
  else
    {
      //leave amp enabled for SS-DRIVE (zpadv > 0)
      if (zpadv>0) ServoStopMotor(zaxis, AMP_ENABLE | MOTOR_OFF);
      else ServoStopMotor(zaxis, MOTOR_OFF);
      ZPosEdit->Color = clSilver;
      opmode = SERVO_OFF;
      rungcode = 0;
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::AllOnButtonClick(TObject *Sender)
{
  XOnCB->Checked = true;
  YOnCB->Checked = true;
  ZOnCB->Checked = true;
  MainForm->XPosEdit->Color = clWindow;
  MainForm->YPosEdit->Color = clWindow;
  MainForm->ZPosEdit->Color = clWindow;
  opmode = IDLE;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::AllOffButtonClick(TObject *Sender)
{
  //Disable servos
  //###XOnCB->Checked = false;
  //###YOnCB->Checked = false;
  //###ZOnCB->Checked = false;

  //set operting mode, turn off g-code interpreter, clear path being built
  //###opmode = SERVO_OFF;
  opmode = IDLE;
  rungcode = 0;
  pathappendmode = 0;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::FormClose(TObject *Sender, TCloseAction &Action)
{
  if (nummod != 0) NmcShutdown();
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XPlusButtonMouseDown(TObject *Sender,
                                                TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the maximum X position
  if ( mill_homed && (double)ServoGetPos(xaxis)/xscale > xmax) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = (long int)(0x10000*0.5*xscale*xmaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = (long int)(0x10000*xscale*xmaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(xaxis, mode, 0, vel, 0, 0);
  opmode = JOGXP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XPlusButtonMouseUp(TObject *Sender,
                                              TMouseButton Button, TShiftState Shift, int X, int Y)
{
  ServoLoadTraj(xaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
  opmode = JOG_STOP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XMinusButtonMouseDown(TObject *Sender,
                                                 TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the minimum X position
  if ( mill_homed && (double)ServoGetPos(xaxis)/xscale < xmin) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = -(long int)(0x10000*0.5*xscale*xmaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = -(long int)(0x10000*xscale*xmaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(xaxis, mode, 0, vel, 0, 0);
  opmode = JOGXM;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YPlusButtonMouseDown(TObject *Sender,
                                                TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the maximum Y position
  if ( mill_homed && (double)ServoGetPos(yaxis)/yscale > ymax) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = (long int)(0x10000*0.5*yscale*ymaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = (long int)(0x10000*yscale*ymaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(yaxis, mode, 0, vel, 0, 0);
  opmode = JOGYP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YMinusButtonMouseDown(TObject *Sender,
                                                 TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the minimum Y position
  if ( mill_homed && (double)ServoGetPos(yaxis)/yscale < ymin) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = -(long int)(0x10000*0.5*yscale*ymaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = -(long int)(0x10000*yscale*ymaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(yaxis, mode, 0, vel, 0, 0);
  opmode = JOGYM;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YPlusButtonMouseUp(TObject *Sender,
                                              TMouseButton Button, TShiftState Shift, int X, int Y)
{
  ServoLoadTraj(yaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
  opmode = JOG_STOP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZPlusButtonMouseDown(TObject *Sender,
                                                TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the maximum Z position
  if ( mill_homed && (double)ServoGetPos(zaxis)/zscale > zmax) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = (long int)(0x10000*0.5*zscale*zmaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = (long int)(0x10000*zscale*zmaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(zaxis, mode, 0, vel, 0, 0);
  opmode = JOGZP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZMinusButtonMouseDown(TObject *Sender,
                                                 TMouseButton Button, TShiftState Shift, int X, int Y)
{
  long int vel;
  byte mode;

  //punt if we are past the minimum Z position
  if ( mill_homed && (double)ServoGetPos(zaxis)/zscale < zmin) return;

  mode = LOAD_VEL | ENABLE_SERVO | VEL_MODE | START_NOW;

  if (Button == mbLeft)
    vel = -(long int)(0x10000*0.5*zscale*zmaxvel*(JogTrackBar->Position+1)/(1953.12*JogTrackBar->Max));
  else
    vel = -(long int)(0x10000*zscale*zmaxvel/1953.12);

  if (vel<0)
    {
      vel = -vel;
      mode |= REVERSE;
    }

  ServoLoadTraj(zaxis, mode, 0, vel, 0, 0);
  opmode = JOGZM;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZPlusButtonMouseUp(TObject *Sender,
                                              TMouseButton Button, TShiftState Shift, int X, int Y)
{
  ServoLoadTraj(zaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
  opmode = JOG_STOP;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XOrgButtonClick(TObject *Sender)
{
  //If servo is on, use commanded position, otherwise, use the actual
  if (ServoGetAux(xaxis) & SERVO_ON )
    xorg = (double)(ServoGetPos(xaxis) + ServoGetPError(xaxis))/xscale;
  else xorg = (double)ServoGetPos(xaxis)/xscale;
  SetOrigin(xorg, yorg, zorg);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YOrgButtonClick(TObject *Sender)
{
  //If servo is on, use commanded position, otherwise, use the actual
  if (ServoGetAux(yaxis) & SERVO_ON )
    yorg = (double)(ServoGetPos(yaxis) + ServoGetPError(yaxis))/yscale;
  else yorg = (double)ServoGetPos(yaxis)/yscale;
  SetOrigin(xorg, yorg, zorg);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZOrgButtonClick(TObject *Sender)
{
  //If servo is on, use commanded position, otherwise, use the actual
  if (ServoGetAux(zaxis) & SERVO_ON )
    zorg = (double)(ServoGetPos(zaxis) + ServoGetPError(zaxis))/zscale;
  else zorg = (double)ServoGetPos(zaxis)/zscale;
  zorg -= toollen[toollennum];
  SetOrigin(xorg, yorg, zorg);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::SetOrgButtonClick(TObject *Sender)
{
  XOrgButtonClick(Sender);
  YOrgButtonClick(Sender);
  ZOrgButtonClick(Sender);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::FeedrateUDClick(TObject *Sender, TUDBtnType Button)
{
  char prnstr[20];

  if (Button == btNext)
    froverride += 10;
  else
    froverride -=10;

  sprintf(prnstr,"%d%",froverride);
  FeedrateEdit->Text = prnstr;

  if (!feedhold)
    SetLimitedFeedrate(feedrate*froverride/100);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::RapidUDClick(TObject *Sender, TUDBtnType Button)
{
  char prnstr[20];

  if (Button == btNext)
    rapidoverride += 10;
  else
    rapidoverride -=10;

  sprintf(prnstr,"%d%",rapidoverride);
  RapidEdit->Text = prnstr;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::BitBtn1Click(TObject *Sender)
{
  Close();
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::OpenButtonClick(TObject *Sender)
{
  int res;
  int i;

  if (OpenDialog->Execute() == false) return;  //punt on cancel

  res = ReadGCodeFile( OpenDialog->FileName.c_str() );
  if ( res == -1 )
    SimpleMsgBox("Could not open selected G-Code file");
  else if (res == -2)
    SimpleMsgBox("G-Code file too large");
  else if (res == -3)
    SimpleMsgBox("G-Code file has too many lines");

  Caption = strcat("Lobo CNC - ", OpenDialog->FileName.c_str());

  //Show the first n lines of code
  GList->Items->Clear();

  for (i=0; i<7; i++)
    {
      if (i == numlines) break;
      GList->Items->Add( &(gcode[line[i]]) );
    }

  GList->ItemIndex = 0;  //Select the first line
  curline = 0;
  feedrate = 0.0;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ExecuteButtonClick(TObject *Sender)
{
  int res;

  ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(xaxis);                                        //clear errors
  ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(yaxis);                                        //clear errors
  ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(zaxis);                                        //clear errors

  //Initialize interpreter positions to the current command position
  gx = (double)(ServoGetPos(xaxis) + ServoGetPError(xaxis))/xscale - xorg;
  gy = (double)(ServoGetPos(yaxis) + ServoGetPError(yaxis))/yscale - yorg;
  gz = (double)(ServoGetPos(zaxis) + ServoGetPError(zaxis))/zscale - zorg;
  gz -= toollen[toollennum];

  res = ExecuteGCode(ImmediateEdit->Text.c_str(), 0);

  if (res != 0) SimpleMsgBox("Syntax or data error");
  else
    {
      DisableMotionControls();
      if (mcode>=0) ExecuteMCode(mcode);
    }

}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ImmediateEditEnter(TObject *Sender)
{
  ImmediateEdit->Tag = 1;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ImmediateEditClick(TObject *Sender)
{
  if (ImmediateEdit->Tag == 1)
    {
      ImmediateEdit->SelStart = 0;
      ImmediateEdit->SelLength = 100;
      ExecuteButton->Default = true;
      ImmediateEdit->Tag = 0;
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ResetButtonClick(TObject *Sender)
{
  curline = 0;
  DisplayGCode(0);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::StepButtonClick(TObject *Sender)
{
  int res;

  if (opmode != IDLE) return;

  if (curline==numlines)
    {
      SimpleMsgBox("At end of G-Code program.");
      return;  //punt if at end of code
    }

  ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(xaxis);                                        //clear errors
  ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(yaxis);                                        //clear errors
  ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(zaxis);                                        //clear errors

  //Initialize interpreter positions to the current command position
  gx = (double)(ServoGetPos(xaxis) + ServoGetPError(xaxis))/xscale - xorg;
  gy = (double)(ServoGetPos(yaxis) + ServoGetPError(yaxis))/yscale - yorg;
  gz = (double)(ServoGetPos(zaxis) + ServoGetPError(zaxis))/zscale - zorg;
  gz -= toollen[toollennum];  //Adjust for tool length

  res = ExecuteGCode( &(gcode[line[curline]]) , 0);

  if (mcode >= 0) ExecuteMCode(mcode);

  if (res == 0 || res == 1)
    {
      curline++;
      if (curline<numlines) DisplayGCode(curline);
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::StopButtonClick(TObject *Sender)
{
  rungcode = 0;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::StartButtonClick(TObject *Sender)
{
  //Initialize interpreter positions to the current command position
  gx = (double)(ServoGetPos(xaxis) + ServoGetPError(xaxis))/xscale - xorg;
  gy = (double)(ServoGetPos(yaxis) + ServoGetPError(yaxis))/yscale - yorg;
  gz = (double)(ServoGetPos(zaxis) + ServoGetPError(zaxis))/zscale - zorg;
  gz -= toollen[toollennum];  //Adjust for tool length

  rungcode = 1;

  ServoStopMotor(xaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(xaxis);                                        //clear errors
  ServoStopMotor(yaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(yaxis);                                        //clear errors
  ServoStopMotor(zaxis, AMP_ENABLE | STOP_ABRUPT);  //start servo
  ServoClearBits(zaxis);                                        //clear errors

  DisableMotionControls();	
  OpenButton->Enabled = false;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::FeedholdButtonClick(TObject *Sender)
{
  if (feedhold) return;  //punt if already holding

  if (opmode == IDLE && !rungcode)
    return;  //punt if nothing is moving

  SetFeedrate(0.0);
  if (opmode == RAPID)
    {
      ServoLoadTraj(xaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      ServoLoadTraj(yaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      ServoLoadTraj(zaxis, LOAD_VEL|ENABLE_SERVO|VEL_MODE|START_NOW, 0, 0, 0, 0);
      feedhold = RAPIDHOLD;
    }
  else feedhold = PATHHOLD;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ResumeButtonClick(TObject *Sender)
{
  if (!feedhold) return;    //punt if not holding

  SetLimitedFeedrate(feedrate*froverride/100);
  if (feedhold == RAPIDHOLD) StartRapid(prevrx, prevry, prevrz);
  feedhold = NO_HOLD;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::AbortButtonClick(TObject *Sender)
{
  if (!feedhold)    //Execute a feedhold if not already holding
    {
      FeedholdButtonClick(Sender);
      SimpleMsgBox("Click 'Abort' again to terminate motion");
    }
  else
    {
      opmode = IDLE;           //force into IDLE mode, remove hold, stop execution
      feedhold = NO_HOLD;
      rungcode = 0;
      SetLimitedFeedrate(feedrate*froverride/100);    //restore old feedrate
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::HelpButtonClick(TObject *Sender)
{
  int result;
  result = (int)ShellExecute(NULL,NULL,"pscnc.pdf",NULL,"",SW_SHOWNORMAL);
  if (result<32) SimpleMsgBox("Acrobat Reader or 'pscnc.pdf' not found");
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::XPosEditClick(TObject *Sender)
{
  double oldval, newval;

  oldval = GetFloat(XPosEdit);
  PositionEditForm->PosEdit->Text = XPosEdit->Text;
  if (PositionEditForm->ShowModal() == 1)
    {
      newval = GetFloat(PositionEditForm->PosEdit);
      xorg -= newval - oldval;
      SetOrigin(xorg, yorg, zorg);
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::YPosEditClick(TObject *Sender)
{
  double oldval, newval;

  oldval = GetFloat(YPosEdit);
  PositionEditForm->PosEdit->Text = YPosEdit->Text;
  if (PositionEditForm->ShowModal() == 1)
    {
      newval = GetFloat(PositionEditForm->PosEdit);
      yorg -= newval - oldval;
      SetOrigin(xorg, yorg, zorg);
    }
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ZPosEditClick(TObject *Sender)
{
  double oldval, newval;

  oldval = GetFloat(ZPosEdit);
  PositionEditForm->PosEdit->Text = ZPosEdit->Text;
  if (PositionEditForm->ShowModal() == 1)
    {
      newval = GetFloat(PositionEditForm->PosEdit);
      zorg -= newval - oldval;
      SetOrigin(xorg, yorg, zorg);
    }
}
//---------------------------------------------------------------------------

