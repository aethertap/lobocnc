//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#include <vcl\vcl.h>
#include <stdio.h>
#pragma hdrstop

#include "sio_util.h"
#include "nmccom.h"
#include "picservo.h"
//---------------------------------------------------------------------------//
extern NMCMOD	mod[]; 		//Array of modules
extern int nummod;
extern HANDLE ComPort;


//---------------------------------------------------------------------------//
//  Function Name:  ServoNewMod (Internal Library Function)                  //
//  Return Value:   Pointer to the new SERVOMOD structure.                   //
//  Parameters:     None                                                     //
//  Description:    Creates and initializes a new SERVOMOD structure.        //
//---------------------------------------------------------------------------//
extern "C" SERVOMOD * WINAPI __declspec(dllexport) ServoNewMod()
{
  SERVOMOD *p;

  p = new SERVOMOD;
  p->pos = 0;
  p->ad = 0;
  p->vel = 0;
  p->aux = 0;
  p->home = 0;
  p->perror = 0;
  p->cmdpos = 0;
  p->cmdvel = 0;
  p->cmdacc = 0;
  p->cmdpwm = 0;

  (p->gain).kp = 0;
  (p->gain).kd = 0;
  (p->gain).ki = 0;
  (p->gain).il = 0;
  (p->gain).ol = 0;
  (p->gain).cl = 0;
  (p->gain).el = 0;
  (p->gain).sr = 1;
  (p->gain).dc = 0;
  (p->gain).sm = 1;

  p->stoppos = 0;
  p->ioctrl = 0;
  p->homectrl = 0;
  p->movectrl = 0;
  p->stopctrl = 0;
  return p;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetStat (Internal Library Function)                 //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Processes and stores returned PIC-SERVO status data.     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoGetStat(byte addr)
{
  int numbytes, numrcvd;
  int i, bytecount;
  byte cksum;
  byte inbuf[20];
  SERVOMOD *p;
  char msgstr[80];

  p = (SERVOMOD *)(mod[addr].p);  //cast the data pointer to the right type

  //Find number of bytes to read:
  numbytes = 2;       //start with stat & cksum
  if ( (mod[addr].statusitems) & SEND_POS )	numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_AD ) 	numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_VEL ) 	numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_AUX ) 	numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_HOME )	numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_ID ) 	numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_PERROR ) numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_NPOINTS ) numbytes +=1;
  numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

  //Verify enough data was read
  if (numrcvd != numbytes)
    {
      sprintf(msgstr,"ServoGetStat (%d) failed to read chars",addr);
      ErrorMsgBox(msgstr);
      return false;
    }

  //Verify checksum:
  cksum = 0;
  for (i=0; i<numbytes-1; i++) cksum = (byte)(cksum + inbuf[i]);
  if (cksum != inbuf[numbytes-1])
    {
      sprintf(msgstr,"ServoGetStat(%d): checksum error",addr);
      ErrorMsgBox(msgstr);
      return false;
    }

  //Verify command was received intact before updating status data
  mod[addr].stat = inbuf[0];
  if (mod[addr].stat & CKSUM_ERROR)
    {
      ErrorMsgBox("Command checksum error!");
      return false;
    }

  //Finally, fill in status data
  bytecount = 1;
  if ( (mod[addr].statusitems) & SEND_POS )
    {
      p->pos = *( (long *)(inbuf + bytecount) );
      bytecount +=4;
    }
  if ( (mod[addr].statusitems) & SEND_AD )
    {
      p->ad = inbuf[bytecount];
      bytecount +=1;
    }
  if ( (mod[addr].statusitems) & SEND_VEL )
    {
      p->vel = *( (short int *)(inbuf + bytecount) );
      bytecount +=2;
    }
  if ( (mod[addr].statusitems) & SEND_AUX )
    {
      p->aux = inbuf[bytecount];
      bytecount +=1;
    }
  if ( (mod[addr].statusitems) & SEND_HOME )
    {
      p->home = *( (unsigned long *)(inbuf + bytecount) );
      bytecount +=4;
    }
  if ( (mod[addr].statusitems) & SEND_ID )
    {
      mod[addr].modtype = inbuf[bytecount];
      mod[addr].modver = inbuf[bytecount+1];
      bytecount +=2;
    }
  if ( (mod[addr].statusitems) & SEND_PERROR )
    {
      p->perror = *( (short int *)(inbuf + bytecount) );
      bytecount +=2;
    }
  if ( (mod[addr].statusitems) & SEND_NPOINTS )
    {
      p->npoints = inbuf[bytecount];
      //bytecount +=1;
    }

  return TRUE;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetPos                                              //
//  Return Value:   Returns the current motor position.                      //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current motor position (stored locally).     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetPos(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->pos;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetAD                                               //
//  Return Value:   Returns the current A/D value.                           //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current A/D value (stored locally).          //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetAD(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->ad;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetVel                                              //
//  Return Value:   Returns the current motor velocity.                      //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current motor velocity (stored locally).     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) short int ServoGetVel(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->vel;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetAux                                              //
//  Return Value:   Returns the current auxiliary status byte.               //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current auxiliary status byte (stored        //
//                  locally).                                                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetAux(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->aux;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetHome                                             //
//  Return Value:   Returns the current motor home position.                 //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current motor home position (stored locally).//
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetHome(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->home;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetPError                                           //
//  Return Value:   Returns the servo positioning error.                     //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the servo positioning error (stored locally).    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) short int ServoGetPError(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->perror;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetNPoints                                          //
//  Return Value:   Returns the number of path points remaining.             //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the number of path points remaining (stored      //
//                  locally).                                                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetNPoints(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->npoints;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetCmdPos                                           //
//  Return Value:   Returns the most recently issued command position.       //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command position.       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetCmdPos(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->cmdpos;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetCmdVel                                           //
//  Return Value:   Returns the most recently issued command velocity.       //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command velocity.       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetCmdVel(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->cmdvel;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetCmdAcc                                           //
//  Return Value:   Returns the most recently issued command acceleration.   //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command acceleration.   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetCmdAcc(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->cmdacc;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetStopPos                                          //
//  Return Value:   Returns the most recently issued stop position.          //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued stop position (by a     //
//                  ServoStopHere() command).                                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long ServoGetStopPos(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->stoppos;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetCmdPwm                                           //
//  Return Value:   Returns the most recently issued command PWM.            //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command PWM.            //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetCmdPwm(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->cmdpwm;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetIoCtrl                                           //
//  Return Value:   Returns most recently issued I/O command control byte.   //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued I/O command control     //
//                  byte.                                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetIoCtrl(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->ioctrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetHomeCtrl                                         //
//  Return Value:   Returns most recently issued home command control byte.  //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued home command control    //
//                  byte.                                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetHomeCtrl(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->homectrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetStopCtrl                                         //
//  Return Value:   Returns most recently issued stop command control byte.  //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued stop command control    //
//                  byte.                                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetStopCtrl(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->stopctrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetMoveCtrl                                         //
//  Return Value:   Returns most recently issued move command control byte.  //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued move command control    //
//                  byte.                                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte ServoGetMoveCtrl(byte addr)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  return p->movectrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetGain                                             //
//  Return Value:   None                                                     //
//  Parameters:     addr: module address (1-32)                              //
//                  kp: position gain Kp                                     //
//                  kd: derivative gain Kd                                   //
//                  ki: integral gain Ki                                     //
//                  il: integration limit IL                                 //
//                  ol: output limit OL                                      //
//                  cl: current limit CL                                     //
//                  el: position error limit EL                              //
//                  sr: servo rate divisor SR                                //
//                  dc: amplifier deadband compenstation DB                  //
//  Description:    Returns the most recently issued servo gain values for   //
//                  a PIC-SERVO module.  For PIC-SERVO SC modules, use       //
//                  ServoGetGain2().                                         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) void ServoGetGain(byte addr, short int * kp, short int * kd, short int * ki,
                                                          short int * il, byte * ol, byte * cl, short int * el,
                                                          byte * sr, byte * dc)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  *kp = (p->gain).kp;
  *kd = (p->gain).kd;
  *ki = (p->gain).ki;
  *il = (p->gain).il;
  *ol = (p->gain).ol;
  *cl = (p->gain).cl;
  *el = (p->gain).el;
  *sr = (p->gain).sr;
  *dc = (p->gain).dc;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoGetGain2                                            //
//  Return Value:   None                                                     //
//  Parameters:     addr: module address (1-32)                              //
//                  kp: position gain Kp                                     //
//                  kd: derivative gain Kd                                   //
//                  ki: integral gain Ki                                     //
//                  il: integration limit IL                                 //
//                  ol: output limit OL                                      //
//                  cl: current limit CL                                     //
//                  el: position error limit EL                              //
//                  sr: servo rate divisor SR                                //
//                  dc: amplifier deadband compenstation DB                  //
//                  sm: step rate multiplier SM                              //
//  Description:    Returns the most recently issued servo gain values for   //
//                  a PIC-SERVO SC module. For non-PIC-SERVO SC modules,     //
//                  use ServoGetGain().                                      //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) void ServoGetGain2(byte addr, short int * kp, short int * kd, short int * ki,
                                                           short int * il, byte * ol, byte * cl, short int * el,
                                                           byte * sr, byte * dc, byte * sm)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  *kp = (p->gain).kp;
  *kd = (p->gain).kd;
  *ki = (p->gain).ki;
  *il = (p->gain).il;
  *ol = (p->gain).ol;
  *cl = (p->gain).cl;
  *el = (p->gain).el;
  *sr = (p->gain).sr;
  *dc = (p->gain).dc;
  *sm = (p->gain).sm;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoSetGain                                             //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  kp: Position gain Kp (0 - +32,767)                       //
//                  kd: Derivative gain Kd (0 - +32,767)                     //
//                  ki: Integral gain Ki (0 - +32,767)                       //
//                  ol: Output limit OL (0 - 255)                            //
//                  cl: Current limit CL (0 - 255)                           //
//                     (odd values: CUR_SENSE proportional to motor current) //
//                     (even values: CUR_SENSE inv. prop. to motor current)  //
//                  el: Position error limit EL (0 - +32,767)                //
//                  sr: Servo rate divisor SR (1 - 255)                      //
//                  dc: Amplifier deadband compenstation DB (0 - 255)        //
//  Description:    Sets most of the non-motion related operating parameters //
//                  of the PIC-SERVO.  New applications should use           //
//                  ServoSetGain2() for all versions of PIC-SERVO.           //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoSetGain(byte addr, short int kp, short int kd, short int ki,
                                                          short int il, byte ol, byte cl, short int el,
                                                          byte sr, byte dc)
{
  SERVOMOD * p;
  char cmdstr[16];

  p = (SERVOMOD *)(mod[addr].p);
  (p->gain).kp = kp;
  (p->gain).kd = kd;
  (p->gain).ki = ki;
  (p->gain).il = il;
  (p->gain).ol = ol;
  (p->gain).cl = cl;
  (p->gain).el = el;
  (p->gain).sr = sr;
  (p->gain).dc = dc;

  *( (short int *)(cmdstr) ) = kp;
  *( (short int *)(cmdstr+2) ) = kd;
  *( (short int *)(cmdstr+4) ) = ki;
  *( (short int *)(cmdstr+6) ) = il;
  *( (byte *)(cmdstr+8) ) = ol;
  *( (byte *)(cmdstr+9) ) = cl;
  *( (short int *)(cmdstr+10) ) = el;
  *( (byte *)(cmdstr+12) ) = sr;
  *( (byte *)(cmdstr+13) ) = dc;

  return NmcSendCmd(addr, SET_GAIN, cmdstr, 14, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoSetGain2                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  kp: Position gain Kp (0 - +32,767)                       //
//                  kd: Derivative gain Kd (0 - +32,767)                     //
//                  ki: Integral gain Ki (0 - +32,767)                       //
//                  ol: Output limit OL (0 - 255)                            //
//                  cl: Current limit CL (0 - 255)                           //
//                     (odd values: CUR_SENSE proportional to motor current) //
//                     (even values: CUR_SENSE inv. prop. to motor current)  //
//                  el: Position error limit EL (0 - +32,767)                //
//                  sr: Servo rate divisor SR (1 - 255)                      //
//                  dc: Amplifier deadband compenstation DB (0 - 255)        //
//                  sm: Step rate multiplier (1 - 255)                       //
//  Description:    Sets most of the non-motion related operating parameters //
//                  of the PIC-SERVO.  New applications should use this      //
//                  version of the Set Gain command (rather than             //
//                  ServoSetGain()) for all versions of the PIC-SERVO.       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoSetGain2(byte addr, short int kp, short int kd, short int ki,
                                                           short int il, byte ol, byte cl, short int el,
                                                           byte sr, byte dc, byte sm)
{
  SERVOMOD * p;
  char cmdstr[16];

  p = (SERVOMOD *)(mod[addr].p);
  (p->gain).kp = kp;
  (p->gain).kd = kd;
  (p->gain).ki = ki;
  (p->gain).il = il;
  (p->gain).ol = ol;
  (p->gain).cl = cl;
  (p->gain).el = el;
  (p->gain).sr = sr;
  (p->gain).dc = dc;
  (p->gain).sm = sm;

  *( (short int *)(cmdstr) ) = kp;
  *( (short int *)(cmdstr+2) ) = kd;
  *( (short int *)(cmdstr+4) ) = ki;
  *( (short int *)(cmdstr+6) ) = il;
  *( (byte *)(cmdstr+8) ) = ol;
  *( (byte *)(cmdstr+9) ) = cl;
  *( (short int *)(cmdstr+10) ) = el;
  *( (byte *)(cmdstr+12) ) = sr;
  *( (byte *)(cmdstr+13) ) = dc;
  *( (byte *)(cmdstr+14) ) = sm;

  return NmcSendCmd(addr, SET_GAIN, cmdstr, 15, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoLoadTraj                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the load trajectory mode bits        //
//                        (see picservo.h)                                   //
//                  pos: Position data if LOAD_POS bit of mode is set        //
//                       (-2,147,483,648 - +2,147,483,647)                   //
//                  vel: Velocity data is LOAD_VEL bit of mode is set        //
//                       (0 - +83,886,080)                                   //
//                  acc: Acceleration data is LOAD_ACC bit of mode is set    //
//                       (0 - +2,147,483,647)                                //
//                  pwm: PWM data is LOAD_PWM bit of mode is set (0 - +255)  //
//  Description:    Loads motion trajectory and PWM information.             //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoLoadTraj(byte addr, byte mode, long pos, long vel, long acc, byte pwm)
{
  SERVOMOD * p;
  char cmdstr[16];
  int count;

  p = (SERVOMOD *)(mod[addr].p);
  p->movectrl = mode;
  p->cmdpos = pos;
  p->cmdvel = vel;
  p->cmdacc = acc;
  p->cmdpwm = pwm;

  count = 0;
  *( (byte *)(cmdstr + count) ) = mode;  count += 1;
  if (mode & LOAD_POS) { *( (long *)(cmdstr + count) ) = pos; count += 4; }
  if (mode & LOAD_VEL) { *( (long *)(cmdstr + count) ) = vel; count += 4; }
  if (mode & LOAD_ACC) { *( (long *)(cmdstr + count) ) = acc; count += 4; }
  if (mode & LOAD_PWM) { *( (byte *)(cmdstr + count) ) = pwm; count += 1; }

  return NmcSendCmd(addr, LOAD_TRAJ, cmdstr, (byte)count, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoInitPath                                            //
//  Return Value:   None                                                     //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Initializes the starting point of a path to the current  //
//                  motor position.                                          //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) void ServoInitPath(byte addr)
{
  SERVOMOD * p;

  NmcReadStatus(addr, SEND_POS | SEND_PERROR);

  p = (SERVOMOD *)(mod[addr].p);
  p->last_ppoint = p->pos + p->perror;
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoAddPathPoints                                       //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  npoints: number of points in list (1-7)                  //
//                  path: path points list of absolute position data         //
//                  freq: path point frequency (P_30HZ, P_60HZ, P-120HZ)     //
//  Description:    Adds a set of path points for path mode operation.  Use  //
//                  ServoStartPathMode() to start path mode.                 //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoAddPathpoints(byte addr, int npoints, long *path, int freq)
{
  SERVOMOD * p;
  char cmdstr[16];
  long diff;
  int rev;
  int i;

  //npoints must be greater than 0
  if (npoints <= 0) return 0;

  p = (SERVOMOD *)(mod[addr].p);

  for (i=0; i<npoints; i++)
    {
      diff = path[i] - p->last_ppoint;
      if (diff<0)
        {
          rev = 0x01;
          diff = -diff;
        }
      else rev = 0x00;

      //Scale the difference appropriately for path freq. used
      if (p->ioctrl & FAST_PATH)  //scale for 60/120 Hz fast path
        {
          if (freq == P_60HZ)
            {
              diff *= (256/32);
              diff |= 0x02;     //60 Hz -> set bit 1 = 1
            }
          else if (freq == P_120HZ) diff *= (256/16);
          else return(false);
        }
      else  //scale for 30/60 Hz slow path
        {
          if (freq == P_30HZ)
            {
              diff *= (256/64);
              diff |= 0x02;     //30 Hz -> set bit 1 = 1
            }
          else if (freq == P_60HZ) diff *= (256/32);
          else return(false);
        }

      diff |= rev;  //bit 0 = reverse bit

      *( (short int *)(cmdstr + 2*i) ) = (short int)diff;

      p->last_ppoint = path[i];
    }


  return NmcSendCmd(addr, ADD_PATHPOINT, cmdstr, (byte)(npoints*2), addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoStartPathMode                                       //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     groupaddr: module addr( 1-32) or group addr (0x80-0xFF)  //
//                  groupleader: group leader address                        //
//                    if groupaddr individual use: groupleader=groupaddr     //
//                    if groupaddr group use: groupleader=group leader addr  //
//                    (if no group leader use: groupleader = 0)              //
//  Description:    Starts execution of the path loaded into the internal    //
//                  path point buffer.                                       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoStartPathMode(byte groupaddr, byte groupleader)
{
  return NmcSendCmd(groupaddr, ADD_PATHPOINT, NULL, 0, groupleader);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoStartMove                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     groupaddr: module addr (1-32) or group addr (0x80-0xFF)  //
//                  groupleader: group leader address                        //
//                    if groupaddr individual use: groupleader=groupaddr     //
//                    if groupaddr group use: groupleader=group leader addr  //
//                    (if no group leader use: groupleader = 0)              //
//  Description:    Synchronously start motions that have been preloaded     //
//                  using ServoLoadTraj().                                   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoStartMove(byte groupaddr, byte groupleader)
{
  return NmcSendCmd(groupaddr, START_MOVE, NULL, 0, groupleader);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoResetPos                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module addr (1-32)                                 //
//  Description:    Resets the position counter to zero.                     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoResetPos(byte addr)
{
  return NmcSendCmd(addr, RESET_POS, NULL, 0, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoResetRelHome                                        //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Resets the position of a module relative to the home     //
//                  position register (home position is now zero position).  //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoResetRelHome(byte addr)
{
  byte mode;

  mode = REL_HOME;
  return NmcSendCmd(addr, RESET_POS, (char *)(&mode), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoSetPos                                              //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  pos: position (-2,147,483,648 - +2,147,483,647)          //
//  Description:    Sets the module position to the specified value.         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoSetPos(byte addr, long pos)
{
  char cmdstr[6];

  cmdstr[0] = SET_POS;          //mode byte for reset pos
  *( (long *)(cmdstr + 1) ) = pos;

  return NmcSendCmd(addr, RESET_POS, cmdstr, 5, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoClearBits                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Clears the latched status bits (OVERCURRENT and          //
//                  POS_ERROR bits in status byte, and POS_WRAP and          //
//                  SERVO_OVERRUN bits in auxiliary status byte).            //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoClearBits(byte addr)
{
  return NmcSendCmd(addr, CLEAR_BITS, NULL, 0, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoStopMotor                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of stop mode bits (see picservo.h)      //
//  Description:    Stop the motor in the manner specified by mode.          //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoStopMotor(byte addr, byte mode)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);

  mode &= (byte)(~STOP_HERE);
  p->stopctrl = mode;       //make sure STOP_HERE bit is cleared

  return NmcSendCmd(addr, STOP_MOTOR, (char *)(&mode), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoStopHere                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the stop mode bits (see picservo.h)  //
//                  pos: unprofiled command position                         //
//  Description:    Stop the motor at the specified position.                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoStopHere(byte addr, byte mode, long pos)
{
  SERVOMOD * p;
  char cmdstr[6];

  p = (SERVOMOD *)(mod[addr].p);

  p->stopctrl = mode;

  cmdstr[0] = mode;
  *( (long *)(cmdstr + 1) ) = pos;

  return NmcSendCmd(addr, STOP_MOTOR, cmdstr, 5, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoSetIoCtrl                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of I/O control bits (see picservo.h)    //
//  Description:    Controls the configuration of the LIMIT1 and LIMIT2      //
//                  I/O pins, as well as other miscellaneous functions.      //
//                  CAUTION: Use extreme care in setting the parameters for  //
//                  this command - incorrect settings could damage your      //
//                  amplifier or the PIC-SERVO chip.                         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoSetIoCtrl(byte addr, byte mode)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);

  p->ioctrl = mode;

  return NmcSendCmd(addr, IO_CTRL, (char *)(&mode), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoSetHoming                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the homing mode bits (see picservo.h)//
//  Description:    Sets homing mode parameters for capturing the home       //
//                  position.                                                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoSetHoming(byte addr, byte mode)
{
  SERVOMOD * p;

  p = (SERVOMOD *)(mod[addr].p);
  p->homectrl = mode;

  return NmcSendCmd(addr, SET_HOMING, (char *)(&mode), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  ServoHardReset                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of reset control bits (see picservo.h)  //
//  Description:    Reset the controller to it's power-up state and          //
//                  optionally store configuration data in EEPROM.           //
//                  (Only valid for PIC-SERVO SC - v.10 and greater)         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL ServoHardReset(byte addr, byte mode)
{
  return NmcSendCmd(addr, HARD_RESET, (char *)(&mode), 1, 0);
}

