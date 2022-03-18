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
#include "picstep.h"
//---------------------------------------------------------------------------//
extern NMCMOD	mod[]; 		//Array of modules
extern int nummod;
extern HANDLE ComPort;


//---------------------------------------------------------------------------//
//  Function Name:  StepNewMod (Internal Library Function)                   //
//  Return Value:   Pointer to the new STEPMOD structure                     //
//  Parameters:     None                                                     //
//  Description:    Creates and initializes a new STEPMOD structure.         //
//---------------------------------------------------------------------------//
//Returns pointer to an initialized IOMOD structure
extern "C" STEPMOD * WINAPI __declspec(dllexport) StepNewMod()
{
STEPMOD *p;

p = new STEPMOD;
p->pos = 0;
p->ad = 0;
p->st = 0;
p->inbyte = 0;
p->home = 0;

p->cmdpos = 0;
p->cmdspeed = 1;
p->cmdacc = 1;
p->cmdst = 0;
p->min_speed = 1;
p->outbyte = 0;
p->homectrl = 0;
p->ctrlmode = SPEED_1X | ESTOP_OFF;
p->stopctrl = 0;
p->run_pwm = 0;
p->hold_pwm = 0;
p->therm_limit = 0;
return p;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetStat (Internal Library Function)                  //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Process and store the status data returned from a        //
//                  PIC-STEP module.                                         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepGetStat(byte addr)
{
int numbytes, numrcvd;
int i, bytecount;
byte cksum;
byte inbuf[20];
STEPMOD *p;
char msgstr[80];

p = (STEPMOD *)(mod[addr].p);  //cast the data pointer to the right type

//Find number of bytes to read:
numbytes = 2;       //start with stat & cksum
if ( (mod[addr].statusitems) & SEND_POS )	numbytes +=4;
if ( (mod[addr].statusitems) & SEND_AD ) 	numbytes +=1;
if ( (mod[addr].statusitems) & SEND_ST ) 	numbytes +=2;
if ( (mod[addr].statusitems) & SEND_INBYTE ) numbytes +=1;
if ( (mod[addr].statusitems) & SEND_HOME )	numbytes +=4;
if ( (mod[addr].statusitems) & SEND_ID ) 	numbytes +=2;
numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

//Verify enough data was read
if (numrcvd != numbytes)
	{
    sprintf(msgstr,"StepGetStat (%d) failed to read chars",addr);
    ErrorMsgBox(msgstr);
    return false;
    }

//Verify checksum:
cksum = 0;
for (i=0; i<numbytes-1; i++) cksum = (byte)(cksum + inbuf[i]);
if (cksum != inbuf[numbytes-1])
	{
    sprintf(msgstr,"StepGetStat(%d): checksum error",addr);
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
if ( (mod[addr].statusitems) & SEND_ST )
	{
	p->st = *( (unsigned short int *)(inbuf + bytecount) );
    bytecount +=2;
    }
if ( (mod[addr].statusitems) & SEND_INBYTE )
	{
    p->inbyte = inbuf[bytecount];
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
    //bytecount +=2;
    }

return TRUE;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetPos                                               //
//  Return Value:   Returns the current motor position.                      //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current motor position (stored locally).     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long StepGetPos(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->pos;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetAD                                                //
//  Return Value:   Returns the current A/D value.                           //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current A/D value (stored locally).          //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetAD(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->ad;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetStepTime                                          //
//  Return Value:   Returns the current timer count.                         //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current timer count (stored locally).        //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) unsigned short int StepGetStepTime(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->st;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetInByte                                            //
//  Return Value:   Returns the current input byte.                          //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current input byte (stored locally).  This   //
//                  includes the E-stop, limit switch, homing switch and     //
//                  auxiliary input bits.                                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetInbyte(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->inbyte;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetHome                                              //
//  Return Value:   Returns the current motor home position.                 //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the current motor home position (stored locally).//
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long StepGetHome(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->home;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetCmdPos                                            //
//  Return Value:   Returns the most recently issued command position.       //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command position.       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) long StepGetCmdPos(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->cmdpos;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetCmdSpeed                                          //
//  Return Value:   Returns the most recently issued command speed.          //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command speed.          //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetCmdSpeed(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->cmdspeed;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetCmdAcc                                            //
//  Return Value:   Returns the most recently issued command acceleration.   //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command acceleration.   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetCmdAcc(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->cmdacc;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetCmdST                                             //
//  Return Value:   Returns the most recently issued command timer count.    //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command timer count.    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) unsigned short int StepGetCmdST(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->cmdst;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetMinSpeed                                          //
//  Return Value:   Returns the minimum stepping speed.                      //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the minimum stepping speed.                      //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetMinSpeed(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->min_speed;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetOutputs                                           //
//  Return Value:   Returns the most recently issued command output byte.    //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently issued command output byte.    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetOutputs(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->outbyte;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetCtrlMode                                          //
//  Return Value:   Returns the control mode byte.                           //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the control mode byte (set with StepSetParam()). //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetCtrlMode(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->ctrlmode;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetRunCurrent                                        //
//  Return Value:   Returns the running current.                             //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the running current (set with StepSetParam()).   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetRunCurrent(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->run_pwm;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetHoldCurrent                                       //
//  Return Value:   Returns the holding current.                             //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the holding current (set with StepSetParam()).   //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetHoldCurrent(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->hold_pwm;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetThermLimit                                        //
//  Return Value:   Returns the thermal limit.                               //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the thermal limit (set with StepSetParam()).     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetThermLimit(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->therm_limit;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetHomeCtrl                                          //
//  Return Value:   Returns the homing control byte.                         //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the homing control byte.                         //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetHomeCtrl(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->homectrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepGetStopCtrl                                          //
//  Return Value:   Returns the stopping control byte.                       //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the stopping control byte.                       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte StepGetStopCtrl(byte addr)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
return p->stopctrl;
}


//---------------------------------------------------------------------------//
//  Function Name:  StepSetParam                                             //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of set parameter mode bits              //
//                        (see picstep.h)                                    //
//                  minspeed: minimum stepping speed (1-250)                 //
//                  runcur: running current limit (0-255)                    //
//                  holdcur: holding current limit (0-255)                   //
//                  thermlim: thermal limit (0-255)                          //
//  Description:    Sets PIC-STEP operating parameters.  This command must   //
//                  issued before any motions can be executed.               //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepSetParam(byte addr, byte mode,
					byte minspeed, byte runcur, byte holdcur, byte thermlim)
{
STEPMOD * p;
char cmdstr[16];

p = (STEPMOD *)(mod[addr].p);
p->ctrlmode = mode;
p->min_speed = minspeed;
p->run_pwm = runcur;
p->hold_pwm = holdcur;
p->therm_limit = thermlim;

cmdstr[0] = mode;
cmdstr[1] = minspeed;
cmdstr[2] = runcur;
cmdstr[3] = holdcur;
cmdstr[4] = thermlim;

return NmcSendCmd(addr, SET_PARAM, cmdstr, 5, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  StepLoadTraj                                             //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the load trajectory mode bits        //
//                        (see picstep.h)                                    //
//                  pos: position data if LOAD_POS bit of mode is set        //
//                       (signed 32 bit int: -2,147,483,648 - +2,147,483,647)//
//                  speed: speed data if LOAD_SPEED bit of mode is set       //
//                       (8 bit int: 1 - 250)                                //
//                  acc: acceleration data if LOAD_ACC bit of mode is set    //
//                       (8 bit int: 1-255; larger values = slower accel)    //
//                  raw_speed: speed data if LOAD_ST bit of mode is set      //
//                       (float: 0.4 - 250.0)                                //
//  Description:    Load motion trajectory information.                      //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepLoadTraj(byte addr, byte mode,
			long pos, byte speed, byte acc, float raw_speed)
{
STEPMOD * p;
char cmdstr[16];
unsigned short int steptime;
byte nearspeed;
int count;

if (raw_speed < 0.4) raw_speed = 0.4;
if (raw_speed > 250.0) raw_speed = 250.0;
if (speed < 1 ) speed = 1;
if (acc < 1 ) acc = 1;

p = (STEPMOD *)(mod[addr].p);

steptime = (unsigned short int)(0x10000 - (unsigned)(25000.0/raw_speed));

//Adjust steptime for the fixed off time of the hardware timer
if ( (p->ctrlmode & 0x03)==SPEED_8X ) steptime += (byte)16;
else if ( (p->ctrlmode & 0x03)==SPEED_4X ) steptime += (byte)8;
else if ( (p->ctrlmode & 0x03)==SPEED_2X ) steptime += (byte)4;
else if ( (p->ctrlmode & 0x03)==SPEED_1X ) steptime += (byte)2;

nearspeed = (unsigned short int)(raw_speed + 0.5);

count = 0;
*( (byte *)(cmdstr + count) ) = mode;  count += 1;
if (mode & LOAD_POS)
  {
  p->cmdpos = pos;
  *( (long *)(cmdstr + count) ) = pos;
  count += 4;
  }
if (mode & LOAD_SPEED)
  {
  p->cmdspeed = speed;
  *( (byte *)(cmdstr + count) ) = speed;
  count += 1;
  }
if (mode & LOAD_ACC)
  {
  p->cmdacc = acc;
  *( (byte *)(cmdstr + count) ) = acc;
  count += 1;
  }
if (mode & LOAD_ST)
  {
  p->cmdst = steptime;
  *( (short int *)(cmdstr + count) ) = steptime;
  count += 2;
  *( (byte *)(cmdstr + count) ) = nearspeed;
  count += 1;
  }

return NmcSendCmd(addr, LOAD_TRAJ, cmdstr, (byte)count, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  StepResetPos                                             //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Resets the position counter to zero.                     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepResetPos(byte addr)
{
return NmcSendCmd(addr, RESET_POS, NULL, 0, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  StepStopMotor                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the stop motor control bits          //
//                        (see picstep.h)                                    //
//  Description:    Stops a motor in the manner specified by mode            //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepStopMotor(byte addr, byte mode)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);

p->stopctrl = mode;

return NmcSendCmd(addr, STOP_MOTOR, (char *)(&mode), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  StepSetOutputs                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  outbyte:  output bit values (bits 0-4).  Setting bit     //
//                            causes corresponding pin to go HI, clearing    //
//                            causes corresponding pin to go LOW.            //
//  Description:   Sets or clears the general purpose output pins.           //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepSetOutputs(byte addr, byte outbyte)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);

p->outbyte = outbyte;

return NmcSendCmd(addr, SET_OUTPUTS, (char *)(&outbyte), 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  StepSetHoming                                            //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  mode: logical OR of the homing mode bits                 //
//                        (see picstep.h)                                    //
//  Description:    Set homing mode parameters for capturing the home        //
//                  position.                                                //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL StepSetHoming(byte addr, byte mode)
{
STEPMOD * p;

p = (STEPMOD *)(mod[addr].p);
p->homectrl = mode;

return NmcSendCmd(addr, SET_HOMING, (char *)(&mode), 1, addr);
}
//---------------------------------------------------------------------------

