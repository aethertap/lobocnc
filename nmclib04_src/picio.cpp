//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#include <vcl\vcl.h>
#pragma hdrstop

#include "sio_util.h"
#include "nmccom.h"
#include "picio.h"
//---------------------------------------------------------------------------//
extern NMCMOD	mod[]; 		//Array of modules
extern int nummod;
extern HANDLE ComPort;


//---------------------------------------------------------------------------//
//  Function Name:  IoNewMod (Internal Library Function)                     //
//  Return Value:   Pointer to new IOMOD structure                           //
//  Parameters:     None                                                     //
//  Description:    Returns pointer to an initialized IOMOD structure.       //
//---------------------------------------------------------------------------//
extern "C" IOMOD * WINAPI __declspec(dllexport) IoNewMod()
{
IOMOD *p;

p = new IOMOD;
p->inbits = 0;
p->ad1 = 0;
p->ad2 = 0;
p->ad3 = 0;
p->timer = 0;
p->inbits_s = 0;
p->timer_s = 0;
p->pwm1 = 0;
p->pwm2 = 0;
p->bitdir = 0x0FFF;
p->outbits = 0;
p->timermode = 0;
return p;
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetStat (Internal Library Function)                    //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Processes and stores returned PIC-IO status data.        //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoGetStat(byte addr)
{
int numbytes, numrcvd;
int i, bytecount;
byte cksum;
byte inbuf[20];
IOMOD *p;

p = (IOMOD *)(mod[addr].p);  //cast the data pointer to the right type

//Find number of bytes to read:
numbytes = 2;       //start with stat & cksum
if ( (mod[addr].statusitems) & SEND_INPUTS )	numbytes +=2;
if ( (mod[addr].statusitems) & SEND_AD1 ) 		numbytes +=1;
if ( (mod[addr].statusitems) & SEND_AD2 ) 		numbytes +=1;
if ( (mod[addr].statusitems) & SEND_AD3 ) 		numbytes +=1;
if ( (mod[addr].statusitems) & SEND_TIMER )		numbytes +=4;
if ( (mod[addr].statusitems) & SEND_ID ) 		numbytes +=2;
if ( (mod[addr].statusitems) & SEND_SYNC_IN )	numbytes +=2;
if ( (mod[addr].statusitems) & SEND_SYNC_TMR )	numbytes +=4;
numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

//Verify enough data was read
if (numrcvd != numbytes)
	{
    ErrorMsgBox("IoGetStat failed to read chars");
    return false;
    }

//Verify checksum:
cksum = 0;
for (i=0; i<numbytes-1; i++) cksum = (byte)(cksum + inbuf[i]);
if (cksum != inbuf[numbytes-1])
	{
    ErrorMsgBox("IoGetStat: checksum error");
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
if ( (mod[addr].statusitems) & SEND_INPUTS )
	{
	p->inbits = *( (short int *)(inbuf + bytecount) );
    bytecount +=2;
    }
if ( (mod[addr].statusitems) & SEND_AD1 )
	{
    p->ad1 = inbuf[bytecount];
    bytecount +=1;
    }
if ( (mod[addr].statusitems) & SEND_AD2 )
	{
    p->ad2 = inbuf[bytecount];
    bytecount +=1;
    }
if ( (mod[addr].statusitems) & SEND_AD3 )
	{
    p->ad3 = inbuf[bytecount];
    bytecount +=1;
    }
if ( (mod[addr].statusitems) & SEND_TIMER )
	{
    p->timer = *( (unsigned long *)(inbuf + bytecount) );
    bytecount +=4;
    }
if ( (mod[addr].statusitems) & SEND_ID )
	{
    mod[addr].modtype = inbuf[bytecount];
    mod[addr].modver = inbuf[bytecount+1];
    bytecount +=2;
    }
if ( (mod[addr].statusitems) & SEND_SYNC_IN )
	{
    p->inbits_s = *( (short int *)(inbuf + bytecount) );
    bytecount +=2;
    }
if ( (mod[addr].statusitems) & SEND_SYNC_TMR )
	{
    p->timer_s = *( (unsigned long *)(inbuf + bytecount) );
    //bytecount +=4;
    }

return TRUE;
}


//---------------------------------------------------------------------------//
//  Function Name:  IoInBitVal                                               //
//  Return Value:   Value of the specified input bit.                        //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to be examined (0-11)                 //
//  Description:    Returns the value of an input bit (stored locally).      //
//                  Note: this data is only valid if the SEND_INPUTS bit has //
//                  been set in the most recently issued NmcDefineStatus()   //
//                  command.                                                 //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoInBitVal(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);
return ((p->inbits >> bitnum) & 1);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoInBitSVal                                              //
//  Return Value:   Value of the synchronously captured input bit.           //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to be examined (0-11)                 //
//  Description:    Returns the value of a synchronously captured input      //
//                  bit (stored locally).  Note: this data is only valid if  //
//                  the SEND_SYNCH_IN bit has been set in the most recently  //
//                  issued NmcDefineStatus() command.                        //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoInBitSVal(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);
return ((p->inbits_s >> bitnum) & 1);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoOutBitVal                                              //
//  Return Value:   The most recently set state (0 or 1) of an output bit.   //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to be examined (0-11)                 //
//  Description:    Returns the most recently set state of an output bit     //
//                  of a PIC-IO module.                                      //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoOutBitVal(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);
return ((p->outbits >> bitnum) & 1);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoSetOutBit                                              //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to set (0-11)                         //
//  Description:    Sets the value of an output bit to 1. This has no effect //
//                  if the bit is defined as an input.                       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoSetOutBit(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->outbits = p->outbits | (short int)(1 << bitnum);

return NmcSendCmd(addr, SET_OUTPUT, (char *)(&(p->outbits)), 2, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoClrOutBit                                              //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to clear (0-11)                       //
//  Description:    Clears the value of the specified output bit to 0.       //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoClrOutBit(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->outbits = p->outbits & (short int)(~(1 << bitnum));

return NmcSendCmd(addr, SET_OUTPUT, (char *)(&(p->outbits)), 2, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetBitDir                                              //
//  Return Value:   0=bit defined as an output, 1=bit defined as an input    //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to be examined (0-11)                 //
//  Description:    Returns 0 if I/O bit is defined as an output, 1 if I/O   //
//                  bit defined as an input.                                 //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoGetBitDir(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);
return ((p->bitdir >> bitnum) & 1);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoBitDirOut                                              //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address                                     //
//                  bitnum: bit number to set (0-11)                         //
//  Description:    Sets the direction of an I/O bit to be an output bit.    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoBitDirOut(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->bitdir = p->bitdir & (short int)(~(1 << bitnum));

return NmcSendCmd(addr, SET_IO_DIR, (char *)(&(p->bitdir)), 2, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoBitDirIn                                               //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  bitnum: bit number to set (0-11)                         //
//  Description:    Sets the direction of an I/O bit to be an input bit.     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoBitDirIn(byte addr, int bitnum)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->bitdir = p->bitdir | (short int)(1 << bitnum);

return NmcSendCmd(addr, SET_IO_DIR, (char *)(&(p->bitdir)), 2, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetADCVal                                              //
//  Return Value:   ADC value (0-255), or 0 for invalid channel              //
//  Parameters:     addr: module address (1-32)                              //
//                  channel: ADC channel (01, or 2)                          //
//  Description:    Returns the A/D value from channel 0, 1, or 2 (stored    //
//                  locally).  Note: this data is only valid if the          //
//                  SEND_ADn (n=1,2,3) has been set in the most recently     //
//                  issued NMCDefineStatus() command.                        //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte IoGetADCVal(byte addr, int channel)
{
IOMOD * p;
p = (IOMOD *)(mod[addr].p);

switch (channel) {
	case 0: return p->ad1;
	case 1: return p->ad2;
	case 2: return p->ad3; 
    }

return 0;
}


//---------------------------------------------------------------------------//
//  Function Name:  IoSetPWMVal                                              //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  pwm1: output value on pin PWM1 (0-255). Value of 255     //
//                        corresponds to 100% duty cycle, and 0 corresponds  //
//                        to 0% duty cycle.                                  //
//                  pwm2: output value on pin PWM2 (0-255). Value of 255     //
//                        corresponds to 100% duty cycle, and 0 corresponds  //
//                        to 0% duty cycle.                                  //
//  Description:    Sets the PWM output values.                              //
//---------------------------------------------------------------------------
extern "C" WINAPI __declspec(dllexport) BOOL IoSetPWMVal(byte addr, byte pwm1, byte pwm2)
{
IOMOD * p;
char cmdstr[4];

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->pwm1 = pwm1;
p->pwm2 = pwm2;
cmdstr[0] = pwm1;
cmdstr[1] = pwm2;

return NmcSendCmd(addr, SET_PWM, cmdstr, 2, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoSetSynchOutput                                         //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  outbits: output bit values (bits 0-11).  Setting a bit   //
//                        will cause corresponding pin to go HI, clearing    //
//                        a bit will cause the pin to go LOW.                //
//                  pwm1: output value on pin PWM1 (0-255). Value of 255     //
//                        corresponds to 100% duty cycle, and 0 corresponds  //
//                        to 0% duty cycle.                                  //
//                  pwm2: output value on pin PWM2 (0-255). Value of 255     //
//                        corresponds to 100% duty cycle, and 0 corresponds  //
//                        to 0% duty cycle.                                  //
//  Description:    Stores output bit values and PWM value in PIC-IO internal//
//                  registers to be set synchronously when the               //
//                  NmcSynchOutput() function is called.                     //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoSetSynchOutput(byte addr, short int outbits, byte pwm1, byte pwm2)
{
IOMOD * p;
char cmdstr[5];

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->outbits =  outbits;
p->pwm1 = pwm1;
p->pwm2 = pwm2;
cmdstr[0] = ((char *)(&outbits))[0];
cmdstr[1] = ((char *)(&outbits))[1];
cmdstr[2] = pwm1;
cmdstr[3] = pwm2;
return NmcSendCmd(addr, SET_SYNCH_OUT, cmdstr, 4, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetPWMVal                                              //
//  Return Value:   Returns the PWM value (0-255) of the specified channel   //
//  Parameters:     addr: module address (1-32)                              //
//                  channel: PWM channel (0 or 1)                            //
//  Description:    Returns the most recently set PWM value for chan 0 or 1. //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte IoGetPWMVal(byte addr, int channel)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
if (channel == 0) return(p->pwm1);
else return(p->pwm2);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoSetTimerMode                                           //
//  Return Value:   0=Fail, 1=Success                                        //
//  Parameters:     addr: module address (1-32)                              //
//                  tmrmode: logical OR of timer mode bits (see picio.h)     //
//  Description:    Sets operating mode of counter/timer.                    //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) BOOL IoSetTimerMode(byte addr, byte tmrmode)
{
IOMOD * p;
char cmdstr[2];

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
p->timermode = tmrmode;
cmdstr[0] = tmrmode;

return NmcSendCmd(addr, SET_TMR_MODE, cmdstr, 1, addr);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetTimerMode                                           //
//  Return Value:   Returns the most recently set timer control byte         //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the most recently set timer control byte.        //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) byte IoGetTimerMode(byte addr)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
return p->timermode;
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetTimerVal                                            //
//  Return Value:   Returns the timer value                                  //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the timer value (stored locally) from a PIC-IO   //
//                  module. Note: this data is only valid if the SEND_TIMER  //
//                  bit has been set in the most recently issued             //
//                  NmcDefineStatus() command.                               //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) unsigned long IoGetTimerVal(byte addr)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
return(p->timer);
}


//---------------------------------------------------------------------------//
//  Function Name:  IoGetTimerSVal                                           //
//  Return Value:   Returns the synchronously captured timer value           //
//  Parameters:     addr: module address (1-32)                              //
//  Description:    Returns the synchronously captured timer value (stored   //
//                  locally) from a PIC-IO module.  Note: this data is only  //
//                  valid if the SEND_SYNCH_TMR bit has been set in the most //
//                  recently issued NmcDefineStatus() command.               //
//---------------------------------------------------------------------------//
extern "C" WINAPI __declspec(dllexport) unsigned long IoGetTimerSVal(byte addr)
{
IOMOD * p;

p = (IOMOD *)(mod[addr].p);  			//Point to the IO data structure
return(p->timer_s);
}
//---------------------------------------------------------------------------//

