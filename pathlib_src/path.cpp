//---------------------------------------------------------------------------
#include <vcl\vcl.h>
#pragma hdrstop

#include <math.h>
#include "path.h"
#include "nmccom.h"
#include "sio_util.h"
#include "picservo.h"
//---------------------------------------------------------------------------
//Globals:
int pathsize;				//local path size
segment seglist[MAXSEG];    //list of segments
int seglistsize;            //number of segments in list
int curseg;                 //current segment number
int curppoint;				//current pathpoint
double tan_tolerance = TAN_3DEGREE;    //minimum cos(th) for angle between tangents
frame cur_arcframe;         //coordinate fram for the current arc
double pathlen;      		//total path length
double arclen;                //total length of path already downloaded
double stublen;              //length of path in current segment
int pathfreq = P_30HZ;		//selected path frequency
int bufsize;   				//max num points to store in the PIC-SERVO buffer
double maxvel, vel, acc;		//max. velocity, cur velocity & acceleration
byte x = 1;  				//axes addresses
byte y = 2;
byte z = 3;
byte group = 0xFF;			//group address for coordinated controllers
byte leader = 0x00;			//group leader address for coordinated controllers
int finaldecel;				//flag for when final deceleration has started
int at_end;  				//flags when we are at end of path
double xoff, yoff, zoff;  	//origin offset
double UTOCX, UTOCY, UTOCZ;	//Units TO Counts conversion factors
double tolerance;			//small distance tolerance use for near-zero comparisons

//---------------------------------------------------------------------------
//  Geometric functions
//---------------------------------------------------------------------------
//Returns the magnitude of a floating point vector
double mag(fp p)
{
return(sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]));
}
//---------------------------------------------------------------------------
//Returns the dot product of two floating point vectors
double dot(fp x, fp y)
{
return( x[0]*y[0] + x[1]*y[1] + x[2]*y[2] );
}
//---------------------------------------------------------------------------
//Z is returned as the cross product of (x cross y)
void cross(fp x, fp y, fp z)
{
z[0] = x[1]*y[2] - x[2]*y[1];
z[1] = x[2]*y[0] - x[0]*y[2];
z[2] = x[0]*y[1] - x[1]*y[0];
}
//---------------------------------------------------------------------------
//y = normalized x   ( normalize(x,x) works OK )
double normalize(fp x, fp y)
{
double a;

a = mag(x);
if (a==0.0) return(a);

y[0] /= a;
y[1] /= a;
y[2] /= a;
return(a);
}
//---------------------------------------------------------------------------
//Cooredinate frame transformation: y =  F*x, ( y = F*y OK )
void fvmult(frame *F, fp x, fp y)
{
fp xtemp;

xtemp[0] = x[0];
xtemp[1] = x[1];
xtemp[2] = x[2];
y[0] = F->x[0]*xtemp[0] + F->y[0]*xtemp[1] + F->z[0]*xtemp[2] + F->p[0];
y[1] = F->x[1]*xtemp[0] + F->y[1]*xtemp[1] + F->z[1]*xtemp[2] + F->p[1];
y[2] = F->x[2]*xtemp[0] + F->y[2]*xtemp[1] + F->z[2]*xtemp[2] + F->p[2];
}
//---------------------------------------------------------------------------
// Coordinate frame inversion: *B = inverse(A)
void finvert(frame A, frame *B)
{
B->x[1] = A.y[0];
B->x[2] = A.z[0];
B->y[2] = A.z[1];
B->y[0] = A.x[1];
B->z[0] = A.x[2];
B->z[1] = A.y[2];
B->x[0] = A.x[0];
B->y[1] = A.y[1];
B->z[2] = A.z[2];
B->p[0] = -A.x[0]*A.p[0] - A.y[0]*A.p[1] - A.z[0]*A.p[2];
B->p[1] = -A.x[1]*A.p[0] - A.y[1]*A.p[1] - A.z[1]*A.p[2];
B->p[2] = -A.x[2]*A.p[0] - A.y[2]*A.p[1] - A.z[2]*A.p[2];
}
//---------------------------------------------------------------------------
//Get a normal vector tangent to a segment endpoint (=1 for p1, =2 for p2)
//returns -1 if segment length or radius < tolerance, 0 on success
int GetTanVect(segment *s, fp p, int endpoint)
{
fp q;

if (s->type == LINE)
  {
  p[0] = s->p2[0] - s->p1[0];
  p[1] = s->p2[1] - s->p1[1];
  p[2] = s->p2[2] - s->p1[2];
  if (normalize(p, p) < tolerance) return(-1);
  else return(0);
  }
else if (s->type == ARC)
  {
  if (endpoint == 1)
    {
    q[0] = s->p1[0] - s->c[0];
    q[1] = s->p1[1] - s->c[1];
    q[2] = s->p1[2] - s->c[2];
    }
  else
    {
    q[0] = s->p2[0] - s->c[0];
    q[1] = s->p2[1] - s->c[1];
    q[2] = s->p2[2] - s->c[2];
    }
  if (normalize(q, q) < tolerance) return(-1);
  cross(s->norm, q, p);
  return(0);
  }

return(0);
}
//---------------------------------------------------------------------------
// Segment list functions
//---------------------------------------------------------------------------
//Theta = allowable angle (in degrees) between continuous path segments
extern "C" WINAPI __declspec(dllexport) void SetTangentTolerance(double theta)
{
tan_tolerance = cos(theta*DTOR);
}
//---------------------------------------------------------------------------
// Clear the current segment list and initialize the starting point
extern "C" WINAPI __declspec(dllexport) void ClearSegList(double xi, double yi, double zi)
{
seglistsize = 0;
seglist[0].p1[0] = xi;
seglist[0].p1[1] = yi;
seglist[0].p1[2] = zi;
}
//---------------------------------------------------------------------------
//Extract the reference frame for an arc
//Also fills in the radius and arclength
void GetArcFrame(segment *seg, frame *F)
{
double q, theta;
fp p;

if (seg->type != ARC) return;   //punt if not an arc

F->p[0] = seg->c[0];		//origin is at the center
F->p[1] = seg->c[1];
F->p[2] = seg->c[2];
F->x[0] = seg->p1[0] - seg->c[0];   //X vector points from center to P1
F->x[1] = seg->p1[1] - seg->c[1];
F->x[2] = seg->p1[2] - seg->c[2];
seg->r = normalize(F->x, F->x);  //extract radius and normalize
q = dot(F->x, seg->norm);     //make sure normal vector is perp. to X
F->z[0] = seg->norm[0] - q*F->x[0];
F->z[1] = seg->norm[1] - q*F->x[1];
F->z[2] = seg->norm[2] - q*F->x[2];
normalize(F->z, F->z);
cross(F->z, F->x, F->y);

p[0] =  seg->p2[0] - seg->c[0];	//get the arclength
p[1] =  seg->p2[1] - seg->c[1];
p[2] =  seg->p2[2] - seg->c[2];
theta = atan2( dot(p,F->y), dot(p,F->x) );
if (theta < 0.0) theta = TWOPI + theta;
if (fabs(theta) < 0.001) theta = TWOPI - theta;
seg->len = fabs(seg->r * theta);
}
//---------------------------------------------------------------------------
//Add a line segment to the segment list
//Returns: position in segment list if OK
//         -1 if segment is not tangent
//         -2 if segment list is full
//Function assumes the normal vector of any previous arc segment is accurate
extern "C" WINAPI __declspec(dllexport) int AddLineSeg(double x, double y, double z)
{
fp pn, qn;

if (seglistsize >= MAXSEG) return(-2);

seglist[seglistsize].type = LINE;

seglist[seglistsize].p2[0] = x;
seglist[seglistsize].p2[1] = y;
seglist[seglistsize].p2[2] = z;

if (seglistsize > 0)   //match start point to end point of prev segment
  {
  seglist[seglistsize].p1[0] = seglist[seglistsize-1].p2[0];
  seglist[seglistsize].p1[1] = seglist[seglistsize-1].p2[1];
  seglist[seglistsize].p1[2] = seglist[seglistsize-1].p2[2];
  }

//Calculate normal vector and length for this segment
pn[0] = x - seglist[seglistsize].p1[0];
pn[1] = y - seglist[seglistsize].p1[1];
pn[2] = z - seglist[seglistsize].p1[2];
seglist[seglistsize].len = normalize(pn, pn);

//Check tangency with prev. segment for segments > tolerance:
if ( (seglist[seglistsize].len > tolerance) && (seglistsize > 0) )
  if ( GetTanVect( &(seglist[seglistsize-1]), qn, 2) == 0 )
    if (dot(pn,qn) < tan_tolerance) return(-1);

seglistsize++;
return(seglistsize - 1);
}
//---------------------------------------------------------------------------
//Add an arc segment to the segment list
//Returns: position in segment list if OK
//         -1 if segment is not tangent
//         -2 if segment list is full
//		   -3 if arc data invalid
//(Invalid arc data - zero len. normal, radius < tolerance, normal not perp.)
//Function assumes the normal vector of any previous arc segment is accurate
extern "C" WINAPI __declspec(dllexport) int AddArcSeg( double x, double y, double z,        //end point
               double cx, double cy, double cz,     //center point
               double nx, double ny, double nz )    //normal
{
fp pn, qn;
frame F;

if (seglistsize >= MAXSEG) return(-2);

seglist[seglistsize].type = ARC;

seglist[seglistsize].p2[0] = x;
seglist[seglistsize].p2[1] = y;
seglist[seglistsize].p2[2] = z;
seglist[seglistsize].c[0] = cx;
seglist[seglistsize].c[1] = cy;
seglist[seglistsize].c[2] = cz;
seglist[seglistsize].norm[0] = nx;
seglist[seglistsize].norm[1] = ny;
seglist[seglistsize].norm[2] = nz;

if (seglistsize > 0)   //match start point to end point of prev segment
  {
  seglist[seglistsize].p1[0] = seglist[seglistsize-1].p2[0];
  seglist[seglistsize].p1[1] = seglist[seglistsize-1].p2[1];
  seglist[seglistsize].p1[2] = seglist[seglistsize-1].p2[2];
  }

//Normalize n and punt if too small
if ( normalize(seglist[seglistsize].norm, seglist[seglistsize].norm) < tolerance )
  return(-3);

//Find radius to p2, and punt if too small
pn[0] = x - cx;
pn[1] = y - cy;
pn[2] = z - cz;
seglist[seglistsize].r = normalize(pn, pn);
if ( seglist[seglistsize].r < tolerance) return(-3);

//Check if normal is perp to c->p2 vector and punt if not
if ( fabs(dot(seglist[seglistsize].norm, pn)) > 0.01 ) return(-3);

//Find radius to p1, and punt if not equal to radius to p2
pn[0] = seglist[seglistsize].p1[0] - cx;
pn[1] = seglist[seglistsize].p1[1] - cy;
pn[2] = seglist[seglistsize].p1[2] - cz;
if ( fabs(seglist[seglistsize].r - normalize(pn, pn)) > tolerance) return(-3);

//Check if normal is perp to c->p2 vector and punt if not
if ( fabs(dot(seglist[seglistsize].norm, pn)) > 0.01 ) return(-3);

//Check for tangency with prev segment
if ( seglistsize > 0 )
  {
  GetTanVect( &(seglist[seglistsize]), pn, 1);    //get current tangent
  if ( GetTanVect( &(seglist[seglistsize-1]), qn, 2) == 0 )   //get prev tangent
    if (dot(pn,qn) < tan_tolerance) return(-1);
  }

GetArcFrame(&(seglist[seglistsize]), &F);  //fills in segment length

seglistsize++;
return(seglistsize - 1);
}
//---------------------------------------------------------------------------
//Returns a point p which lies on a line segment, and is a distance s from
//the start of the line segment
void GetLineSegPoint(segment *seg, double s, fp p)
{
double r;

r = s/seg->len;
p[0] = seg->p1[0] + r*(seg->p2[0] - seg->p1[0]);
p[1] = seg->p1[1] + r*(seg->p2[1] - seg->p1[1]);
p[2] = seg->p1[2] + r*(seg->p2[2] - seg->p1[2]);
}
//---------------------------------------------------------------------------
//Set feedrate in units per second
extern "C" WINAPI __declspec(dllexport) void SetFeedrate(double fr)
{
switch (pathfreq)       //calculate velocity in units per tick
	{
    case P_30HZ: maxvel = fr/30.0;
    			 break;
    case P_60HZ: maxvel = fr/60.0;
    			 break;
    case P_120HZ: maxvel = fr/120.0;
    			 break;
    default: maxvel = fr/30.0;
    }
}
//---------------------------------------------------------------------------
//Set the origin to which all segment data is relative
extern "C" WINAPI __declspec(dllexport) void SetOrigin(double xoffset, double yoffset, double zoffset)
{
xoff = xoffset;  		//origin offset
yoff = yoffset;
zoff = zoffset;
}
//---------------------------------------------------------------------------
//Initialize various parameters for this path generation module
//Returns -1 if Status items are not set properly,
//        -2 if scale facotrs are zero
extern "C" WINAPI __declspec(dllexport) int SetPathParams2(int freq, int nbuf,
                   int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
                   double xscale, double yscale, double zscale,
                   double accel )
{
byte statitems;
byte ioctrl;

//SimpleMsgBox(" PIC-SERVO Coordinated Control Example\n\n- FOR EVALUATION PURPOSES ONLY -");

pathfreq = freq;        //set to 30 or 60 hz
bufsize = nbuf;   	  	//max num points to store in the PIC-SERVO buffer
x = (byte)xaxis;  		//axes addresses
y = (byte)yaxis;
z = (byte)zaxis;
group = (byte)groupaddr;
leader = (byte)leaderaddr;

if (freq == P_120HZ)    //set fast path mode if using 120 Hz path
  {
  ioctrl = ServoGetIoCtrl(x);
  ServoSetIoCtrl(x, (byte)(ioctrl | FAST_PATH));
  ioctrl = ServoGetIoCtrl(y);
  ServoSetIoCtrl(y, (byte)(ioctrl | FAST_PATH));
  if (z)
    {
    ioctrl = ServoGetIoCtrl(z);
    ServoSetIoCtrl(z, (byte)(ioctrl | FAST_PATH));
    }
  }
else    //clear fast path bit if using slower modes
  {
  ioctrl = ServoGetIoCtrl(x);
  ServoSetIoCtrl(x, (byte)(ioctrl & ~((byte)(FAST_PATH)) ));
  ioctrl = ServoGetIoCtrl(y);
  ServoSetIoCtrl(y, (byte)(ioctrl & ~((byte)(FAST_PATH)) ));
  if (z)
    {
    ioctrl = ServoGetIoCtrl(z);
    ServoSetIoCtrl(z, (byte)(ioctrl & ~((byte)(FAST_PATH)) ));
    }
  }

if (fabs(xscale)<1.0 || fabs(yscale)<1.0) return(-2);
if ( zaxis && (fabs(zscale)<1.0) ) return(-2);

UTOCX = xscale;   		//Units To X counts
UTOCY = yscale;
UTOCZ = zscale;

//Set the tolerance equivalent to 40 counts of the lowest resolution axis
tolerance = fabs(40.0/xscale);
if (tolerance < fabs(40.0/yscale)) tolerance = fabs(40.0/yscale);
if (zaxis)
  if (tolerance < fabs(40.0/zscale)) tolerance = fabs(40.0/zscale);

switch (pathfreq)       //calculate acceleration in units per tick^2
	{
    case P_30HZ: acc = accel/30.0/30.0;
    			 break;
    case P_60HZ: acc = accel/60.0/60.0;
    			 break;
    case P_120HZ: acc = accel/120.0/120.0;
    			 break;
    default: acc = accel/30.0/30.0;
    }

//Check that the required status data will be returned with each command:
statitems = SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX;

if ( ( (NmcGetStatItems(x)& statitems) != statitems )  ||
     ( (NmcGetStatItems(y)& statitems) != statitems )  ||
     ( (z && (NmcGetStatItems(z)& statitems) != statitems )) )
  {
  ErrorMsgBox("Required status items have not been set");
  return(-1);
  }

return(0);
}
//---------------------------------------------------------------------------
//Initialize various parameters for this path generation module
//Returns -1 if Status items are not set properly,
//        -2 if scale facotrs are zero
extern "C" WINAPI __declspec(dllexport) int SetPathParams(int freq, int nbuf,
                   int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
                   double xscale, double yscale, double zscale,
                   double accel )
{
byte statitems;

//SimpleMsgBox(" PIC-SERVO Coordinated Control Example\n\n- FOR EVALUATION PURPOSES ONLY -");

pathfreq = freq;        //set to 30 or 60 hz
bufsize = nbuf;   	  	//max num points to store in the PIC-SERVO buffer
x = (byte)xaxis;  		//axes addresses
y = (byte)yaxis;
z = (byte)zaxis;
group = (byte)groupaddr;
leader = (byte)leaderaddr;

if (freq == P_120HZ)    //set fast path mode if using 120 Hz path
  {
  ServoSetIoCtrl(x, IO1_IN | IO2_IN | FAST_PATH);
  ServoSetIoCtrl(y, IO1_IN | IO2_IN | FAST_PATH);
  if (z) ServoSetIoCtrl(z, IO1_IN | IO2_IN | FAST_PATH);
  }
else     //otherwise, use slow mode
  {
  ServoSetIoCtrl(x, IO1_IN | IO2_IN);
  ServoSetIoCtrl(y, IO1_IN | IO2_IN);
  if (z) ServoSetIoCtrl(z, IO1_IN | IO2_IN);
  }

if (fabs(xscale)<1.0 || fabs(yscale)<1.0) return(-2);
if ( zaxis && (fabs(zscale)<1.0) ) return(-2);

UTOCX = xscale;   		//Units To X counts
UTOCY = yscale;
UTOCZ = zscale;

//Set the tolerance equivalent to 40 counts of the lowest resolution axis
tolerance = fabs(40.0/xscale);
if (tolerance < fabs(40.0/yscale)) tolerance = fabs(40.0/yscale);
if (zaxis)
  if (tolerance < fabs(40.0/zscale)) tolerance = fabs(40.0/zscale);

switch (pathfreq)       //calculate acceleration in units per tick^2
	{
    case P_30HZ: acc = accel/30.0/30.0;
    			 break;
    case P_60HZ: acc = accel/60.0/60.0;
    			 break;
    case P_120HZ: acc = accel/120.0/120.0;
    			 break;
    default: acc = accel/30.0/30.0;
    }

//Check that the required status data will be returned with each command:
statitems = SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX;

if ( ( (NmcGetStatItems(x)& statitems) != statitems )  ||
     ( (NmcGetStatItems(y)& statitems) != statitems )  ||
     ( (z && (NmcGetStatItems(z)& statitems) != statitems )) )
  {
  ErrorMsgBox("Required status items have not been set");
  return(-1);
  }

return(0);
}
//---------------------------------------------------------------------------
//Initializes the coordinated path after all of the segments have been added.
//This function should be called just before the application starts calling
//the function AddPathPoints().
//Returns the overall path length for all of the segments.
//Returns 0.0 on communications error
extern "C" WINAPI __declspec(dllexport) double InitPath()
{
int i;

curseg = 0;
curppoint = 0;
arclen = 0.0;
stublen = 0.0;
pathlen = 0.0;
vel = 0.0;
finaldecel = 0;
at_end = 0;

for (i=0; i<seglistsize; i++) pathlen += seglist[i].len;

if (seglist[0].type == ARC) GetArcFrame( &(seglist[0]), &cur_arcframe );

//make sure we exit path mode first
if (!ServoStopMotor(x, ServoGetStopCtrl(x) & (byte)AMP_ENABLE)) return(0.0);
if (!ServoStopMotor(y, ServoGetStopCtrl(y) & (byte)AMP_ENABLE)) return(0.0);
if (z)
  if (!ServoStopMotor(z, ServoGetStopCtrl(z) & (byte)AMP_ENABLE)) return(0.0);

ServoInitPath(x);   //set the beginning of the path to the current position
ServoInitPath(y);
if (z) ServoInitPath(z);

return(pathlen);
}
//---------------------------------------------------------------------------
//Gets the next point in the path.
//returns -1 if already at the end of the path
//returns 1 if the last point in the path
//returns 0 otherwise
int GetNextPathpoint(long int *xp, long int *yp, long int *zp)
{
fp p;

if (at_end) return(-1);

//First check if decelerating to the endpoint
if (finaldecel || (vel*vel > 2*acc*(pathlen - arclen)) )
  {
  vel -= acc;
  if (vel<acc) vel = acc;   //use acc value as minimum velocity
  finaldecel = 1;
  }
else if (vel<maxvel)      //check for acceleration to current maxvel
  {
  vel+=acc;
  if (vel>maxvel) vel = maxvel;
  }
else if (vel>maxvel)      //check for deceleration to current maxvel
  {
  vel-=acc;
  if (vel<maxvel) vel = maxvel;
  }

while (1)  //skip over a segment (or more) if necessary
  {
  if ( (stublen + vel) > seglist[curseg].len )   //if past end of segment
    {
    stublen -= seglist[curseg].len;  		//subtract off the seg length
    curseg++;  						 		// & move to next segment
    if (seglist[curseg].type == ARC) 		//update arcframe if necessary
        GetArcFrame( &(seglist[curseg]), &cur_arcframe );
    if (curseg == seglistsize)
      {
      at_end = 1;  //check for end of segment list
      break;
      }
    }
  else
    {
    stublen += vel;
    arclen += vel;
    if (arclen > pathlen) at_end = 1;
    break;
    }
  }

if (at_end)   //return the final endpoint
  {
  *xp = (long int)( (seglist[seglistsize-1].p2[0] + xoff) * UTOCX );
  *yp = (long int)( (seglist[seglistsize-1].p2[1] + yoff) * UTOCY );
  *zp = (long int)( (seglist[seglistsize-1].p2[2] + zoff) * UTOCZ );
  }
else   //find the point within the current segment
  {
  if (seglist[curseg].type == LINE)
    {
    GetLineSegPoint( &(seglist[curseg]), stublen, p );
    }
  else if (seglist[curseg].type == ARC)
    {
    p[0] = seglist[curseg].r * cos(stublen/seglist[curseg].r);
    p[1] = seglist[curseg].r * sin(stublen/seglist[curseg].r);
    p[2] = 0;
    fvmult(&cur_arcframe, p, p);
    }
  *xp = (long int)( (p[0] + xoff) * UTOCX );
  *yp = (long int)( (p[1] + yoff) * UTOCY );
  *zp = (long int)( (p[2] + zoff) * UTOCZ );
  }

return(at_end);
}
//---------------------------------------------------------------------------
//Adds points to path buffer - should be called at regular intervals which
//are shorter than the buffer time (bufsize/pathfreq).
//
//Returns: -1 if path download is done
//         curseg if in middle  of the path
//		   -2 if communication error
extern "C" WINAPI __declspec(dllexport) int AddPathPoints()
{
long int xp[7], yp[7], zp[7];  //set of up to 7 pathpoints
int pcount;

//VEL = MAXVEL = 0.0 defines a feedhold condition
//Setting MAXVEL to a non-zero value will resume path execution
if (maxvel==0.0 && vel == 0.0) return(curseg);

while (!at_end)  //GetNextPathpoint() sets the global at_end
  {
  if (!NmcNoOp(x)) return(-2);                     //read num points from X

  //punt when PIC-SERVO buffer is full
  if ((ServoGetNPoints(x)>bufsize) || (ServoGetNPoints(x)>87))
    {
    if (!NmcNoOp(y)) return(-2);  //make sure data is updated even if points are not added
    if (!NmcNoOp(z)) return(-2);
    break;
    }

  for (pcount=0; pcount<7; pcount++)     //get upto 7 new points
  	if ( GetNextPathpoint( xp+pcount, yp+pcount, zp+pcount ) ) break;

  if (pcount<7) pcount++;
  
  if (!ServoAddPathpoints(x, pcount, xp, pathfreq)) return(-2);
  if (!ServoAddPathpoints(y, pcount, yp, pathfreq)) return(-2);
  if (z)
    if (!ServoAddPathpoints(z, pcount, zp, pathfreq)) return(-2);
  }

if ( !(ServoGetAux(x) & PATH_MODE) )   //start path mode when buffer full
  if (!ServoStartPathMode(group, leader)) return(-2);

if (at_end) return(-1);
return(curseg);
}
//---------------------------------------------------------------------------
