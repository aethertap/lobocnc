//---------------------------------------------------------------------------
#ifndef pathH
#define pathH
//---------------------------------------------------------------------------
//Defines:

//Segment types:
#define LINE 0
#define ARC 1

#define MAXSEG 5000     //Maximum number of segments
#define PI 3.14159
#define TWOPI 6.28319
#define DTOR 0.017453

//Valuse for tangent tolerance
#define TAN_1DEGREE 0.99985
#define TAN_3DEGREE 0.99863
#define TAN_5DEGREE 0.99619
#define TAN_10DEGREE 0.98481
#define TAN_20DEGREE 0.93969
#define TAN_45DEGREE 0.70711

#define ONLINE 1
//---------------------------------------------------------------------------
//Data types:

typedef double fp[3];     //floating point 3x1 vector

typedef long int ip[3];  //integer 3x1 vector

typedef struct {         //data type for line segments or arc segments
			int type;		//LINE or ARC
			fp p1;          //Starting point
            fp p2;          //Ending point
            fp c;           //Center point (arcs only)
            fp norm;        //Normal vector (arcs only)
            double len;		//Segment length
            double r;		//Radius (arcs only)
            } segment;

typedef struct {         //data type for a coordinate frame
			fp	x;
            fp	y;
            fp	z;
            fp	p;
            } frame;

//---------------------------------------------------------------------------
//Function prototypes:
double mag(fp p);
double dot(fp x, fp y);
void cross(fp x, fp y, fp z);
double normalize(fp x, fp y);
void fvmult(frame *F, fp x, fp y);
void finvert(frame A, frame *B);
int GetTanVect(segment *s, fp p, int endpoint);
void GetArcFrame(segment *seg, frame *F);
void GetLineSegPoint(segment *seg, double s, fp p);
int GetNextPathpoint(long int *xp, long int *yp, long int *zp);

//Path mode API functions:
extern "C" WINAPI __declspec(dllexport) void SetTangentTolerance(double theta);
extern "C" WINAPI __declspec(dllexport) void ClearSegList(double x, double y, double z);
extern "C" WINAPI __declspec(dllexport) int AddLineSeg(double x, double y, double z);
extern "C" WINAPI __declspec(dllexport) int AddArcSeg( double x, double y, double z,        //end point
               										   double cx, double cy, double cz,     //center point
               										   double nx, double ny, double nz );    //normal
extern "C" WINAPI __declspec(dllexport) void SetFeedrate(double fr);
extern "C" WINAPI __declspec(dllexport) void SetOrigin(double xoffset, double yoffset, double zoffset);
extern "C" WINAPI __declspec(dllexport) int SetPathParams(int freq, int nbuf,
                   										  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
                   										  double xscale, double yscale, double zscale,
                   										  double accel );
extern "C" WINAPI __declspec(dllexport) int SetPathParams2(int freq, int nbuf,
                   										  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
                   										  double xscale, double yscale, double zscale,
                   										  double accel );
extern "C" WINAPI __declspec(dllexport) double InitPath();
extern "C" WINAPI __declspec(dllexport) int AddPathPoints();

//---------------------------------------------------------------------------
#endif
