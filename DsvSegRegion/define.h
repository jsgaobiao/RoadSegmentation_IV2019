#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cstdio>
#include <memory>
#include <cmath>
#include <ctime>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>
#include <queue>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int maxn = 2000000000;
const double topi = acos(-1.0)/180.0;	// pi/180
#define BOUND(x,min,max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define	nint(x)			(int)((x>0)?(x+0.5):(x-0.5))
#define	sqr(x)			((x)*(x))

typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned int UINT;
typedef void *HANDLE;
typedef unsigned long DWORD;
typedef long long LONGLONG;
typedef long LONG;

typedef union _LARGE_INTEGER {
    struct {
        DWORD LowPart;
        LONG HighPart;
    } DUMMYSTRUCTNAME;
    struct {
        DWORD LowPart;
        LONG HightPart;
    } u;
    LONGLONG QuadPart;
} LARGE_INTEGER;

struct point2d
{
	double x;
	double y;
};

struct point3d
{
    double x;
    double y;
    double z;
};

typedef double  MATRIX[3][3] ; 

typedef struct {
	point3d			ang;
	point3d			shv;
	MATRIX			rot;
} TRANSINFO;

typedef double  MAT2D[2][2] ; 

typedef struct {
	double			ang;
	point2d			shv;
	MAT2D			rot;
} TRANS2D;

#define	PNTS_PER_LINE		32
#define	LINES_PER_BLK		12
#define	PTNUM_PER_BLK		(32*12)
#define	BKNUM_PER_FRM		580 / 2
#define	SCANDATASIZE		(BKNUM_PER_FRM*LINES_PER_BLK/2)

//for vel64
//HORIERRFACTOR=tan（水平角分辨率=0.1度）*（放大系数=2.0）=0.0018*5
#define	HORIERRFACTOR	0.02	//0.006
//VERTERRFACTOR=tan（垂直角分辨率=0.38度）*（放大系数=1.5）=0.0067*5
#define	VERTERRFACTOR	0.035	//0.035
#define	BASEERROR		0.3
#define	MAXSMOOTHERR	1.0
#define	MAXDISTHRE		2.0


#define	M_PI		3.1415926536

#define	INVALIDDOUBLE		99999999.9


typedef struct {
	float			x, y, z;
    u_char			i;
} point3fi;

typedef struct {
	int x, y;
} point2i;

typedef struct {
	long long		millisec;
	point3fi		points[PTNUM_PER_BLK];
} ONEVDNDATA;

typedef struct {
	point3d			ang;
	point3d			shv;
	long long		millisec;
	point3fi		points[PTNUM_PER_BLK];
	MATRIX			rot;
} ONEDSVDATA;

typedef struct {
	ONEDSVDATA		dsv[BKNUM_PER_FRM];
} ONEDSVFRAME;

typedef struct {
    long long millisec;
    double x, y, z, roll, pitch, yaw;
    int gpsStatus;
} NAVDATA;

typedef	struct {
	unsigned short	lab;
	point2i		dmin;
	point2i		dmax;
	point3fi	maxxp,maxyp, maxzp;
	point3fi	minxp,minyp, minzp;
	point3d		cp;
	int			ptnum;
	point3d		norm;
	double		var;
} SEGBUF;

//vel64
#define	VMINANG		(-21.627*M_PI/180.0)
#define	VMAXANG		(2.432*M_PI/180.0)

typedef struct {
	int				wid;
	int				len;
	double			h0;
	double			v0;
	double			hres;
	double			vres;
	point3fi		*pts;
	point2i			*idx;
	BYTE			*di;		//for data alignment only
	int				*regionID;
	int				regnum;
	SEGBUF			*segbuf;
    IplImage		*rMap;      // range image
    IplImage		*lMap;      // region segmentation image
} RMAP;

#define	WIDSIZ		60.0
#define	LENSIZ		60.0
#define	PIXSIZ		0.2        //0.25
#define	POSOBSMINHEIGHT	0.6		//0.6m
#define	VEHICLEHEIGHT	3.0		//3.0m
#define	NEARVEHICLEDIS	6.0		//5.0m


typedef struct {
    int			x0,x1;		//DEM中的地面点开始和结束像素序号[0,dm.wid)
//	int			y;			//DEM中的像素序号[0,dm.len)
    double		h;			//中心点位置(x0+x1)/2))的地面高度
    double		dl;			//纵向距离y处的地面扫描线与前一条扫描线间的正常水平距离
                            //前一条扫描线为与车体更近的那条，两个扫描线之间的角度d_ang=(VMAXANG-VMINANG)/63)
} CENTERLN;

typedef struct {
    int				wid;
    int				len;
    double			*demg;			//ground
    int				*demgnum;
    double			*demhmin;		//non-ground
    double			*demhmax;		//non-ground
    int				*demhnum;
    BYTE			*lab;
    double			*groll;
    double			*gpitch;
    BYTE			*sublab;
    double			*lpr;			//probability of the lab
    double			*WX, *WY, *WZ;
    CENTERLN		*centerln;
    IplImage		*lmap;  // label map
    IplImage		*smap;  // sublabel map
    IplImage        *zmap;  // z height map
    IplImage        *pmap;  // probability map
    TRANS2D			trans;
    bool			dataon;
} DMAP;

#define UNKNOWN			0
#define NONVALID		-9999
#define DONTCARE        -99
#define EDGEPT			-9

#define TRAVESABLE		1
#define NONTRAVESABLE	2
#define POSSIOBSTA		3
#define	NEGATOBSTA		4
#define HANGDOWNTR		5
#define HANGDOWNUN		6
#define	FLATGROUND		10
#define DOWNSLOPE		11
#define UPSLOPE			12
#define	LEFTSIDESLOPE	13
#define	RIGHTSIDESLOPE	14
#define	EDGEPOINTS		15

#define	istravesable(x)			(x==TRAVESABLE||x==DOWNSLOPE||x==UPSLOPE||x==SIDESLOPE)


//extern RMAP	rm;
//extern TRANSINFO calibInfo;
//extern ONEDSVFRAME	*onefrm;

void rMatrixInit (MATRIX &rt);
void rMatrixmulti (MATRIX &r, MATRIX &rt);
void createRotMatrix_ZYX (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_XYZ (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_ZXY (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void shiftPoint3d (point3d &pt, point3d &sh);
void rotatePoint3d (point3d &pt, MATRIX &a);
double normalize2d (point2d *p);
double ppDistance2d (point2d *p1, point2d *p2);
double innerProduct2d (point2d *v1, point2d *v2);
double ppDistance3fi (point3fi *pt1, point3fi *pt2);
double p2r (point3fi *pt1);
void rotatePoint3fi (point3fi &pt, MATRIX &a);

BOOL ContourSegger();
void SmoothingData ();
void Region2Seg ();
void EstimateSeg ();
void ContourExtraction();
UINT RegionGrow();
void EdgeGrow();
void ClassiSeg ();
void OutputLog (char *filename, char *str);

void INVshiftPoint3d (point3d &pt, point3d &sh);
void INVrotatePoint3d (point3d &pt, MATRIX &a);
void shiftPoint3fi (point3fi &pt, point3d &sh);
void rotatePoint3fi (point3fi &pt, MATRIX &a);

void Calculate_Plane(int Points_Total, double *X_Coord, double *Y_Coord, double *Z_Coord,
					 int Origin_Flag, double Plane_Eq[4]);
void Calculate_Residuals(double *X, double *Y, double *Z, double Equation[4], 
						 double *Error, int PointsTotal);
void shiftPoint2d (point2d &pt, point2d &sh);
void rotatePoint2d (point2d &pt, MAT2D &a);

void DrawRangeView ();
void GenerateRangeView ();
void InitRmap (RMAP *rm);
void ReleaseRmap (RMAP *rm);

void DrawDem (DMAP &m);
void CopyGloDem (DMAP *tar, DMAP *src);
void ZeroGloDem (DMAP *m);
void InitDmap (DMAP *dm);
void ReleaseDmap (DMAP *dm);
void PredictGloDem (DMAP &gmtar, DMAP &gmtmp);
void UpdateGloDem (DMAP &glo, DMAP &loc);
void GenerateLocDem (DMAP &loc);
void CallbackLocDem(int event, int x, int y, int flags, void *ustc);
void LabelRoadSurface (DMAP &glo);
void LabelObstacle (DMAP &glo);
void ExtractRoadCenterline (DMAP &glo);
