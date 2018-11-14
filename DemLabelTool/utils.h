#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define RED "\e[1;31m"
#define NONE "\e[0m"

#define BOUND(x,min,max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define	nint(x)			(int)((x>0)?(x+0.5):(x-0.5))
#define	sqr(x)			((x)*(x))
#define topi (acos(-1.0)/180.0)     // pi/180

typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned int UINT;
typedef void *HANDLE;
typedef unsigned long DWORD;
typedef long long LONGLONG;
typedef long LONG;

#define	WIDSIZ		60.0
#define	LENSIZ		60.0
#define	PIXSIZ		0.2        //0.25
#define	PNTS_PER_LINE		32
#define	LINES_PER_BLK		12
#define	PTNUM_PER_BLK		(32*12)
#define	BKNUM_PER_FRM		(580 / 2)
#define	SCANDATASIZE		(BKNUM_PER_FRM*LINES_PER_BLK/2)

typedef double  MAT2D[2][2] ;

typedef struct {
    float			x, y, z;
    u_char			i;
} point3fi;

typedef struct {
    int x, y;
} point2i;

typedef struct {
    long long millisec;
    double x, y, z, roll, pitch, yaw;
    int gpsStatus;
} NAVDATA;

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

typedef struct {
    double			ang;
    point2d			shv;
    MAT2D			rot;
} TRANS2D;

typedef double  MATRIX[3][3] ;
typedef struct {
    point3d			ang;
    point3d			shv;
    MATRIX			rot;
} TRANSINFO;

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

extern FILE *dfp;
extern FILE* fFileNameList;
extern std::vector<NAVDATA> nav;
extern bool camCalibFlag;
extern TRANSINFO	calibInfo;
extern int dsbytesiz;
extern ONEDSVFRAME	*onefrm;
extern int dFrmNo;
extern ONEDSVDATA preFrameDsv;

// utils.cpp
LONGLONG myGetFileSize(FILE *f);
bool ReadOneDsvFrame ();
bool LoadCalibFile (const char *szFile);
void Gt2BGR(cv::Mat &img);
void BGR2Gt(cv::Mat &img, cv::Mat gt);
bool LoadCameraCalib (const char *filename);
void pointCloudsProject(cv::Mat &img, cv::Mat gm, cv::Mat zmap);
point3fi CorrectPoints (point3fi p, ONEDSVDATA dsv);
void TransAnnotation(cv::Mat &srcImg);

// Calculation.cpp
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
void shiftPoint3fi (point3fi &pt, point3d &sh);
void rotatePoint3fi (point3fi &pt, MATRIX &a);
void shiftPoint2d (point2d &pt, point2d &sh);
void rotatePoint2d (point2d &pt, MAT2D &a);

// MyCalib.cpp
void WC2IC_fang(double Xw, double Yw, double Zw, double *Xfd, double *Yfd);
#endif // UTILS_H
