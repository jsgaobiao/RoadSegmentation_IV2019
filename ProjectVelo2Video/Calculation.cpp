
#include "utils.h"

void rMatrixInit (MATRIX &rt)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;
}

/////////////////////////////////////////////////////////////////////////////
//
//  ‹@”\     : ‚R–‚Rs—ñ‚ÌæŽZ
//
//  “ü—Í     : r		‚R–‚Rs—ñ
//  @@       rt		‚R–‚Rs—ñ
//  
//  •Ô‚è’l   : r <-	r – rt
//  
//  ”õl     : ŒÄ‚Ño‚µƒtƒ@ƒ“ƒNƒVƒ‡ƒ“ 
//@		@ ‚È‚µ
//  
/////////////////////////////////////////////////////////////////////////////

void rMatrixmulti (MATRIX &r, MATRIX &rt)
{
	double	rin[3][3];	 
	int		i, j;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			rin[i][j] = r[i][j];

	for (i=0; i<3; i++) 
		for (j=0; j<3; j++) { 
			r[i][j] = rin[i][0]*rt[0][j] +
					  rin[i][1]*rt[1][j] +
					  rin[i][2]*rt[2][j];
		}
}

/////////////////////////////////////////////////////////////////////////////
//
//  ‹@”\     : ‰ñ“]Šp“x‚É‚æ‚è‚R–‚RÀ•WŒn•ÏŠ·s—ñ‚ð‹‚ß‚é
//
//  “ü—Í     : rt		‚R–‚RÀ•WŒn•ÏŠ·s—ñ
//  @@       rotateX	‚wŽ²‚Ì‰ñ“]Špiƒ[Špj   ’PˆÊF
//  @@       rotateY	‚xŽ²‚Ì‰ñ“]Špiƒs[ƒ`Špj
//  @@       rotateZ	‚yŽ²‚Ì‰ñ“]Špiƒˆ[Špj
//  
//  •Ô‚è’l   : rt <- Rz * Ry * Rx
//  
//  ”õl     : ŒÄ‚Ño‚µƒtƒ@ƒ“ƒNƒVƒ‡ƒ“ 
//@		@ ‡@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_ZYX (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz  cosz   0.0
				   0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0  siny
			 		0.0   1.0   0.0
				  -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}
}

/////////////////////////////////////////////////////////////////////////////
//
//  ‹@”\     : ‰ñ“]Šp“x‚É‚æ‚è‚R–‚RÀ•WŒn•ÏŠ·s—ñ‚ð‹‚ß‚é
//
//  “ü—Í     : rt		‚R–‚RÀ•WŒn•ÏŠ·s—ñ
//  @@       rotateX	‚wŽ²‚Ì‰ñ“]Špiƒ[Špj   ’PˆÊF
//  @@       rotateY	‚xŽ²‚Ì‰ñ“]Špiƒs[ƒ`Špj
//  @@       rotateZ	‚yŽ²‚Ì‰ñ“]Špiƒˆ[Špj
//  
//  •Ô‚è’l   : rt <- Rz * Rx * Ry
//  
//  ”õl     : ŒÄ‚Ño‚µƒtƒ@ƒ“ƒNƒVƒ‡ƒ“ 
//@		@ ‡@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_ZXY (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz  cosz   0.0
				   0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}


	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0  siny
			 		0.0   1.0   0.0
				  -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

}

/////////////////////////////////////////////////////////////////////////////
//
//  ‹@”\     : ‰ñ“]Šp“x‚É‚æ‚è‚R–‚RÀ•WŒn•ÏŠ·s—ñ‚ð‹‚ß‚é
//
//  “ü—Í     : rt		‚R–‚RÀ•WŒn•ÏŠ·s—ñ
//  @@       rotateX	‚wŽ²‚Ì‰ñ“]Špiƒ[Špj   ’PˆÊF
//  @@       rotateY	‚xŽ²‚Ì‰ñ“]Špiƒs[ƒ`Špj
//  @@       rotateZ	‚yŽ²‚Ì‰ñ“]Špiƒˆ[Špj
//  
//  •Ô‚è’l   : rt <- Rx * Ry * Rz
//  
//  ”õl     : ŒÄ‚Ño‚µƒtƒ@ƒ“ƒNƒVƒ‡ƒ“ 
//@		@ ‡@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_XYZ (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0 siny
			 		0.0   1.0   0.0
				   -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz cosz   0.0
					0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}
}

void shiftPoint3fi (point3fi &pt, point3d &sh)
{
	point3d		p;

	p.x = pt.x + sh.x;
	p.y = pt.y + sh.y;
	p.z = pt.z + sh.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void rotatePoint3fi (point3fi &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[0][1]*pt.y + a[0][2]*pt.z;
	p.y = a[1][0]*pt.x + a[1][1]*pt.y + a[1][2]*pt.z;
	p.z = a[2][0]*pt.x + a[2][1]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}


void shiftPoint3d (point3d &pt, point3d &sh)
{
	point3d		p;

	p.x = pt.x + sh.x;
	p.y = pt.y + sh.y;
	p.z = pt.z + sh.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void rotatePoint3d (point3d &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[0][1]*pt.y + a[0][2]*pt.z;
	p.y = a[1][0]*pt.x + a[1][1]*pt.y + a[1][2]*pt.z;
	p.z = a[2][0]*pt.x + a[2][1]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void INVshiftPoint3d (point3d &pt, point3d &sh)
{
	pt.x = pt.x - sh.x;
	pt.y = pt.y - sh.y;
	pt.z = pt.z - sh.z;
}

void INVrotatePoint3d (point3d &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[1][0]*pt.y + a[2][0]*pt.z;
	p.y = a[0][1]*pt.x + a[1][1]*pt.y + a[2][1]*pt.z;
	p.z = a[0][2]*pt.x + a[1][2]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

double ppDistance2d (point2d *p1, point2d *p2)
{
	return sqrt(sqr(p1->x-p2->x)+sqr(p1->y-p2->y));
}

double innerProduct2d (point2d *v1, point2d *v2)
{
	return (v1->x*v2->x+v1->y*v2->y);
}

double normVector2d (point2d *p)
{
	return (sqrt(sqr(p->x)+sqr(p->y)));
}

double normalize2d (point2d *p)
{
	const   double NORM_EPS = 1.0e-10;
	double	norm;

	norm = normVector2d (p);
	if (norm) {
		p->x /= (norm + NORM_EPS);
		p->y /= (norm + NORM_EPS);
	}
	return norm;
}

double ppDistance3fi (point3fi *pt1, point3fi *pt2)
{
	return (sqrt(sqr(pt1->x-pt2->x)+sqr(pt1->y-pt2->y)+sqr(pt1->z-pt2->z)));
}

double p2r (point3fi *pt1)
{
	return (sqrt(sqr(pt1->x)+sqr(pt1->y)+sqr(pt1->z)));
}

void shiftPoint2d (point2d &pt, point2d &sh)
{
	pt.x += sh.x;
	pt.y += sh.y;
}

void rotatePoint2d (point2d &pt, MAT2D &a)
{
	point2d	p;

	p.x = a[0][0]*pt.x + a[0][1]*pt.y;
	p.y = a[1][0]*pt.x + a[1][1]*pt.y;
	pt.x = p.x;
	pt.y = p.y;
}

void ppTransform2d (point2d &p, TRANS2D &srctrans, TRANS2D &tartrans)
{
	MAT2D	rot1, rot2;

	//rot1: R_t^{-1}*R_{t-1}, srctrans:t-1, tartrans:t
	rot1[0][0] = cos (srctrans.ang-tartrans.ang);
	rot1[0][1] = -sin (srctrans.ang-tartrans.ang);
	rot1[1][0] = sin (srctrans.ang-tartrans.ang);
	rot1[1][1] = cos (srctrans.ang-tartrans.ang);

	//rot2: R_t^{-1}, srctrans:t-1, tartrans:t
	rot2[0][0] = cos (-tartrans.ang);
	rot2[0][1] = -sin (-tartrans.ang);
	rot2[1][0] = sin (-tartrans.ang);
	rot2[1][1] = cos (-tartrans.ang);

	//shv: SHV_{t-1}-SHV_{t}, srctrans:t-1, tartrans:t
	point2d shv;
	shv.x = srctrans.shv.x-tartrans.shv.x;
	shv.y = srctrans.shv.y-tartrans.shv.y;

	rotatePoint2d (p, rot1);	//R_t^{-1}*R_{t-1}*p
	rotatePoint2d (shv, rot2);	//R_t^{-1}*(SHV_{t-1}-SHV_{t})
	shiftPoint2d (p, shv);		//p'=R_t^{-1}*R_{t-1}*p+R_t^{-1}*(SHV_{t-1}-SHV_{t})
}
