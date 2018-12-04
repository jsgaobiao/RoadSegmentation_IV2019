//-------------------------------------------------------------------------
//	Camera Calibration using Tsai's Pin-hole Model
//-------------------------------------------------------------------------
//	The original source code is downloaded from
//			http://www-2.cs.cmu.edu/~rgw/TsaiCode.html
//-------------------------------------------------------------------------
//	GUI is Developed by 
//			Huijing ZHAO
//			CSIS. Univ. of Tokyo
//	Date		Apr. 2004
//-------------------------------------------------------------------------

#include "CAL_MAIN.H"
#include "utils.h"

double		vxx, vxy, vxz;
double		vyx, vyy, vyz;
double		vzx, vzy, vzz;
double		opx, opy, opz;
int			coplanar=0;

double		IMAGEWID=640;
double		IMAGELEN=480;

extern camera_parameters cp;
extern calibration_data cd;
extern calibration_constants cc;
extern char   camera_type[];

#include <iostream>
void ModifyManually()
{
	char flag[2];
	std::cin >> flag;
	if (strcmp(flag, "RX") == 0)
	{
		std::cout << "cc->Rx" << cc.Rx << std::endl;
		std::cin >> cc.Rx;
	}
	else if (strcmp(flag, "RY") == 0)
	{
		std::cout << "cc->Ry" << cc.Ry << std::endl;
		std::cin >> cc.Ry;
	}
	else if (strcmp(flag, "RZ") == 0)
	{
		std::cout << "cc->Rz" << cc.Rz << std::endl;
		std::cin >> cc.Rz;
	}
	else if (strcmp(flag, "TX") == 0)
	{
		std::cout << "cc->Tx" << cc.Tx << std::endl;
		std::cin >> cc.Tx;
	}
	else if (strcmp(flag, "TY") == 0)
	{
		std::cout << "cc->Ty" << cc.Ty << std::endl;
		std::cin >> cc.Ty;
	}
	else if (strcmp(flag, "TZ") == 0)
	{
		std::cout << "cc->Tz" << cc.Tz << std::endl;
		std::cin >> cc.Tz;
	}
	else if (strcmp(flag, "F") == 0)
	{
		std::cout << "cc->f" << cc.f << std::endl;
		std::cin >> cc.f;
	}
	else
	{
		std::cout << "None\n";
		return;
	}

	double    sa,
		ca,
		sb,
		cb,
		sg,
		cg;

	SINCOS(cc.Rx, sa, ca);
	SINCOS(cc.Ry, sb, cb);
	SINCOS(cc.Rz, sg, cg);

	cc.r1 = cb * cg;
	cc.r2 = cg * sa * sb - ca * sg;
	cc.r3 = sa * sg + ca * cg * sb;
	cc.r4 = cb * sg;
	cc.r5 = sa * sb * sg + ca * cg;
	cc.r6 = ca * sb * sg - cg * sa;
	cc.r7 = -sb;
	cc.r8 = cb * sa;
	cc.r9 = ca * cb;
}

bool LoadCameraCalib (const char *filename)
{
    FILE		*fp;

    fp = fopen (filename, "r");
	if (!fp) 
		return false;

    initialize_photometrics_parms ();
	my_load_cp_cc_data (fp, &cp, &cc);

	fclose (fp);
	return true;
}
void WC2IC_fang(double Xw, double Yw, double Zw, double *Xfd, double *Yfd)
{
	Xw *= 1000;//convert the unit to millimeter
	Yw *= 1000;
	Zw *= 1000;
	
	double	x, y, z;

	if (coplanar) {
		x = (Xw - opx)*vxx + (Yw - opy)*vxy + (Zw - opz)*vxz;
		y = (Xw - opx)*vyx + (Yw - opy)*vyy + (Zw - opz)*vyz;
		z = (Xw - opx)*vzx + (Yw - opy)*vzy + (Zw - opz)*vzz;
		//x += 1000;
		//y += 1000;
		z = -z;
	}
	else {
		x = Xw + 10000;
		y = Yw + 10000;
		z = Zw + 10000;
	}

	world_coord_to_image_coord(x, y, z, Xfd, Yfd);

	*Xfd = *Xfd;
	*Yfd = IMAGELEN - 1 - *Yfd;

}

void IC2WC_fang(double * xw, double * yw, double zw, double Xfd, double Yfd)
{
	Yfd = IMAGELEN - 1 - Yfd;
	double	x, y, z;
	double	Xw, Yw, Zw;
	int	cnt = 0;

	if (!coplanar) {
		z = zw - 10000;
		image_coord_to_world_coord(Xfd, Yfd, z, &x, &y);
		(*xw) = x - 10000;
		(*yw) = y - 10000;
		return;
	}

	*xw = -1;
	*yw = -1;

	z = zw;

	while (cnt < 10) {
		image_coord_to_world_coord(Xfd, Yfd, z, &x, &y);

		y -= 1000;
		x -= 1000;
		z = -z;

		Xw = vxx * x + vyx * y + vzx * z + opx;
		Yw = vxy * x + vyy * y + vzy * z + opy;
		Zw = vxz * x + vyz * y + vzz * z + opz;

		if (fabs(Zw - zw) < 0.01) {
			*xw = x / 1000;
			*yw = y / 1000;
			return;
		}

		Zw = zw;
		z = (Xw - opx)*vzx + (Yw - opy)*vzy + (Zw - opz)*vzz;
		z = -z;

		cnt++;
	}

	if (cnt >= 10)
	{
		*yw /= 1000;
		*xw /= 1000;
		return;
	}
}

void WC2IC (double Xw, double Yw, double Zw, double *Xfd, double *Yfd)
{
	double	x, y, z;

	if (coplanar) {
		x = (Xw-opx)*vxx+(Yw-opy)*vxy+(Zw-opz)*vxz;
		y = (Xw-opx)*vyx+(Yw-opy)*vyy+(Zw-opz)*vyz;
		z = (Xw-opx)*vzx+(Yw-opy)*vzy+(Zw-opz)*vzz;
		x += 1000;
		y += 1000;
		z = -z;
	}
	else {
		x = Xw+10000;
		y = Yw+10000;
		z = Zw+10000;
	}

	world_coord_to_image_coord (x, y, z, Xfd, Yfd);
}

void IC2WC (double *xw, double *yw, double zw, double Xfd, double Yfd)
{
	double	x, y, z;
	double	Xw, Yw, Zw;
	int	cnt = 0;

	if (!coplanar) {
		z = zw-10000;
		image_coord_to_world_coord (Xfd, Yfd, z, &x, &y);
		(*xw) = x-10000;
		(*yw) = y-10000;
		return;
	}

	*xw = -1;
	*yw = -1;

	z = zw;

	while (cnt<10) {
		image_coord_to_world_coord (Xfd, Yfd, z, &x, &y);

		y -= 1000;
		x -= 1000;
		z = -z;

		Xw = vxx*x+vyx*y+vzx*z+opx;
		Yw = vxy*x+vyy*y+vzy*z+opy;
		Zw = vxz*x+vyz*y+vzz*z+opz;

		if (fabs(Zw-zw)<0.01) {
			*xw = x;
			*yw = y;
			return;
		}
		
		Zw = zw;
		z = (Xw-opx)*vzx+(Yw-opy)*vzy+(Zw-opz)*vzz;
		z = -z;

		cnt ++;
	}

	if (cnt>=10)
		return;
}

void my_load_cp_cc_data (FILE *fp,camera_parameters *cp, calibration_constants *cc)
{
    double    sa,
              ca,
              sb,
              cb,
              sg,
              cg;
	int			i;

    fscanf (fp, "%lf", &(cp->Ncx));
    fscanf (fp, "%lf", &(cp->Nfx));
    fscanf (fp, "%lf", &(cp->dx));
    fscanf (fp, "%lf", &(cp->dy));
    fscanf (fp, "%lf", &(cp->dpx));
    fscanf (fp, "%lf", &(cp->dpy));
    fscanf (fp, "%lf", &(cp->Cx));
    fscanf (fp, "%lf", &(cp->Cy));
    fscanf (fp, "%lf", &(cp->sx));

    fscanf (fp, "%lf", &(cc->f));
    fscanf (fp, "%lf", &(cc->kappa1));
    fscanf (fp, "%lf", &(cc->Tx));
    fscanf (fp, "%lf", &(cc->Ty));
    fscanf (fp, "%lf", &(cc->Tz));
    fscanf (fp, "%lf", &(cc->Rx));
	fscanf (fp, "%lf", &(cc->Ry));
    fscanf (fp, "%lf", &(cc->Rz));

    SINCOS (cc->Rx, sa, ca);
    SINCOS (cc->Ry, sb, cb);
    SINCOS (cc->Rz, sg, cg);

    cc->r1 = cb * cg;
    cc->r2 = cg * sa * sb - ca * sg;
    cc->r3 = sa * sg + ca * cg * sb;
    cc->r4 = cb * sg;
    cc->r5 = sa * sb * sg + ca * cg;
    cc->r6 = ca * sb * sg - cg * sa;
    cc->r7 = -sb;
    cc->r8 = cb * sa;
    cc->r9 = ca * cb;

    fscanf (fp, "%lf", &(cc->p1));
    fscanf (fp, "%lf", &(cc->p2));

	fscanf (fp, "%lf", &(vxx));
	fscanf (fp, "%lf", &(vxy));
	fscanf (fp, "%lf", &(vxz));
	fscanf (fp, "%lf", &(vyx));
	fscanf (fp, "%lf", &(vyy));
	fscanf (fp, "%lf", &(vyz));
	fscanf (fp, "%lf", &(vzx));
	fscanf (fp, "%lf", &(vzy));
	fscanf (fp, "%lf", &(vzz));
	fscanf (fp, "%lf", &(opx));
	fscanf (fp, "%lf", &(opy));
	fscanf (fp, "%lf", &(opz));
	fscanf (fp, "%d", &(i));
	if (i) coplanar = 1;
	else coplanar = 0;
	if (fscanf (fp, "%d", &(i)) == EOF)
		return;
	IMAGEWID = i;
	if (fscanf (fp, "%d", &(i)) == EOF)
		return;
	IMAGELEN = i;
}

void my_init_calib ()
{
    double    sa,
              ca,
              sb,
              cb,
              sg,
              cg;

	initialize_photometrics_parms ();

	cp.Ncx = 5.7600000000e+002;
	cp.Nfx = 5.7600000000e+002;
	cp.dx = 2.3000000000e-002;
	cp.dy = 2.3000000000e-002;
	cp.dpx = 2.3000000000e-002;
	cp.dpy = 2.3000000000e-002;
	cp.Cx = 4.5656272986e+002;
	cp.Cy = 4.5119173674e+002;
	cp.sx = 1.0000000000e+000;
	cc.f = 6.1695610912e+001;
	cc.kappa1 = 1.5421050480e-004;
	cc.Tx = 1.0633433354e+004;
	cc.Ty = -2.2866119175e+001;
	cc.Tz = 4.1310329350e+004;
	cc.Rx = -1.9861054087e-001;
	cc.Ry = -9.4928378862e-001;
	cc.Rz = 1.8613070586e+000;

    SINCOS (cc.Rx, sa, ca);
    SINCOS (cc.Ry, sb, cb);
    SINCOS (cc.Rz, sg, cg);

    cc.r1 = cb * cg;
    cc.r2 = cg * sa * sb - ca * sg;
    cc.r3 = sa * sg + ca * cg * sb;
    cc.r4 = cb * sg;
    cc.r5 = sa * sb * sg + ca * cg;
    cc.r6 = ca * sb * sg - cg * sa;
    cc.r7 = -sb;
    cc.r8 = cb * sa;
    cc.r9 = ca * cb;

	cc.p1 = 0.0000000000e+000;
	cc.p2 = 0.0000000000e+000;
	vxx = 1.0000000000e+000;
	vxy = 0.0000000000e+000;
	vxz = 0.0000000000e+000;
	vyx = 0.0000000000e+000;
	vyy = 1.0000000000e+000;
	vyz = 0.0000000000e+000;
	vzx = 0.0000000000e+000;
	vzy = 0.0000000000e+000;
	vzz = 1.0000000000e+000;
	opx = 0.0000000000e+000;
	opy = 0.0000000000e+000;
	opz = 2.6000000000e+002;
	coplanar = 1;
}

void my_dump_cp_cc_data (FILE *fp, camera_parameters *cp, calibration_constants *cc)
{
    fprintf (fp, "%17.10le\n", cp->Ncx);
    fprintf (fp, "%17.10le\n", cp->Nfx);
    fprintf (fp, "%17.10le\n", cp->dx);
    fprintf (fp, "%17.10le\n", cp->dy);
    fprintf (fp, "%17.10le\n", cp->dpx);
    fprintf (fp, "%17.10le\n", cp->dpy);
    fprintf (fp, "%17.10le\n", cp->Cx);
    fprintf (fp, "%17.10le\n", cp->Cy);
    fprintf (fp, "%17.10le\n", cp->sx);

    fprintf (fp, "%17.10le\n", cc->f);
    fprintf (fp, "%17.10le\n", cc->kappa1);
    fprintf (fp, "%17.10le\n", cc->Tx);
    fprintf (fp, "%17.10le\n", cc->Ty);
    fprintf (fp, "%17.10le\n", cc->Tz);
    fprintf (fp, "%17.10le\n", cc->Rx);
    fprintf (fp, "%17.10le\n", cc->Ry);
    fprintf (fp, "%17.10le\n", cc->Rz);
    fprintf (fp, "%17.10le\n", cc->p1);
    fprintf (fp, "%17.10le\n", cc->p2);

	fprintf (fp, "%17.10le\n", vxx);
	fprintf (fp, "%17.10le\n", vxy);
	fprintf (fp, "%17.10le\n", vxz);
	fprintf (fp, "%17.10le\n", vyx);
	fprintf (fp, "%17.10le\n", vyy);
	fprintf (fp, "%17.10le\n", vyz);
	fprintf (fp, "%17.10le\n", vzx);
	fprintf (fp, "%17.10le\n", vzy);
	fprintf (fp, "%17.10le\n", vzz);
	fprintf (fp, "%17.10le\n", opx);
	fprintf (fp, "%17.10le\n", opy);
	fprintf (fp, "%17.10le\n", opz);
	fprintf (fp, "%d\n", coplanar?1:0);

}

void IC2WC_OnePt (double x, double y, double *xx, double *yy, double spheight)
{
	double	xw, yw, zw;
	double	ix, iy;

	ix = x * IMAGEWID;
	iy = y * IMAGELEN;
	zw = spheight*1000.0;
	IC2WC (&xw, &yw, zw, ix, iy);		//the unit of xw and yw is millimeter
	(*xx) = xw/1000.0;					//convert the unit to meter
	(*yy) = yw/1000.0;
}

void WC2IC_OnePt (double x, double y, double z, double *xx, double *yy)
{
	double	xw, yw, zw;
	double	ix, iy;

	xw = x*1000.0;					//convert the unit to millimeter
	yw = y*1000.0;
	zw = z*1000.0;
	WC2IC (xw, yw, zw, &ix, &iy);
	(*xx) = ix/(double)IMAGEWID;
	(*yy) = iy/(double)IMAGELEN;
}


