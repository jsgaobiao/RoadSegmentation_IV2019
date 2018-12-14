#include "define.h"
#include "ContourSegger.h"

#include <vector>
using namespace std;


extern RMAP	rm;
extern TRANSINFO calibInfo;
extern ONEDSVFRAME	*onefrm;

vector <PTOpti> VecOpti;


bool isppNeighbor (point3fi *pt, point3fi *cp, bool ishori)
{
    double rng = max (p2r(pt),p2r(cp));
    double maxdd = ishori?max(BASEERROR,0.03*rng):max(BASEERROR,0.3*rng);
    double dd = ppDistance3fi (pt, cp);
    if (dd>maxdd)
        return false;

    bool isneighbor=false;
    double dz = fabs(pt->z-cp->z);
    // 高度差小
    if (dz<0.1)
//    if (dz<0.07)  // cha
        isneighbor = true;
    else {
        double dh = sqrt(sqr(pt->x-cp->x)+sqr(pt->y-cp->y));
        // 高度差大，水平距离小 --> 比较陡
        if (dh<0.56)
//        if (dh<0.86)  // cha
            isneighbor = false;
        else {
            // 坡度
            double ang = atan2(dz, dh);
            if (ang<0.17)		//5deg
//            if (ang<0.05)		//5deg  cha
                isneighbor = true;
            else
                isneighbor = false;
        }
        isneighbor = false;
    }

    return isneighbor;
}


bool AddPoints(IMCOORDINATE seed,vector<IMCOORDINATE> & vec, int _regid)
{
	if (rm.regionID[rm.wid*seed.x +seed.y] == UNKNOWN)
		rm.regionID[rm.wid*seed.x +seed.y] = _regid;
	else if (rm.regionID[rm.wid*seed.x +seed.y] != _regid)
		return false;

	IMCOORDINATE Neighbor;

	point3fi *cp = &rm.pts[rm.wid*seed.x +seed.y];

	for (int k=0; k<4; k++) 
	{
		double errfactor;
		switch (k) {
		case 0:	Neighbor.x = seed.x - 1; Neighbor.y = seed.y;errfactor=VERTERRFACTOR; break;
		case 1: Neighbor.x = seed.x + 1; Neighbor.y = seed.y;errfactor=VERTERRFACTOR; break;
		case 2: Neighbor.x = seed.x; Neighbor.y = seed.y - 1;errfactor=HORIERRFACTOR; break;
		case 3: Neighbor.x = seed.x; Neighbor.y = seed.y + 1;errfactor=HORIERRFACTOR; break;
//		case 4:	Neighbor.x = seed.x - 1; Neighbor.y = seed.y - 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 5: Neighbor.x = seed.x + 1; Neighbor.y = seed.y + 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 6: Neighbor.x = seed.x + 1; Neighbor.y = seed.y - 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
//		case 7: Neighbor.x = seed.x - 1; Neighbor.y = seed.y + 1;errfactor=(VERTERRFACTOR+HORIERRFACTOR)/2.0; break;
		}

		if (Neighbor.x>=0&&Neighbor.x<rm.len&&
			Neighbor.y>=0&&Neighbor.y<rm.wid&&
			rm.pts[rm.wid*Neighbor.x +Neighbor.y].i&&
			rm.regionID[rm.wid*Neighbor.x +Neighbor.y] == UNKNOWN)
		{
			vec.push_back(Neighbor);
			rm.regionID[rm.wid*Neighbor.x +Neighbor.y] = _regid;
		}
	}
	return true;
}

void    ContourExtraction ()
{
	point3fi *cp,*pt;
	int x,y,xx,yy;

	for (x = 0; x<rm.len; x++)
	{
		for (y = 0; y<rm.wid; y++)
		{
			cp = &rm.pts[rm.wid*x+y];
			if (!cp->i)
			{
				rm.regionID[rm.wid*x +y] = NONVALID;
				continue;
			}

			for (yy = y-1; yy<=y+1; yy++) 
			{
				if(yy<0||yy>=rm.wid)
					continue;
				for (xx = x-1; xx<=x+1; xx++)
				{
					if (xx<0||xx>=rm.len)
						continue;
					if (xx!=x&&yy!=y)
						continue;

					pt = &rm.pts[rm.wid*xx+yy];
					if (!pt->i)
						continue;

                    // 判断是否是上下两个像素
                    bool ishori = (yy == y) ? false : true;
                    if (!isppNeighbor (pt, cp, ishori)) {
						rm.regionID[rm.wid*x +y] = EDGEPT;
						break;
					}
				}
				if (rm.regionID[rm.wid*x +y] == EDGEPT)
					break;
			}
		}
	}
}

void GrowOne(IMCOORDINATE seed, UINT _regid)
{
	vector <IMCOORDINATE> Vec1;
	vector <IMCOORDINATE> Vec2;
	bool isVec1 = true;
	int		j;
	Vec1.clear();
	Vec2.clear();
	Vec1.push_back(seed);
    // 循环队列BFS
	while (1)
	{
		switch (isVec1)
		{
		case true:
			Vec2.clear();
			for (j =0; j<Vec1.size(); j++)
			{
				AddPoints(Vec1[j], Vec2, _regid);
			}
			isVec1 = !isVec1;
			if (Vec2.size()<1)
			{
				return;
			}
			break;
		case false:
			Vec1.clear();
			for (j =0; j<Vec2.size(); j++)
			{
				AddPoints(Vec2[j], Vec1, _regid);
			}
			isVec1 = !isVec1;
			if (Vec1.size()<1)
			{
				return;
			}
			break;
		}
	}
}

UINT RegionGrow()
{
	IMCOORDINATE	seed;
	int m,x1,x2;
	UINT _regid = 1;

	int x,y,yf;
	for (x=0; x<rm.len; x++)
	{
		for (yf = 0; yf<rm.wid; yf++) {
            if (rm.pts[rm.wid*x+yf].i)
				break;
		}
		if(yf >= rm.wid)
			continue;
		for (y = yf; y<rm.wid; y++)
		{
            // 如果是未标记过的点，且在路面高度范围内
            if (rm.regionID[rm.wid*x+y]==UNKNOWN && rm.pts[rm.wid*x+y].z>-3 && rm.pts[rm.wid*x+y].z<3) {
//            if (rm.regionID[rm.wid*x+y]==UNKNOWN && rm.pts[rm.wid*x+y].z>-2 && rm.pts[rm.wid*x+y].z<2) {
				seed.x = x;
				seed.y = y;
                // 区域增长
				GrowOne(seed, _regid);
				_regid++;
			}
		}
	}
	return _regid-1;
}


BOOL ContourSegger()
{
	ContourExtraction();
	
	rm.regnum = RegionGrow()+1;

    return true;
}

void Region2Seg ()
{
	SEGBUF *segbuf;
	int		regionid;

	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
		segbuf->minxp.x = segbuf->minxp.y = segbuf->minxp.z = 9999.0;
		segbuf->minyp.x = segbuf->minyp.y = segbuf->minyp.z = 9999.0;
		segbuf->minzp.x = segbuf->minzp.y = segbuf->minzp.z = 9999.0;
		segbuf->maxxp.x = segbuf->maxxp.y =	segbuf->maxxp.z = -9999.0;
		segbuf->maxyp.x = segbuf->maxyp.y =	segbuf->maxyp.z = -9999.0;
		segbuf->maxzp.x = segbuf->maxzp.y =	segbuf->maxzp.z = -9999.0;
		segbuf->cp.x = segbuf->cp.y = segbuf->cp.z = 0;
		segbuf->dmin.x = rm.wid;
		segbuf->dmin.y = rm.len;
		segbuf->ptnum = 0;
	}

	//为每个regionID，生成一个segbuf，记录该区域块的特征
	int x, y;
	for (y=0; y<rm.len; y++) {
		for (x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;
			regionid = rm.regionID[y*rm.wid+x];
			if (regionid<=0 || regionid>=rm.regnum) 
				continue;

			segbuf = &rm.segbuf[regionid];
            // center point
			segbuf->cp.x += rm.pts[y*rm.wid+x].x;
			segbuf->cp.y += rm.pts[y*rm.wid+x].y;
			segbuf->cp.z += rm.pts[y*rm.wid+x].z;
            // min/max x/y
			segbuf->dmin.x = min (segbuf->dmin.x, x);
			segbuf->dmin.y = min (segbuf->dmin.y, y);
			segbuf->dmax.x = max (segbuf->dmax.x, x);
			segbuf->dmax.y = max (segbuf->dmax.y, y);
            // min/max(x,y,z) point
			if (segbuf->minxp.x>rm.pts[y*rm.wid+x].x) 
				segbuf->minxp = rm.pts[y*rm.wid+x];
			if (segbuf->minyp.y>rm.pts[y*rm.wid+x].y) 
				segbuf->minyp = rm.pts[y*rm.wid+x];
			if (segbuf->minzp.z>rm.pts[y*rm.wid+x].z) 
				segbuf->minzp = rm.pts[y*rm.wid+x];
			if (segbuf->maxxp.x<rm.pts[y*rm.wid+x].x) 
				segbuf->maxxp = rm.pts[y*rm.wid+x];
			if (segbuf->maxyp.y<rm.pts[y*rm.wid+x].y) 
				segbuf->maxyp = rm.pts[y*rm.wid+x];
			if (segbuf->maxzp.z<rm.pts[y*rm.wid+x].z) 
				segbuf->maxzp = rm.pts[y*rm.wid+x];
			segbuf->ptnum ++;
		}
	}

	double Equation[4];
	double WX[6],WY[6],WZ[6];
	int num;
	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
        if (segbuf->ptnum<100) {
//        if (segbuf->ptnum<300) {
			segbuf->ptnum=0;
			continue;
		}
		segbuf->cp.x /= (double)segbuf->ptnum;
		segbuf->cp.y /= (double)segbuf->ptnum;
		segbuf->cp.z /= (double)segbuf->ptnum;

		WX[0]=segbuf->minxp.x;WX[1]=segbuf->minyp.x;WX[2]=segbuf->minzp.x;WX[3]=segbuf->maxxp.x;WX[4]=segbuf->maxyp.x;WX[5]=segbuf->maxzp.x;
		WY[0]=segbuf->minxp.y;WY[1]=segbuf->minyp.y;WY[2]=segbuf->minzp.y;WY[3]=segbuf->maxxp.y;WY[4]=segbuf->maxyp.y;WY[5]=segbuf->maxzp.y;
		WZ[0]=segbuf->minxp.z;WZ[1]=segbuf->minyp.z;WZ[2]=segbuf->minzp.z;WZ[3]=segbuf->maxxp.z;WZ[4]=segbuf->maxyp.z;WZ[5]=segbuf->maxzp.z;
		num = 6;
		Calculate_Plane(num,WX,WY,WZ,0,Equation);
		Calculate_Residuals(&segbuf->cp.x,&segbuf->cp.y,&segbuf->cp.z,Equation,&segbuf->var,1);

		if (Equation[2]<0) {
			segbuf->norm.x = -Equation[0];
			segbuf->norm.y = -Equation[1];
			segbuf->norm.z = -Equation[2];
		}
		else {
			segbuf->norm.x = Equation[0];
			segbuf->norm.y = Equation[1];
			segbuf->norm.z = Equation[2];
		}
        if (segbuf->norm.z<0.9)
			segbuf->ptnum=0;
		else 
			segbuf->ptnum=segbuf->ptnum;
	}
}

