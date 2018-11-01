#include "define.h"

TRANSINFO	calibInfo;

//HANDLE	dfp=INVALID_HANDLE_VALUE;
FILE    *dfp;
int		dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);
int		dFrmNum=0;
int		dFrmNo=0;
int     idxRcsPointCloud=0;

RMAP	rm;
DMAP	dm;
DMAP	gm, ggm;

ONEDSVFRAME	*onefrm;
IplImage * col;
IplImage *demVis;
CvFont font;
list<point2d> trajList;

bool LoadCalibFile (char *szFile)
{
	char			i_line[200];
    FILE			*fp;
	MATRIX			rt;

	fp = fopen (szFile, "r");
	if (!fp) 
		return false;

	rMatrixInit (calibInfo.rot);

	int	i = 0;
	while (1) {
		if (fgets (i_line, 80, fp) == NULL)
			break;

        if (strncmp(i_line, "rot", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.ang.x = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.y = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.z = atof (strtok (NULL, " ,\t\n"))*topi;
			createRotMatrix_ZYX (rt, calibInfo.ang.x, calibInfo.ang.y, calibInfo.ang.z);
			rMatrixmulti (calibInfo.rot, rt);
			continue;
		}

        if (strncmp (i_line, "shv", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.shv.x = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.y = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.z = atof (strtok (NULL, " ,\t\n"));
		}
	}
	fclose (fp);

	return true;
}

void SmoothingData ()
{
	int maxcnt = 3;

	for (int y=0; y<rm.len; y++) {
		for (int x=1; x<(rm.wid-1); x++) {
			if (rm.pts[y*rm.wid+(x-1)].i && !rm.pts[y*rm.wid+x].i) {

				int xx;
				for (xx=x+1; xx<rm.wid; xx++) {
					if (rm.pts[y*rm.wid+xx].i)
						break;
				}
				if (xx>=rm.wid)
					continue;
				int cnt = xx-x+1;
				if (cnt>maxcnt) {
					x = xx;
					continue;
				}
				point3fi *p1 = &rm.pts[y*rm.wid+(x-1)];
				point3fi *p2 = &rm.pts[y*rm.wid+xx];
				double dis = ppDistance3fi (p1, p2);
				double rng = max(p2r(p1),p2r(p2));
				double maxdis = min(MAXSMOOTHERR, max (BASEERROR, HORIERRFACTOR*cnt*rng));
				if (dis<maxdis) {
					for (int xxx=x; xxx<xx; xxx++) {
						point3fi *p = &rm.pts[y*rm.wid+xxx];
						p->x = (p2->x-p1->x)/cnt*(xxx-x+1)+p1->x;
						p->y = (p2->y-p1->y)/cnt*(xxx-x+1)+p1->y;
						p->z = (p2->z-p1->z)/cnt*(xxx-x+1)+p1->z;
						p->i = 1;
					}
				}
				x = xx;
			}
		}
	}
}

void CorrectPoints ()
{
	MAT2D	rot1, rot2;

	//transform points to the vehicle frame of onefrm->dsv[0]
	//src: block i; tar: block 0

    //rot2: R_tar^{-1}
	rot2[0][0] = cos (-onefrm->dsv[0].ang.z);
	rot2[0][1] = -sin (-onefrm->dsv[0].ang.z);
	rot2[1][0] = sin (-onefrm->dsv[0].ang.z);
    rot2[1][1] = cos (-onefrm->dsv[0].ang.z);

	for (int i=1; i<BKNUM_PER_FRM; i++) {
		for (int j=0; j<PTNUM_PER_BLK; j++) {
			if (!onefrm->dsv[i].points[j].i)
				continue;

			rotatePoint3fi(onefrm->dsv[i].points[j], calibInfo.rot);
			shiftPoint3fi(onefrm->dsv[i].points[j], calibInfo.shv); 
			rotatePoint3fi(onefrm->dsv[i].points[j], onefrm->dsv[i].rot);

			//rot1: R_tar^{-1}*R_src
			rot1[0][0] = cos (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[0][1] = -sin (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[1][0] = sin (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[1][1] = cos (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);

			//shv: SHV_src-SHV_tar
			point2d shv;
            shv.x = onefrm->dsv[i].shv.x-onefrm->dsv[0].shv.x;
            shv.y = onefrm->dsv[i].shv.y-onefrm->dsv[0].shv.y;

			point2d pp;
			pp.x = onefrm->dsv[i].points[j].x; pp.y = onefrm->dsv[i].points[j].y;
			rotatePoint2d (pp, rot1);	//R_tar^{-1}*R_src*p
			rotatePoint2d (shv, rot2);	//R_tar^{-1}*(SHV_src-SHV_tar)
			shiftPoint2d (pp, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
			onefrm->dsv[i].points[j].x = pp.x;
			onefrm->dsv[i].points[j].y = pp.y;
		}
	}

	for (int ry=0; ry<rm.len; ry++) {
		for (int rx=0; rx<rm.wid; rx++) {
			int i=rm.idx[ry*rm.wid+rx].x;
			int j=rm.idx[ry*rm.wid+rx].y;
			if (!i&&!j)
				continue;
			rm.pts[ry*rm.wid+rx] = onefrm->dsv[i].points[j];
		}
	}

    // 记录历史轨迹
    trajList.push_front(point2d{onefrm->dsv[0].shv.x, onefrm->dsv[0].shv.y});
    if (trajList.size() > 2000)
        trajList.pop_back();
}

void ProcessOneFrame ()
{
	//生成距离图像帧
	GenerateRangeView ();

	//根据calib参数将激光点转换到车体坐标系，根据车体角度roll、pitch修正激光帧到水平，数据点转换到第0个数据包航向角和位移所对应的车体坐标系
	CorrectPoints ();	

	//对每一行数据中短暂无效激光点（cnt<5，约水平1度)进行内插补齐，否则这些无效点处会被认为是边界点
	SmoothingData ();

    //分割出路面区域
	//第一步：标注边界点ContourExtraction();
	//第二步：区域增长方式标注区域内点RegionGrow()
	memset (rm.regionID, 0, sizeof(int)*rm.wid*rm.len);
	rm.regnum = 0;
	ContourSegger ();
	
	//为每个区域生成一个segbuf，用于分类、目前仅提取了少量特征
    if (rm.regnum) {
		rm.segbuf = new SEGBUF[rm.regnum];
		memset (rm.segbuf, 0, sizeof (SEGBUF)*rm.regnum);
        Region2Seg ();
	}	
	//生成可视化距离图像处理结果
    DrawRangeView ();
	
    //将全局DEM转换到当前车体坐标系下
    PredictGloDem (gm,ggm);

	//生成单帧数据的DEM
	GenerateLocDem (dm);

    //用当前帧DEM更新全局DEM
    UpdateGloDem (gm,dm);

    //提取道路中心线
    ExtractRoadCenterline (gm);

    //计算地面俯仰和横滚角，分类地形（上下坡）(只给可通行区域打标签)
    LabelRoadSurface (gm);

    //提取路面上的障碍物（凹凸障碍）
    LabelObstacle (gm);

	//生成可视化单帧数据DEM
	DrawDem (dm);

    //生成可视化全局DEM
	DrawDem (gm);

	if (rm.segbuf)
		delete []rm.segbuf;

}

//读取一帧vel64数据（一帧为580×12×32个激光点）保存到onefrm->dsv，未作坐标转换
BOOL ReadOneDsvFrame ()
{
	DWORD	dwReadBytes;
	int		i;
    for (i=0; i<BKNUM_PER_FRM; i++) {
        dwReadBytes = fread((ONEDSVDATA *)&onefrm->dsv[i], 1, dsbytesiz, dfp);
        if ((dsbytesiz != dwReadBytes) || (ferror(dfp))) {
            printf("Error from reading file.\n");
			break;
        }
//		createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , onefrm->dsv[i].ang.z ) ; 
        createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , 0 ) ;

        // 滤掉车身上的点 for guilin
        for (int j=0; j<LINES_PER_BLK; j++) {
            for (int k=0; k<PNTS_PER_LINE; k++) {
                point3fi *p = &onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
                double dis2Vehicle = sqrt(sqr(p->x)+sqr(p->y)+sqr(p->z));
                if (dis2Vehicle < 4.0) {    // m
                    p->i = 0;
                }
            }
        }
    }
	if (i<BKNUM_PER_FRM)
        return false;
	else
        return true;
}


void CallbackLocDem(int event, int x, int y, int flags, void *ustc)
{
    static CvPoint lu, rb;

    if (event == CV_EVENT_LBUTTONDOWN) {
        lu.x = x; lu.y = y;
    }
    if (event == CV_EVENT_LBUTTONUP) {

        rb.x = x; rb.y = y;
        IplImage *tmp = cvCreateImage (cvSize (dm.wid, dm.len),IPL_DEPTH_8U,3);
        cvCopy (dm.lmap, tmp);
        cvRectangle (dm.lmap, lu, rb, cvScalar(255, 255, 0), 3);
        cvShowImage("ldemlab",dm.lmap);

        int ix, iy;
        for (iy=min(lu.y,rb.y); iy<=max(lu.y,rb.y); iy++)
            for (ix=min(lu.x,rb.x); ix<=max(lu.x,rb.x); ix++)
                printf("%d, %d, %.3f,%.3f\n", ix, iy, dm.demg[iy*dm.wid+ix], dm.demhmin[iy*dm.wid+ix]);
        cvReleaseImage(&tmp);
    }
}

// 获取文件大小
LONGLONG myGetFileSize(FILE *f)
{
    // set the file pointer to end of file
    fseeko(f, 0, SEEK_END);
    // get the file size
    LONGLONG retSize = ftello(f);
    // return the file pointer to the begin of file
    rewind(f);
    return retSize;
}

// 绘制历史轨迹
void DrawTraj(IplImage *img)
{
    MAT2D	rot2;
    list<point2d>::iterator iter;
    iter = trajList.begin();
    point2d centerPoint = point2d{iter->x, iter->y};
    point2i centerPixel = point2i{img->height/2, img->width/2};

    for (iter = trajList.begin(); iter != trajList.end(); iter ++) {
        point2d tmpPoint;
        tmpPoint.x = centerPixel.y;
        tmpPoint.y = centerPixel.x;

        //rot2: R_tar^{-1}
        rot2[0][0] = cos (-onefrm->dsv[0].ang.z);
        rot2[0][1] = -sin (-onefrm->dsv[0].ang.z);
        rot2[1][0] = sin (-onefrm->dsv[0].ang.z);
        rot2[1][1] = cos (-onefrm->dsv[0].ang.z);

        //shv: SHV_src-SHV_tar
        point2d shv;
        shv.x = (iter->x - centerPoint.x) / PIXSIZ;
        shv.y = (iter->y - centerPoint.y) / PIXSIZ;
//        printf("centerPoint : (%lf,%lf)\n", centerPoint.x, centerPoint.y);
//        printf("centerPixel : (%d,%d)\n", centerPixel.x, centerPixel.y);
//        printf("shv: (%lf,%lf)\n", shv.x, shv.y);

        rotatePoint2d (shv, rot2);          //R_tar^{-1}*(SHV_src-SHV_tar)
        shiftPoint2d (tmpPoint, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
        cvCircle(img, cvPoint((int)tmpPoint.x, (int)tmpPoint.y), 3, cv::Scalar(255,255,255), -1, 8);
//        printf("Draw point: (%d,%d)\n\n", (int)tmpPoint.x, (int)tmpPoint.y);
    }
}

// 获取下一个激光点数据
//point3fi GetNextRcsPoint(P_CGQHDL64E_INFO_MSG *veloData)
//{
//    /*
//    offset:0 datatype:7 count:1 name:x
//    offset:4 datatype:7 count:1 name:y
//    offset:8 datatype:7 count:1 name:z
//    offset:16 datatype:7 count:1 name:intensity
//    offset:20 datatype:4 count:1 name:ring
//    */
//    point3fi ret;
//    ret.x = ret.y = ret.z = ret.i = 0;
//    if (idxRcsPointCloud >= veloData->data_length) {
//        return ret;
//    }
//    // X
//    unsigned char *p_uchar = (unsigned char *)&ret.x;
//    for (int i = 0; i < 4; i ++) {
//        p_uchar[i] = veloData->data[idxRcsPointCloud + i];
//    }
//    idxRcsPointCloud += 4;
//    // Y
//    p_uchar = (unsigned char *)&ret.y;
//    for (int i = 0; i < 4; i ++) {
//        p_uchar[i] = veloData->data[idxRcsPointCloud + i];
//    }
//    idxRcsPointCloud += 4;
//    // Z
//    p_uchar = (unsigned char *)&ret.z;
//    for (int i = 0; i < 4; i ++) {
//        p_uchar[i] = veloData->data[idxRcsPointCloud + i];
//    }
//    idxRcsPointCloud += 8;
//    // Intensity
//    float tmpIntensity = 0;
//    p_uchar = (unsigned char *)&tmpIntensity;
//    for (int i = 0; i < 4; i ++) {
//        p_uchar[i] = veloData->data[idxRcsPointCloud + i];
//    }
//    ret.i = tmpIntensity;
//    idxRcsPointCloud += 16;
//    // Coordinate transform
//    ret.y *= -1;
//    // 滤除车身上的点
//    if (sqrt(sqr(ret.x)+sqr(ret.y)+sqr(ret.z)) < 3.5)
//        ret = point3fi{0, 0, 0, 0};
//    return ret;
//}

// 读入RCS格式数据，并保存到onefrm中
//BOOL ReadRcsData(P_CGQHDL64E_INFO_MSG *veloData, P_DWDX_INFO_MSG *dwdxData)
//{
//    idxRcsPointCloud = 0;
//    int		i;
//    for (i=0; i<BKNUM_PER_FRM; i++) {
//        // get and convert roll/pitch/yaw to radian
//        onefrm->dsv[i].ang = point3d{(double)dwdxData->roll * 0.01 / 180.0 * M_PI,
//                                    (double)dwdxData->pitch * 0.01 / 180.0 * M_PI,
//                                    (double)dwdxData->heading * 0.01 / 180.0 * M_PI - M_PI/2.0};
//        // time stamp
//        onefrm->dsv[i].millisec = veloData->header.stamp;
//        onefrm->dsv[i].shv = point3d{(double)dwdxData->global_x * 0.1, (double)dwdxData->global_y * 0.1, (double)dwdxData->global_h * 0.1};
//        for (int j = 0; j < PTNUM_PER_BLK; j ++) {
//            onefrm->dsv[i].points[j] = GetNextRcsPoint(veloData);
//        }
//        createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , 0 ) ;
//    }
//    return true;
//}

// DoProcessing之前的预处理
void PrePorcessing()
{
    // 读入Velodyne外参标定文件
    if (!LoadCalibFile ("/home/gaobiao/Documents/201-2018/ANS_test_pku/DsvSegRegion_RCS/vel.calib")) {
        printf ("Invalid calibration file\n");
        getchar ();
        exit (1);
    }
    InitRmap (&rm);
    InitDmap (&dm);
    InitDmap (&gm);
    InitDmap (&ggm);
    onefrm= new ONEDSVFRAME[1];
    col = cvCreateImage (cvSize (1024, rm.len*3),IPL_DEPTH_8U,3);
    cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1,1, 0, 2);

    dFrmNo = 0;
}

// DoProcessing后处理，释放内存空间
void PostProcessing()
{
    ReleaseRmap (&rm);
    ReleaseDmap (&dm);
    ReleaseDmap (&gm);
    ReleaseDmap (&ggm);
    cvReleaseImage(&col);
    delete []onefrm;
}

// 将结果写入demMap
//void WriteDemMap(unsigned long stamp, P_DWDX_INFO_MSG *dwdxData, IplImage* zmap, P_CJDEMMAP_MSG &demMap)
//{
//    demMap.time_stamp = stamp;
//    demMap.fc.time_label = dwdxData->time_stamp;
//    demMap.fc.global_x = dwdxData->global_x;
//    demMap.fc.global_y = dwdxData->global_y;
//    demMap.fc.global_h = dwdxData->global_h;
//    demMap.fc.zone = dwdxData->zone;
//    demMap.fc.heading = dwdxData->heading;
//    demMap.fc.pitch = dwdxData->pitch;
//    demMap.fc.roll = dwdxData->roll;
//    demMap.fc.global_vx = dwdxData->global_vx;
//    demMap.fc.global_vy = dwdxData->global_vy;
//    demMap.fc.global_vh = dwdxData->global_vz;
//    demMap.fc.heading_rate = dwdxData->global_wx;
//    demMap.fc.pitch_rate = dwdxData->global_wy;
//    demMap.fc.roll_rate = dwdxData->global_wz;
//    demMap.fc.longitude = dwdxData->longitude;
//    demMap.fc.latitude = dwdxData->latitude;
//    demMap.map_height = 325;
//    demMap.map_width = 150;
//    demMap.grid_height = 20;    // cm
//    demMap.grid_width = 20;     // cm
//    demMap.vehicle_gridX = 250;
//    demMap.vehicle_gridY = 75;
//    demMap.local_cjdemmap_length = 0;

//    demVis = cvCreateImage(cvSize(demMap.map_width, demMap.map_height), IPL_DEPTH_8U, 1);
//    cvZero(demVis);
//    int x0 = zmap->height/2 - 75;
//    int x1 = zmap->height/2 + 250;
//    int y0 = zmap->width/2 - 75;
//    int y1 = zmap->width/2 + 75;
//    int step = demVis->widthStep / sizeof(uchar);
//    int zstep = zmap->widthStep / sizeof(uchar);

//    for (int x = x1; x > x0; x --)
//        for (int y = y1; y > y0; y --) {
//            demMap.local_cjdemmap[demMap.local_cjdemmap_length ++] = zmap->imageData[x * zstep + y];
//            demVis->imageData[(x1 - x) * step + (y1 - y)] = zmap->imageData[x * zstep + y];
//        }
//    cvShowImage("DEM", demVis);
//    cvReleaseImage(&demVis);
//}
// 将结果写入AttributeMap
//void WriteAttributeMap(unsigned long stamp, P_DWDX_INFO_MSG *dwdxData, IplImage* pmap, P_CJATTRIBUTEMAP_MSG &attributeMap)
//{
//    attributeMap.time_stamp = stamp;
//    attributeMap.fc.time_label = dwdxData->time_stamp;
//    attributeMap.fc.global_x = dwdxData->global_x;
//    attributeMap.fc.global_y = dwdxData->global_y;
//    attributeMap.fc.global_h = dwdxData->global_h;
//    attributeMap.fc.zone = dwdxData->zone;
//    attributeMap.fc.heading = dwdxData->heading;
//    attributeMap.fc.pitch = dwdxData->pitch;
//    attributeMap.fc.roll = dwdxData->roll;
//    attributeMap.fc.global_vx = dwdxData->global_vx;
//    attributeMap.fc.global_vy = dwdxData->global_vy;
//    attributeMap.fc.global_vh = dwdxData->global_vz;
//    attributeMap.fc.heading_rate = dwdxData->global_wx;
//    attributeMap.fc.pitch_rate = dwdxData->global_wy;
//    attributeMap.fc.roll_rate = dwdxData->global_wz;
//    attributeMap.fc.longitude = dwdxData->longitude;
//    attributeMap.fc.latitude = dwdxData->latitude;
//    attributeMap.map_height = 325;
//    attributeMap.map_width = 150;
//    attributeMap.grid_height = 20;    // cm
//    attributeMap.grid_width = 20;     // cm
//    attributeMap.vehicle_gridX = 250;
//    attributeMap.vehicle_gridY = 75;
//    attributeMap.local_cjattributemap_length = 0;

//    int x0 = pmap->height/2 - 75;
//    int x1 = pmap->height/2 + 250;
//    int y0 = pmap->width/2 - 75;
//    int y1 = pmap->width/2 + 75;
//    int pstep = pmap->widthStep / sizeof(uchar);

//    for (int x = x1; x > x0; x --)
//        for (int y = y1; y > y0; y --) {
//            // Probability of trafficability
//            int newProb = double(pmap->imageData[x * pstep + y] & 0xFF) / 2.55;
//            // Label of pixel
//            int newLabel = 0;
//            attributeMap.local_cjattributemap[attributeMap.local_cjattributemap_length ++] = newProb;
//        }
//}

// 在线处理程序
//void DoProcessing(P_CGQHDL64E_INFO_MSG *veloData, P_DWDX_INFO_MSG *dwdxData, P_CJDEMMAP_MSG &demMap, P_CJATTRIBUTEMAP_MSG &attributeMap)
//{
//    if (ReadRcsData(veloData, dwdxData))  // 读取一帧数据（580个block），保存在onefrm中
//    {
//        if (dFrmNo%100==0)
//            printf("%d (%d)\n",dFrmNo,dFrmNum);

//        //每一帧的处理
//        ProcessOneFrame ();

//        //将结果写到DemMap和AttributeMap里
//        WriteDemMap(veloData->header.stamp, dwdxData, gm.zmap, demMap);
//        WriteAttributeMap(veloData->header.stamp, dwdxData, gm.pmap, attributeMap);

//        //绘制历史轨迹
//        DrawTraj(dm.lmap);
//        DrawTraj(gm.smap);

//        //可视化
//        char str[10];
//        sprintf (str, "Fno%d", dFrmNo);
//        cvPutText(dm.lmap, str, cvPoint(50,50), &font, CV_RGB(0,0,255));

//        cvResize (rm.rMap, col);  // 距离图像 可视化
//        cvShowImage("range image",col);
//        cvResize (rm.lMap, col);  // 分割图像 可视化
//        cvShowImage("region",col);
//        cv::Mat zmapRGB;          // 高程图 伪彩可视化
//        cv::applyColorMap(cvarrToMat(gm.zmap), zmapRGB, cv::COLORMAP_HOT);
//        cv::Mat pmapRGB;          // 通行概率图 伪彩可视化
//        cv::applyColorMap(cvarrToMat(gm.pmap), pmapRGB, cv::COLORMAP_BONE);

//        if (dm.lmap) cvShowImage("l_dem",dm.lmap);    // 单帧 可行驶区域
//        if (gm.zmap) cv::imshow("g_zdem", zmapRGB);      // 高程图
//        if (gm.pmap) cv::imshow("g_pdem", pmapRGB);    // 多帧 可行驶概率图
//        if (gm.smap) cvShowImage("g_sublab",gm.smap);    // 属性图

//        char WaitKey;
//        WaitKey = cvWaitKey(1);
//        dFrmNo++;

//    }
//}

void Cvt2Gt(IplImage* img, cv::Mat &gtMap)
{
    int step = img->widthStep / (sizeof(uchar)*3);
    gtMap.setTo(cv::Scalar::all(0));
    for (int i = 0; i < img->height; i ++)
        for (int j = 0; j < img->width; j ++) {
            // 可通行
            if (img->imageData[(i * step + j) * 3 + 1] != 0) {
                gtMap.at<uchar>(i, j) = 1;
            }
            // 不可通行
            else if (img->imageData[(i * step + j) * 3] != 0 || img->imageData[(i * step + j) * 3 + 2] != 0) {
                gtMap.at<uchar>(i, j) = 2;
            }
        }
}

//主处理程序（离线）
void DoProcessingOffline(/*P_CGQHDL64E_INFO_MSG *veloData, P_DWDX_INFO_MSG *dwdxData, P_CJDEMMAP_MSG &demMap, P_CJATTRIBUTEMAP_MSG &attributeMap*/)
{
    if (!LoadCalibFile ("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/vel_hongling.calib")) {
        printf ("Invalid calibration file\n");
        getchar ();
        exit (1);
    }
    if ((dfp = fopen("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/hongling_round1_1.dsv", "r")) == NULL) {
        printf("File open failure\n");
        getchar ();
        exit (1);
    }
    VideoCapture cap("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/1.avi");
    FILE* tsFp = fopen("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/1.avi.ts", "r");
    if (!cap.isOpened()) {
        printf("Error opening video stream or file.\n");
        getchar();
        exit(1);
    }

    LONGLONG fileSize = myGetFileSize(dfp);
    dFrmNum = fileSize / 580 / dsbytesiz;
	InitRmap (&rm);
	InitDmap (&dm);
	InitDmap (&gm);
	InitDmap (&ggm);
	onefrm= new ONEDSVFRAME[1];
	IplImage * col = cvCreateImage (cvSize (1024, rm.len*3),IPL_DEPTH_8U,3); 
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX, 1,1, 0, 2);
    int waitkeydelay=0;
	dFrmNo = 0;
    // Name window
    cv::namedWindow("region");
    cv::namedWindow("range image");
    cv::namedWindow("g_zdem");
    cv::namedWindow("g_sublab");
    cv::namedWindow("g_pdem");
    cv::namedWindow("l_dem");
    cv::namedWindow("video");
    cv::moveWindow("region", 0, LENSIZ/PIXSIZ + 200);
    cv::moveWindow("range image", 0, LENSIZ/PIXSIZ + 400);
    cv::moveWindow("g_zdem", 0, 0);
    cv::moveWindow("g_pdem", WIDSIZ*3.3/PIXSIZ, 0);
    cv::moveWindow("g_sublab", WIDSIZ*1.3/PIXSIZ, 0);
    cv::moveWindow("l_dem", WIDSIZ*2.3/PIXSIZ, 0);
    cv::moveWindow("video", 1000, 500);

    printf("size of ONEDSVDATA: %d\n", sizeof(ONEDSVDATA));
    printf("size of MATRIX: %d\n", sizeof(MATRIX));
    while (ReadOneDsvFrame ())  // 读取一帧数据（580个block），保存在onefrm中
	{
		if (dFrmNo%100==0)
			printf("%d (%d)\n",dFrmNo,dFrmNum);

        //每一帧的处理
        ProcessOneFrame ();

        //绘制历史轨迹
        DrawTraj(dm.lmap);
        DrawTraj(gm.smap);

        //可视化
        char str[10];
        sprintf (str, "%d", onefrm->dsv[0].millisec);
        cvPutText(dm.lmap, str, cvPoint(30,30), &font, CV_RGB(255,255,255));

        cvResize (rm.rMap, col);  // 距离图像 可视化
        cvShowImage("range image",col);
        cvResize (rm.lMap, col);  // 分割图像 可视化
        cvShowImage("region",col);
        cv::Mat zmapRGB;          // 高程图 伪彩可视化
        cv::applyColorMap(cvarrToMat(gm.zmap), zmapRGB, cv::COLORMAP_HOT);
        cv::Mat pmapRGB;          // 通行概率图 伪彩可视化
        cv::applyColorMap(cvarrToMat(gm.pmap), pmapRGB, cv::COLORMAP_BONE);

        // 可视化视频
        cv::Mat vFrame;
        int vTs;
        cap >> vFrame;
        fscanf(tsFp, "%d\n", &vTs);
        // 找到和激光匹配的视频帧
        while (vTs < onefrm->dsv[0].millisec) {
            cap >> vFrame;
            fscanf(tsFp, "%d\n", &vTs);
        }
        cv::resize(vFrame, vFrame, cv::Size(vFrame.cols / 3, vFrame.rows / 3));
        if (!vFrame.empty()) cv::imshow("video", vFrame);

        if (dm.lmap) cvShowImage("l_dem",dm.lmap);    // 单帧 可行驶区域
        if (gm.zmap) cv::imshow("g_zdem", zmapRGB);      // 高程图
        if (gm.pmap) cv::imshow("g_pdem", pmapRGB);    // 多帧 可行驶概率图
        if (gm.smap) cvShowImage("g_sublab",gm.smap);    // 属性图


        // 将图片保存为png格式，用作训练/测试数据
        cv::Mat gtMap(gm.smap->height, gm.smap->width, CV_8UC1);
        if (dFrmNo > 45) {
//            stringstream s_fno;
//            s_fno << setw(6) << setfill('0') << dFrmNo;
//            std::string DATA_PATH = "/home/gaobiao/Documents/RoadSegmentation_IV2019/data/image/";
//            cv::imwrite(DATA_PATH + s_fno.str() + "_img.png", cv::cvarrToMat(gm.zmap));
//            Cvt2Gt(gm.smap, gtMap);
//            cv::imwrite(DATA_PATH + s_fno.str() + "_gt.png", gtMap);
        }

//        cv::setMouseCallback("gsublab", CallbackLocDem, 0);

		char WaitKey;
		WaitKey = cvWaitKey(waitkeydelay);
		if (WaitKey==27)
			break;
        if (WaitKey=='z') {     // 连续播放
			if (waitkeydelay==1)
				waitkeydelay=0;
			else
				waitkeydelay=1;
		}
        if (WaitKey == 'a') {     // Back
            dFrmNo -= 20;
            if (dFrmNo < 0) {
                dFrmNo = 0;
            }
            fseeko64(dfp, dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
            continue;
        }
        if (WaitKey == 'd') {     // Forword
            dFrmNo += 20;
            if (dFrmNo >= dFrmNum) {
                dFrmNo = dFrmNum - 1;
            }
            fseeko64(dfp, dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
            continue;
        }
        dFrmNo++;
    }

    cap.release();
	ReleaseRmap (&rm);
	ReleaseDmap (&dm);
	ReleaseDmap (&gm);
	ReleaseDmap (&ggm);
	cvReleaseImage(&col);
    delete []onefrm;
}

int main (int argc, char *argv[])
{

//    if (argc<3) {
//        printf ("Usage : %s [infile] [calibfile]\n", argv[0]);
//        printf ("[infile] DSV file.\n");
//        printf ("[calibfile] define the calibration parameters of the DSV file.\n");
//        printf ("[outfile] segmentation results to DSVL file.\n");
//        printf ("[seglog] data association results to LOG file.\n");
//        printf ("[videooutflg] 1: output video to default files, 0: no output.\n");
//        exit(1);
//    }

//    if (!LoadCalibFile (argv[2])) {
//        printf ("Invalid calibration file : %s.\n", argv[2]);
//        getchar ();
//        exit (1);
//    }

//    if ((dfp = fopen(argv[1], "r")) == NULL) {
//        printf("File open failure : %s\n", argv[1]);
//        getchar ();
//        exit (1);
//    }

    DoProcessingOffline ();

    printf ("Done.\n");

    fclose(dfp);

    return 0;
}
