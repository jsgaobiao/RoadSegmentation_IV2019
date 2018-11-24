#include "define.h"

TRANSINFO	calibInfo;

//HANDLE	dfp=INVALID_HANDLE_VALUE;
FILE    *dfp;
FILE    *navFp;
int     navLeft, navRight;
int		dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);
int		dFrmNum=0;
int		dFrmNo=0;
int     idxRcsPointCloud=0;
bool    camCalibFlag=true;

RMAP	rm;
DMAP	dm;
DMAP	gm, ggm;

ONEDSVFRAME	*onefrm;
ONEDSVFRAME	*originFrm;
std::vector<NAVDATA> nav;
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
    GenerateLocDem (dm, gm);

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
                double dis2Vehicle = sqrt(sqr(p->x)+sqr(p->y));
                if (dis2Vehicle < 4.0) {    // m
                    p->i = 0;
                }
            }
        }
    }
    if (camCalibFlag) {
        memcpy(originFrm, onefrm, sizeof(*onefrm));
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

        rotatePoint2d (shv, rot2);          //R_tar^{-1}*(SHV_src-SHV_tar)
        shiftPoint2d (tmpPoint, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
        cvCircle(img, cvPoint((int)tmpPoint.x, (int)tmpPoint.y), 3, cv::Scalar(255,255,255), -1, 8);
    }
}

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

void DrawNav(cv::Mat &img, cv::Mat &weakGT)
{
    MAT2D	rot2;
    list<point2d>::iterator iter;
    iter = trajList.begin();
    point2d centerPoint = point2d{iter->x, iter->y};
    point2i centerPixel = point2i{img.rows/2, img.cols/2};

    // navRight的时间至少比当前时间靠后
    while (nav[navRight].millisec < onefrm->dsv[0].millisec) {
        if (navRight < nav.size()) navRight ++;
                            else   break;
    }
    for (int i = navLeft; i < navRight; i ++) {
        if (sqrt(sqr(nav[navLeft].x - centerPoint.x)+sqr(nav[navLeft].y - centerPoint.y)) > 30.0) {
            navLeft ++;
            continue;
        }
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
        shv.x = (nav[i].x - centerPoint.x) / PIXSIZ;
        shv.y = (nav[i].y - centerPoint.y) / PIXSIZ;

        rotatePoint2d (shv, rot2);          //R_tar^{-1}*(SHV_src-SHV_tar)
        shiftPoint2d (tmpPoint, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
        if ((tmpPoint.x < 0 || tmpPoint.y < 0 || tmpPoint.x > img.rows || tmpPoint.y > img.cols)
             &&(navLeft == i - 1))
            navLeft = i;
        cv::circle(img, cv::Point((int)tmpPoint.x, (int)tmpPoint.y), 3, cv::Scalar(0,255,0), -1, 8);
        // draw weak labels
        cv::circle(weakGT, cv::Point((int)tmpPoint.x, (int)tmpPoint.y), 4, cv::Scalar(1), -1, 8);
    }
    for (; navRight < nav.size(); navRight ++) {
        if (sqrt(sqr(nav[navRight].x - centerPoint.x)+sqr(nav[navRight].y - centerPoint.y)) > 25.0) {
            break;
        }
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
        shv.x = (nav[navRight].x - centerPoint.x) / PIXSIZ;
        shv.y = (nav[navRight].y - centerPoint.y) / PIXSIZ;

        rotatePoint2d (shv, rot2);          //R_tar^{-1}*(SHV_src-SHV_tar)
        shiftPoint2d (tmpPoint, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
        if (tmpPoint.x < 0 || tmpPoint.y < 0 || tmpPoint.x > img.rows || tmpPoint.y > img.cols)
            break;
        cv::circle(img, cv::Point((int)tmpPoint.x, (int)tmpPoint.y), 3, cv::Scalar(0,255,0), -1, 8);
        // draw weak labels
        cv::circle(weakGT, cv::Point((int)tmpPoint.x, (int)tmpPoint.y), 4, cv::Scalar(1), -1, 8);
    }
}

void DrawObs(cv::Mat &img, cv::Mat &weakGT)
{
    for (int y=0; y<gm.len; y++) {
        for (int x=0; x<gm.wid; x++) {
            if (gm.demhnum[y*gm.wid+x] > 300 && !gm.demgnum[y*gm.wid+x]) {
                weakGT.at<uchar>(y, x) = 2;
                img.at<Vec3b>(y, x)[0] = 0;
                img.at<Vec3b>(y, x)[1] = 0;
                img.at<Vec3b>(y, x)[2] = 255;
            }
        }
    }
}

void DrawUnlabeled(cv::Mat &img, cv::Mat &weakGT)
{
    for (int y=0; y<img.rows; y++) {
        for (int x=0; x<img.cols; x++) {
            int sigma = img.at<Vec3b>(y, x)[0] + img.at<Vec3b>(y, x)[1] + img.at<Vec3b>(y, x)[2];
            // Unlabeled pixels with observation
            if (sigma > 0 && weakGT.at<uchar>(y, x) == 0) {
                weakGT.at<uchar>(y, x) = 3;
            }
        }
    }
}

void LoadNav()
{
    int millisec, stat;
    double gx, gy, gz, roll, pitch, yaw;
    while (fscanf(navFp, "%d %lf %lf %lf %lf %lf %lf %d\n", &millisec, &roll, &pitch, &yaw, &gx, &gy, &gz, &stat) != EOF) {
        nav.push_back((NAVDATA){millisec, gx, gy, gz, roll, pitch, yaw, stat});
    }
    navLeft = 0;
    navRight = 0;
    printf("size of NAV: %d\n", nav.size());
}

//主处理程序（离线）
void DoProcessingOffline(/*P_CGQHDL64E_INFO_MSG *veloData, P_DWDX_INFO_MSG *dwdxData, P_CJDEMMAP_MSG &demMap, P_CJATTRIBUTEMAP_MSG &attributeMap*/)
{
    if (!LoadCalibFile ("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/vel_hongling.calib")) {
        printf ("Invalid calibration file\n");
        getchar ();
        exit (1);
    }
    // dsv
    if ((dfp = fopen("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/hongling_round1_0.dsv", "r")) == NULL) {
        printf("File open failure\n");
        getchar ();
        exit (1);
    }
    // video
    VideoCapture cap("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/0.avi");
    FILE* tsFp = fopen("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/0.avi.ts", "r");
    if (!cap.isOpened()) {
        printf("Error opening video stream or file.\n");
        getchar();
        exit(1);
    }
    // nav
    if ((navFp = fopen("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/all.nav", "r")) == NULL) {
        printf("Nav open failure\n");
        getchar ();
        exit (1);
    }
    LoadNav();
    // Camera/Velodyne calib file
    if(!LoadCameraCalib("/home/gaobiao/Documents/RoadSegmentation_IV2019/src/DsvSegRegion/Sampledata-001-Camera.camera")){
        printf("Open Camera Calibration files fails.\n");
        camCalibFlag = false;
    }

    LONGLONG fileSize = myGetFileSize(dfp);
    dFrmNum = fileSize / (BKNUM_PER_FRM) / dsbytesiz;
	InitRmap (&rm);
	InitDmap (&dm);
	InitDmap (&gm);
	InitDmap (&ggm);
	onefrm= new ONEDSVFRAME[1];
    if (camCalibFlag) {
        originFrm = new ONEDSVFRAME[1];
    }
	IplImage * col = cvCreateImage (cvSize (1024, rm.len*3),IPL_DEPTH_8U,3); 
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX, 1,1, 0, 2);
    int waitkeydelay=0;
	dFrmNo = 0;
    // Name window
    cv::namedWindow("region");
//    cv::namedWindow("range image");
    cv::namedWindow("g_zdem");
    cv::namedWindow("g_sublab");
//    cv::namedWindow("g_pdem");
    cv::namedWindow("l_dem");
    cv::namedWindow("video");
    cv::namedWindow("label_img");
    cv::moveWindow("region", 0, LENSIZ/PIXSIZ + 70);
//    cv::moveWindow("range image", 0, LENSIZ/PIXSIZ + 400);
    cv::moveWindow("g_zdem", 0, 0);
//    cv::moveWindow("g_pdem", WIDSIZ*3.3/PIXSIZ, 0);
    cv::moveWindow("g_sublab", WIDSIZ*1.3/PIXSIZ, 0);
    cv::moveWindow("l_dem", WIDSIZ*2.3/PIXSIZ, 0);
    cv::moveWindow("video", 0, LENSIZ * 2 /PIXSIZ);
    cv::moveWindow("label_img", WIDSIZ*3.5/PIXSIZ, 0);

    printf("size of ONEDSVDATA: %d\n", sizeof(ONEDSVDATA));
    printf("size of MATRIX: %d\n", sizeof(MATRIX));
    while (ReadOneDsvFrame ())  // 读取一帧数据，保存在onefrm中
	{
		if (dFrmNo%100==0)
			printf("%d (%d)\n",dFrmNo,dFrmNum);

        //每一帧的处理
        ProcessOneFrame ();

        //绘制历史轨迹
        DrawTraj(dm.lmap);
//        DrawTraj(gm.smap);

        //可视化

//        cvResize (rm.rMap, col);  // 距离图像 可视化
//        cvShowImage("range image",col);
        cvResize (rm.lMap, col);  // 分割图像 可视化
        cvShowImage("region",col);
        cv::Mat zmapRGB;          // 高程图 伪彩可视化
        cv::applyColorMap(cvarrToMat(gm.zmap), zmapRGB, cv::COLORMAP_HOT);
        cv::Mat pmapRGB;          // 通行概率图 伪彩可视化
        cv::applyColorMap(cvarrToMat(gm.pmap), pmapRGB, cv::COLORMAP_BONE);

        // 可视化视频(要在激光点进行校准前，使用原始激光数据)
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
        if (!vFrame.empty()) {
            cv::imshow("video", vFrame);
            if (camCalibFlag) {
                cv::Mat pvFrame = vFrame.clone();
                pointCloudsProject(pvFrame, gm);
                cv::imshow("video & point cloud", pvFrame);
            }
        }

        cv::Mat visImg;
        if (dm.lmap) {    // 单帧 可行驶区域
            cv::flip(cv::cvarrToMat(dm.lmap),visImg,0);
            char str[10];
            sprintf (str, "%d", onefrm->dsv[0].millisec);
            cv:putText(visImg, str, cvPoint(30,30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,255,255));
            cv::imshow("l_dem",visImg);
        }
        if (gm.zmap) {cv::flip(zmapRGB,visImg,0); cv::imshow("g_zdem", visImg);}      // 高程图
//        if (gm.pmap) cv::imshow("g_pdem", pmapRGB);    // 多帧 可行驶概率图
        if (gm.smap) {cv::flip(cv::cvarrToMat(gm.smap),visImg,0); cv::imshow("g_sublab",visImg);};    // 属性图

        // 可视化待标注的DEM
        cv::Mat zMap = cv::cvarrToMat(gm.zmap);
        cv::Mat weakGT(gm.smap->height, gm.smap->width, CV_8UC1);
        weakGT.setTo(0);
        cv::cvtColor(zMap, zMap, cv::COLOR_GRAY2BGR);
        DrawNav(zMap, weakGT);
        DrawObs(zMap, weakGT);
        DrawUnlabeled(zMap, weakGT);
        cv::flip(weakGT, weakGT, 0);
        cv::flip(zMap, zMap, 0);
        if (!zMap.empty()) cv::imshow("label_img", zMap);

        // 将图片保存为png格式，用作训练/测试数据
        cv::Mat gtMap(gm.smap->height, gm.smap->width, CV_8UC1);
        if (dFrmNo > 50 && onefrm->dsv[0].millisec > 54289915) {    // 去除train和test重复的路段
            stringstream s_fno;
            s_fno << setw(8) << setfill('0') << onefrm->dsv[0].millisec;
            std::string DATA_PATH = "/home/gaobiao/Documents/RoadSegmentation_IV2019/data/guilin_for_annotation/";

            zMap = cv::cvarrToMat(gm.zmap);
            cv::flip(zMap, zMap, 0);
//            cv::imwrite(DATA_PATH + s_fno.str() + "_img.png", zMap);
            Cvt2Gt(gm.smap, gtMap);
            cv::flip(gtMap, gtMap, 0);
//            cv::imwrite(DATA_PATH + s_fno.str() + "_gt.png", gtMap);
//            cv::imwrite(DATA_PATH + s_fno.str() + "_wgt.png", weakGT);
//            cv::imwrite(DATA_PATH + s_fno.str() + "_video.png", vFrame);
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
            fseeko64(dfp, (LONGLONG)dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
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

    DoProcessingOffline ();

    printf ("Done.\n");

    fclose(dfp);

    return 0;
}
