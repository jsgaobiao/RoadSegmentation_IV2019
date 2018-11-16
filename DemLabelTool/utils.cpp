#include "utils.h"

// transform the previous human annotation into the coordination of the newest position
void TransAnnotation(cv::Mat &srcImg)
{
    // initialization
    cv::Mat tmpImg = srcImg.clone();
    srcImg.setTo(255);

    // estimation for transformation
    MAT2D	rot1, rot2;

    // rot1: R_tmpImg^{-1}*R_srcImg, srctrans:srcImg, tartrans:tmpImg
    double ang = (-preFrameDsv.ang.z + onefrm->dsv[0].ang.z);
    rot1[0][0] = cos (ang);
    rot1[0][1] = -sin (ang);
    rot1[1][0] = sin (ang);
    rot1[1][1] = cos (ang);

    //rot2: R_gmtar^{-1}
    ang = -onefrm->dsv[0].ang.z;
    rot2[0][0] = cos (ang);
    rot2[0][1] = -sin (ang);
    rot2[1][0] = sin (ang);
    rot2[1][1] = cos (ang);


    // shv: SHV_srcImg-SHV_tmpImg
    point2d shv, tmpShv;
    tmpShv.x = (preFrameDsv.shv.x-onefrm->dsv[0].shv.x);
    tmpShv.y = (preFrameDsv.shv.y-onefrm->dsv[0].shv.y);
    shv.x = (-tmpShv.x) / PIXSIZ;
    shv.y = (-tmpShv.y) / PIXSIZ;
    rotatePoint2d (shv, rot2);	//R_t^{-1}*(SHV_{t-1}-SHV_{t})

    // transform from the frame of srcImg to tmpImg
    for (int y = 0; y < tmpImg.rows; y ++) {
        for (int x = 0; x < tmpImg.cols; x ++) {
            point2d p;
            p.x = (x - tmpImg.cols / 2.0);
            p.y = (y - tmpImg.rows / 2.0);
            rotatePoint2d (p, rot1);	//R_t^{-1}*R_{t-1}*p
            shiftPoint2d (p, shv);		//p'=R_t^{-1}*R_{t-1}*p+R_t^{-1}*(SHV_{t-1}-SHV_{t})
            p.x = round(p.x + srcImg.cols / 2.0);
            p.y = round(p.y + srcImg.rows / 2.0);
            if (p.x >= 0 && p.y >= 0 && p.x < srcImg.cols && p.y < srcImg.rows)
                srcImg.at<uchar>(p.y, p.x) = tmpImg.at<uchar>(y, x);
        }
    }
}

bool ColorEqual(cv::Vec3b a, cv::Scalar b)
{
    return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
}

void pointCloudsProject(cv::Mat &img, cv::Mat _gtImg, cv::Mat zmap)
{
    double ratio = img.rows / 1086.0;
    cv::Mat baseImg = img.clone();
    for (int i = 0; i < BKNUM_PER_FRM; i ++) {
        for (int j = 0; j < LINES_PER_BLK; j ++) {
            for (int k = 0; k < PNTS_PER_LINE; k ++) {
                point3fi *p = &onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
                if (!p->i) continue;

                // 按照标定参数校准
                point3fi tp = CorrectPoints (*p, onefrm->dsv[i]);
                int ix = nint(tp.x/PIXSIZ) + WIDSIZ/PIXSIZ/2;
                int iy = nint(tp.y/PIXSIZ) + LENSIZ/PIXSIZ/2;
                iy = _gtImg.rows - iy;
                if (ix < 0 || iy < 0 || ix >= _gtImg.cols || iy >= _gtImg.rows)
                    continue;

                // 投影后的高度
                int pHeight = BOUND(tp.z * 50 + 100,  1, 255);
                // 只保留输入DEM高度下方的激光点进行投影
                if (pHeight > zmap.at<cv::Vec3b>(iy, ix)[0] + HEIGHT_DELTA)
                    continue;

                // 计算投影坐标
                double newX, newY;
                WC2IC_fang(-p->y, p->x, p->z, &newX, &newY);
                newX *= ratio;
                newY *= ratio;

                if (ColorEqual(_gtImg.at<cv::Vec3b>(iy, ix), colorTable[0]))  // unknown
                    continue;
                cv::Scalar c(_gtImg.at<cv::Vec3b>(iy, ix)[0], _gtImg.at<cv::Vec3b>(iy, ix)[1], _gtImg.at<cv::Vec3b>(iy, ix)[2]);
                if (p->x > 6 && p->x < 50)   // 保留车辆前方6-50m的激光点，排除噪点
                    cv::circle(img, cv::Point(newX, newY), 2, c, 1);
            }
        }
    }
    addWeighted(baseImg, 0.6, img, 0.6, 0, img);
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

point3fi CorrectPoints (point3fi p, ONEDSVDATA dsv)
{
    point3fi pt = p;
    MAT2D	rot1, rot2;
    //transform points to the vehicle frame of onefrm->dsv[0]
    //src: block i; tar: block 0

    //rot2: R_tar^{-1}
    rot2[0][0] = cos (-onefrm->dsv[0].ang.z);
    rot2[0][1] = -sin (-onefrm->dsv[0].ang.z);
    rot2[1][0] = sin (-onefrm->dsv[0].ang.z);
    rot2[1][1] = cos (-onefrm->dsv[0].ang.z);

    rotatePoint3fi(pt, calibInfo.rot);
    shiftPoint3fi(pt, calibInfo.shv);
    rotatePoint3fi(pt, dsv.rot);

    //rot1: R_tar^{-1}*R_src
    rot1[0][0] = cos (dsv.ang.z-onefrm->dsv[0].ang.z);
    rot1[0][1] = -sin (dsv.ang.z-onefrm->dsv[0].ang.z);
    rot1[1][0] = sin (dsv.ang.z-onefrm->dsv[0].ang.z);
    rot1[1][1] = cos (dsv.ang.z-onefrm->dsv[0].ang.z);

    //shv: SHV_src-SHV_tar
    point2d shv;
    shv.x = dsv.shv.x-onefrm->dsv[0].shv.x;
    shv.y = dsv.shv.y-onefrm->dsv[0].shv.y;

    point2d pp;
    pp.x = pt.x; pp.y = pt.y;
    rotatePoint2d (pp, rot1);	//R_tar^{-1}*R_src*p
    rotatePoint2d (shv, rot2);	//R_tar^{-1}*(SHV_src-SHV_tar)
    shiftPoint2d (pp, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)

    pt.x = pp.x; pt.y = pp.y;
    return pt;
}

//读取一帧vel64数据（一帧为580×12×32个激光点）保存到onefrm->dsv，未作坐标转换
bool ReadOneDsvFrame ()
{
    DWORD	dwReadBytes;
    int		i;
    for (i=0; i<BKNUM_PER_FRM; i++) {
        dwReadBytes = fread((ONEDSVDATA *)&onefrm->dsv[i], 1, dsbytesiz, dfp);
        if ((dsbytesiz != dwReadBytes) || (ferror(dfp))) {
            printf("Error from reading file.\n");
            break;
        }
        createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , 0 );

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
    else {
        return true;
    }
}

bool LoadCalibFile (const char *szFile)
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

void Gt2BGR(cv::Mat &img)
{
    for (int i = 0; i < img.rows; i ++) {
        for (int j = 0; j < img.cols; j ++) {
            int g = img.at<cv::Vec3b>(i, j)[0];
            img.at<cv::Vec3b>(i, j)[0] = colorTable[g][0];
            img.at<cv::Vec3b>(i, j)[1] = colorTable[g][1];
            img.at<cv::Vec3b>(i, j)[2] = colorTable[g][2];
        }
    }
}

void BGR2Gt(cv::Mat &img, cv::Mat gt)
{
    for (int i = 0; i < img.rows; i ++) {
        for (int j = 0; j < img.cols; j ++) {

            for (int k = 0; k < colorTable.size(); k ++) {
                if (ColorEqual(img.at<cv::Vec3b>(i, j), colorTable[k])) {
                    gt.at<uchar>(i, j) = k;
                    break;
                }
            }

        }
    }
}
