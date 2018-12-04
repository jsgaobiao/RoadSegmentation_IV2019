#include <define.h>

#define HEIGHT_DELTA 20

extern ONEDSVFRAME	*onefrm;
extern ONEDSVFRAME	*originFrm;

void pointCloudsProject(cv::Mat &img, DMAP &gm)
{
    cv::Mat baseImg = img.clone();
    for (int i = 0; i < BKNUM_PER_FRM; i ++) {
        for (int j = 0; j < LINES_PER_BLK; j ++) {
            for (int k = 0; k < PNTS_PER_LINE; k ++) {
                point3fi *origin_p = &originFrm->dsv[i].points[j*PNTS_PER_LINE+k];
                point3fi *p = &onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
                if (!p->i) continue;
                // 计算投影坐标
                double newX, newY;
                WC2IC_fang(-origin_p->y, origin_p->x, origin_p->z, &newX, &newY);
                newX /= 3.0;
                newY /= 3.0;

                int ix = nint(p->x/PIXSIZ) + WIDSIZ/PIXSIZ/2;
                int iy = nint(p->y/PIXSIZ) + LENSIZ/PIXSIZ/2;

                int step = gm.zmap->widthStep / sizeof(uchar);

                int pHeight = BOUND(p->z * 50 + 100,  1, 255);
                int tHeight = int(gm.zmap->imageData[iy * step + ix]) + HEIGHT_DELTA;
                if (pHeight > tHeight)
                    continue;

                cv::Scalar pColor;
                if (gm.smap->imageData[(iy * step + ix) * 3 + 1] != 0)  // passable
                    pColor = cv::Scalar(0, 255, 0);
                else
                    pColor = cv::Scalar(0, 0, 255);

                if (origin_p->x > 5)   // 保留车辆前方5m以外的激光点，5m内有噪点
                    cv::circle(img, cv::Point(newX, newY), 2, pColor, 1);
            }
        }
    }
    addWeighted(baseImg, 0.6, img, 0.5, 0, img);
}
