#include <define.h>

extern ONEDSVFRAME	*onefrm;
extern ONEDSVFRAME	*originFrm;

void pointCloudsProject(cv::Mat &img, DMAP &gm)
{
    for (int i = 0; i < BKNUM_PER_FRM; i ++) {
        for (int j = 0; j < LINES_PER_BLK; j ++) {
            for (int k = 0; k < PNTS_PER_LINE; k ++) {
                point3fi *origin_p = &originFrm->dsv[i].points[j*PNTS_PER_LINE+k];
                point3fi *p = &onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
                if (!p->i) continue;
                // 计算投影坐标
                double newX, newY;
                WC2IC_fang(origin_p->x, origin_p->y, origin_p->z, &newX, &newY);

                int ix = nint(p->x/PIXSIZ) + WIDSIZ/PIXSIZ/2;
                int iy = nint(p->y/PIXSIZ) + LENSIZ/PIXSIZ/2;
                int step = gm.zmap->widthStep / sizeof(uchar);
                cv::Scalar pColor;
                if (gm.smap->imageData[(iy * step + ix) * 3 + 1] != 0)  // passable
                    pColor = cv::Scalar(0, 255, 0);
                else
                    pColor = cv::Scalar(0, 0, 255);

                if (p->y > 0)   // 车辆前方的激光点
                    cv::circle(img, cv::Point(newX, newY), 3, pColor, 1);
            }
        }
    }
}
