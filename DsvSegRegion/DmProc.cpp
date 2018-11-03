#include "define.h"

extern RMAP	rm;
extern TRANSINFO calibInfo;
extern ONEDSVFRAME	*onefrm;
int cntFlag = 0;

void  DrawDem (DMAP &m)
{
    // 计算高程图
    if (!m.zmap)
        m.zmap = cvCreateImage(cvSize(m.wid,m.len), IPL_DEPTH_8U, 1);
    // 0表示未知高度
    cvZero(m.zmap);
    int step = m.zmap->widthStep / sizeof(uchar);

    int x, y;
    for (y=0; y<m.len; y++) {
        for (x=0; x<m.wid; x++) {
            double pr = m.lpr[y*m.wid+x];
            // 如果是路面区域，取路面高度
            if (m.demgnum[y*m.wid+x]) {
                // 映射前的高度范围(-2.0m, 3.1m)
                m.zmap->imageData[y*step+x] = BOUND((m.demg[y*m.wid+x]) * 50 + 100,  1, 255);
            }
            // 如果是非路面区域,取最大高度（test）,映射前的高度范围(-2.0m, 3.1m)
            else if (m.demhnum[y*m.wid+x]) {
                m.zmap->imageData[y*step+x] = BOUND((m.demhmax[y*m.wid+x]) * 50 + 100,  1, 255);
            }
        }
    }

    // 可视化lab 可通行区域
    if (m.lab) {
        // lmap:标记可通行区域和不可通行区域
        if (!m.lmap) {
            m.lmap = cvCreateImage(cvSize(m.wid,m.len), IPL_DEPTH_8U, 3);
            printf("new lmap\n");
        }
        cvZero(m.lmap);
        if (!m.pmap)
            m.pmap = cvCreateImage(cvSize(m.wid,m.len), IPL_DEPTH_8U, 1);
        cvZero(m.pmap);

        // 可视化lab 可通行区域 和 不可通行区域
        int x, y;
        for (y=0; y<m.len; y++) {
            for (x=0; x<m.wid; x++) {
                if (m.lab[y*m.wid+x]==UNKNOWN)
                    continue;
                double pr = m.lpr[y*m.wid+x];
                //可通行区域图
                if (m.lab[y*m.wid+x]==TRAVESABLE) {
                    m.lmap->imageData[(y*step+x)*3] = 0;
                    m.lmap->imageData[(y*step+x)*3+1] =55+200*pr;
                    m.lmap->imageData[(y*step+x)*3+2] = 0;
                    m.pmap->imageData[y*step+x] = std::min(m.lpr[y*m.wid+x], 1.0) * 127 + 128;
                }
                //不可通行区域
                else {
                    // 如果 有道路中心线（累计图）
                    if (m.centerln) {
                        if (m.demhmax[y*m.wid+x] > m.centerln[y].h + POSOBSMINHEIGHT) {
                            // 正障碍，红色
                            m.lmap->imageData[(y*step+x)*3] = 0;
                            m.lmap->imageData[(y*step+x)*3+1] =0;
                            m.lmap->imageData[(y*step+x)*3+2] = 55+200*pr;
                            m.pmap->imageData[y*step+x] = - min(m.lpr[y*m.wid+x], 1.0) * 127 + 128;
                        }
                        else
                        if (m.demhmin[y*m.wid+x] < m.centerln[y].h - POSOBSMINHEIGHT) {
                            //负障碍，蓝色
                            m.lmap->imageData[(y*step+x)*3] = 55+200*pr;
                            m.lmap->imageData[(y*step+x)*3+1] =0;
                            m.lmap->imageData[(y*step+x)*3+2] = 0;
                            m.pmap->imageData[y*step+x] = - min(m.lpr[y*m.wid+x], 1.0) * 127 + 128;
                        }
                        else {
                            //与道路同高度，按TRAVESABLE可视化
                            m.lmap->imageData[(y*step+x)*3] = 0;
                            m.lmap->imageData[(y*step+x)*3+1] = 55+200*pr;
                            m.lmap->imageData[(y*step+x)*3+2] = 0;
                            m.pmap->imageData[y*step+x] = min(m.lpr[y*m.wid+x], 1.0) * 127 + 128;
                        }
                    }
                    // 如果 没有道路中心线（单帧图）
                    else {
                        //不可通行NONTRAVESABLE，粉色
                        m.lmap->imageData[(y*step+x)*3] = 55+200*pr;
                        m.lmap->imageData[(y*step+x)*3+1] =0;
                        m.lmap->imageData[(y*step+x)*3+2] = 55+200*pr;
                        m.pmap->imageData[y*step+x] = - min(m.lpr[y*m.wid+x], 1.0) * 127 + 128;
                    }
                }
            }
        }
    }

    // 可视化属性图sublab 凹凸障碍，上下坡等
    if (m.sublab) {
        if (!m.smap)
            m.smap = cvCreateImage(cvSize(m.wid,m.len), IPL_DEPTH_8U, 3);
        memset (m.smap->imageData, 0, m.len*m.wid*3);
//		cvZero(m.smap);

        int x, y;
        for (y=0; y<m.len; y++) {
            for (x=0; x<m.wid; x++) {
                // 不可通行区域的障碍标签
                if (m.sublab[y*m.wid+x]==UNKNOWN) {
                    if (m.lab[y*m.wid+x]==UNKNOWN)
                        continue;
                    // 如果 有道路中心线（累计图）
                    if (m.centerln) {
                        if (m.demhmax[y*m.wid+x] > m.centerln[y].h + POSOBSMINHEIGHT) {
                            // 正障碍，红色
                            m.smap->imageData[(y*step+x)*3] = 0;
                            m.smap->imageData[(y*step+x)*3+1] =0;
                            m.smap->imageData[(y*step+x)*3+2] = 255;
                        }
                        else
                        if (m.demhmin[y*m.wid+x] < m.centerln[y].h - POSOBSMINHEIGHT) {
                            //负障碍，蓝色
                            m.smap->imageData[(y*step+x)*3] = 255;
                            m.smap->imageData[(y*step+x)*3+1] =0;
                            m.smap->imageData[(y*step+x)*3+2] = 0;
                        }
                    }
                }
                // 可通行区域的标签
                if (m.sublab[y*m.wid+x]==FLATGROUND || m.lab[y*m.wid+x]==TRAVESABLE) {
                    //平面 绿色
                    m.smap->imageData[(y*step+x)*3] = 0;
                    m.smap->imageData[(y*step+x)*3+1] =255;
                    m.smap->imageData[(y*step+x)*3+2] = 0;
                }
                // else
                if (m.sublab[y*m.wid+x]==DOWNSLOPE) {
                    //下坡(方块化，需要调试)
//                    m.smap->imageData[(y*step+x)*3] = 128; //BOUND(nint(fabs(m.groll[y*m.wid+x])*255),0,255);
//                    m.smap->imageData[(y*step+x)*3+1] = 128;
//                    m.smap->imageData[(y*step+x)*3+2] = 0;
                }
                else if (m.sublab[y*m.wid+x]==UPSLOPE) {
                    //上坡(方块化，需要调试)
//                    m.smap->imageData[(y*step+x)*3] = 64; //BOUND(nint(fabs(m.groll[y*m.wid+x])*255),0,255);
//                    m.smap->imageData[(y*step+x)*3+1] = 128;
//                    m.smap->imageData[(y*step+x)*3+2] = 255;
                }
                else if (m.sublab[y*m.wid+x]==LEFTSIDESLOPE) {
                    //左斜坡
//                    m.smap->imageData[(y*step+x)*3] = 128;
//                    m.smap->imageData[(y*step+x)*3+1] = 128;
//                    m.smap->imageData[(y*step+x)*3+2] = 0; //BOUND(nint(fabs(m.gpitch[y*m.wid+x])*255),0,255);
                }
                else if (m.sublab[y*m.wid+x]==RIGHTSIDESLOPE) {
                    //右斜坡
//                    m.smap->imageData[(y*step+x)*3] = 128;
//                    m.smap->imageData[(y*step+x)*3+1] =128;
//                    m.smap->imageData[(y*step+x)*3+2] = 0; //BOUND(nint(fabs(m.gpitch[y*m.wid+x])*255),0,255);;
                }
                else if (m.sublab[y*m.wid+x]==NEGATOBSTA) {
                    //负障碍 蓝色
//                    m.smap->imageData[(y*step+x)*3] = 255;
//                    m.smap->imageData[(y*step+x)*3+1] =0;
//                    m.smap->imageData[(y*step+x)*3+2] = 0;
                }
                else if (m.sublab[y*m.wid+x]==POSSIOBSTA) {
                    //正障碍 红色
//                    m.smap->imageData[(y*step+x)*3] = 0;
//                    m.smap->imageData[(y*step+x)*3+1] =0;
//                    m.smap->imageData[(y*step+x)*3+2] = 255;
                }
            }
        }
    }
/*
    if (m.centerln) {
        int x, y;
        for (y=0; y<m.len; y++) {
            x=int((m.centerln[y].x0+m.centerln[y].x1)/2.0);
            m.lmap->imageData[(y*step+x)*3] = 255;
            m.lmap->imageData[(y*step+x)*3+1] =255;
            m.lmap->imageData[(y*step+x)*3+2] = 255;
            if ((x+1)>=m.wid) continue;
            m.lmap->imageData[(y*step+x+1)*3] = 255;
            m.lmap->imageData[(y*step+x+1)*3+1] =255;
            m.lmap->imageData[(y*step+x+1)*3+2] = 255;
        }
    }
    */
}

void CopyGloDem (DMAP *tar, DMAP *src)
{
    if (!src->dataon || src->wid!=tar->wid || src->len!=tar->len)
        return;

    if (!tar->demg) tar->demg = new double[tar->wid*tar->len];
    memcpy (tar->demg, src->demg, sizeof (double)*src->wid*src->len);

    if (!tar->demhmin) tar->demhmin = new double[tar->wid*tar->len];
    memcpy (tar->demhmin, src->demhmin, sizeof (double)*src->wid*src->len);

    if (!tar->demhmax) tar->demhmax = new double[tar->wid*tar->len];
    memcpy (tar->demhmax, src->demhmax, sizeof (double)*src->wid*src->len);

    if (!tar->demgnum) tar->demgnum = new int[tar->wid*tar->len];
    memcpy (tar->demgnum, src->demgnum, sizeof (int)*src->wid*src->len);

    if (!tar->demhnum) tar->demhnum = new int[tar->wid*tar->len];
    memcpy (tar->demhnum, src->demhnum, sizeof (int)*src->wid*src->len);

    if (!tar->lab) tar->lab = new BYTE[tar->wid*tar->len];
    memcpy (tar->lab, src->lab, sizeof (BYTE)*src->wid*src->len);

    if (!tar->lpr) tar->lpr = new double[tar->wid*tar->len];
    memcpy (tar->lpr, src->lpr, sizeof (double)*src->wid*src->len);

    tar->trans = src->trans;
    tar->dataon = true;
}

void ZeroGloDem (DMAP *m)
{
    if (!m->dataon)
        return;

    memset (m->demg, 0, sizeof (double)*m->wid*m->len);
    memset (m->demhmin, 0, sizeof (double)*m->wid*m->len);
    memset (m->demhmax, 0, sizeof (double)*m->wid*m->len);
    memset (m->demgnum, 0, sizeof (int)*m->wid*m->len);
    memset (m->demhnum, 0, sizeof (int)*m->wid*m->len);
    memset (m->lab, 0, sizeof (BYTE)*m->wid*m->len);
    memset (m->lpr, 0, sizeof (double)*m->wid*m->len);
}

void PredictGloDem (DMAP &gmtar, DMAP &gmtmp)
{
    if (!gmtar.dataon)
        return;

    //gmtar is the data of the previous frame
    //copy gmtar to gmtmp
    CopyGloDem (&gmtmp, &gmtar);
    //clear gmtar
    ZeroGloDem (&gmtar);

    //update the pose of gmtar to the current frame
    gmtar.trans.ang = onefrm->dsv[0].ang.z;
    gmtar.trans.shv.x = onefrm->dsv[0].shv.x;
    gmtar.trans.shv.y = onefrm->dsv[0].shv.y;

    //estimation for transformation
    MAT2D	rot1, rot2;

    //rot1: R_gmtar^{-1}*R_gmtmp, srctrans:gmtmp, tartrans:gmtar
    rot1[0][0] = cos (gmtmp.trans.ang - gmtar.trans.ang);
    rot1[0][1] = -sin (gmtmp.trans.ang - gmtar.trans.ang);
    rot1[1][0] = sin (gmtmp.trans.ang - gmtar.trans.ang);
    rot1[1][1] = cos (gmtmp.trans.ang - gmtar.trans.ang);

    //rot2: R_gmtar^{-1}
    rot2[0][0] = cos (-gmtar.trans.ang);
    rot2[0][1] = -sin (-gmtar.trans.ang);
    rot2[1][0] = sin (-gmtar.trans.ang);
    rot2[1][1] = cos (-gmtar.trans.ang);

    //shv: SHV_gmtmp-SHV_gmtar
    point2d shv;
    shv.x = gmtmp.trans.shv.x-gmtar.trans.shv.x;
    shv.y = gmtmp.trans.shv.y-gmtar.trans.shv.y;
    rotatePoint2d (shv, rot2);	//R_t^{-1}*(SHV_{t-1}-SHV_{t})

    //transform from the frame of gmtmp to gmtar
    int x, y;
    for (y=0; y<gmtmp.len; y++) {
        for (x=0; x<gmtmp.wid; x++) {
            if (!gmtmp.lab[y*gmtmp.wid+x])
                continue;
            if (gmtmp.lpr[y*gmtmp.wid+x]<0.2)   // probability of the lab
                continue;

            point2d p;
            p.x = (x-gmtmp.wid/2)*PIXSIZ;
            p.y = (y-gmtmp.len/2)*PIXSIZ;
            rotatePoint2d (p, rot1);	//R_t^{-1}*R_{t-1}*p
            shiftPoint2d (p, shv);		//p'=R_t^{-1}*R_{t-1}*p+R_t^{-1}*(SHV_{t-1}-SHV_{t})

            p.x = p.x/PIXSIZ+gmtar.wid/2;
            p.y = p.y/PIXSIZ+gmtar.len/2;
            int x0, y0, x1, y1;
            x0 = round(p.x)-0; y0 = round(p.y)-0;
            x1 = round(p.x)+0; y1 = round(p.y)+0;
            for (int yy=y0; yy<=y1; yy++) {
                if (yy<0 || yy>=gmtar.len) continue;
                for (int xx=x0; xx<=x1; xx++) {
                    if (xx<0 || xx>=gmtar.wid) continue;
                    double dd=sqrt(double((xx-p.x)*(xx-p.x)+(yy-p.y)*(yy-p.y)));
                    double fac=0.89;//(1.0-dd/1.415)*0.9;
                    int dn;
                    double lpr = gmtmp.lpr[y*gmtmp.wid+x]*fac;
                    if (lpr<0.2) continue;

                    //lab还没有设置
                    if (!gmtar.lab[yy*gmtar.wid+xx]) {
                        // Ground
                        if (gmtmp.demgnum[y*gmtmp.wid+x]) {
                            gmtar.demg[yy*gmtar.wid+xx]=gmtmp.demg[y*gmtmp.wid+x];
                            gmtar.demgnum[yy*gmtar.wid+xx]=gmtmp.demgnum[y*gmtmp.wid+x];
                        }
                        // non-ground
                        if (gmtmp.demhnum[y*gmtmp.wid+x]) {
                            gmtar.demhmin[yy*gmtar.wid+xx]=gmtmp.demhmin[y*gmtmp.wid+x];
                            gmtar.demhmax[yy*gmtar.wid+xx]=gmtmp.demhmax[y*gmtmp.wid+x];
                            gmtar.demhnum[yy*gmtar.wid+xx]=gmtmp.demhnum[y*gmtmp.wid+x];
                        }
                        gmtar.lab[yy*gmtar.wid+xx]=gmtmp.lab[y*gmtmp.wid+x];
                        gmtar.lpr[yy*gmtar.wid+xx]=gmtmp.lpr[y*gmtmp.wid+x]*fac;
                    }
                    else if (gmtar.lpr[yy*gmtar.wid+xx]<(gmtmp.lpr[y*gmtmp.wid+x]*fac)) {
                        //取概率大的赋值
                        if (gmtmp.demgnum[y*gmtmp.wid+x]) {
                            gmtar.demg[yy*gmtar.wid+xx]=gmtmp.demg[y*gmtmp.wid+x];
                            gmtar.demgnum[yy*gmtar.wid+xx]=gmtmp.demgnum[y*gmtmp.wid+x];
                        }
                        if (gmtmp.demhnum[y*gmtmp.wid+x]) {
                            gmtar.demhmin[yy*gmtar.wid+xx]=gmtmp.demhmin[y*gmtmp.wid+x];
                            gmtar.demhmax[yy*gmtar.wid+xx]=gmtmp.demhmax[y*gmtmp.wid+x];
                            gmtar.demhnum[yy*gmtar.wid+xx]=gmtmp.demhnum[y*gmtmp.wid+x];
                        }

                        if (gmtar.lab[yy*gmtar.wid+xx]==gmtmp.lab[y*gmtmp.wid+x]) {
                            //如果两个lab相同，概率增加1.2为经验系数
//							gmtar.lpr[yy*gmtar.wid+xx]=min(1.0,gmtar.lpr[yy*gmtar.wid+xx]+gmtmp.lpr[y*gmtmp.wid+x]*fac);
                            gmtar.lpr[yy*gmtar.wid+xx]=gmtmp.lpr[y*gmtmp.wid+x]*fac*1.2;
                        }
                        else {
                            //如果两个lab不相同，概率降低0.8，为经验系数
                            gmtar.lab[yy*gmtar.wid+xx]=gmtmp.lab[y*gmtmp.wid+x];
                            gmtar.lpr[yy*gmtar.wid+xx]=gmtmp.lpr[y*gmtmp.wid+x]*fac*0.8;
                        }
                    }
                }
            }
        }
    }
}


void UpdateGloDem (DMAP &glo, DMAP &loc)
{
    if (!glo.dataon) {
        CopyGloDem (&glo, &loc);
        glo.dataon = true;
        return;
    }
    double fac;
    int dx, dy;
    for (dy=0; dy<loc.len; dy++) {
        for (dx=0; dx<loc.wid; dx++) {
            double dis2Vehicle = sqrt(sqr(dx-loc.wid/2)+sqr(dy-loc.len/2));
            if (loc.lab[dy*loc.wid+dx] == UNKNOWN &&  dis2Vehicle > 60.0 / PIXSIZ)
                continue;
            int gx, gy;
            gx = dx-loc.wid/2+glo.wid/2;
            gy = dy-loc.len/2+glo.len/2;
            // 空的位置直接赋值
            if (!glo.lab[gy*glo.wid+gx]) {
                glo.lab[gy*glo.wid+gx] = loc.lab[dy*loc.wid+dx];
                glo.lpr[gy*glo.wid+gx] = loc.lpr[dy*loc.wid+dx];
            }
            // 相同标签，加强
            else if (loc.lab[dy*loc.wid+dx] == glo.lab[gy*glo.wid+gx]) {
                if (loc.lab[dy*loc.wid+dx] == UNKNOWN)
                    break;
                fac = loc.lpr[dy*loc.wid+dx] * 2.0;
                glo.lpr[gy*glo.wid+gx] = min (1.0, glo.lpr[gy*glo.wid+gx]*fac);
            }
            // 不同标签，竞争
            else { //if (loc.lab[dy*loc.wid+dx] != glo.lab[gy*glo.wid+gx])
                // 车周围黑圈处理
                if (loc.lab[dy*loc.wid+dx] == UNKNOWN) {
                    double _fac = 1.8;
                    // 超过车辆盲区的未知区域，使用大于1的衰减速率（减少衰减）
                    if (dis2Vehicle > 10.0 / PIXSIZ)
                        _fac = 1.1;
                    glo.lpr[gy*glo.wid+gx] = min (1.0, glo.lpr[gy*glo.wid+gx] * _fac);
                }
                else {
                    fac = loc.lpr[dy*loc.wid+dx] / 2.0;
                    // (暂时不加)可通行区域优先覆盖障碍物格点
                    //if (loc.lab[dy*loc.wid+dx] == TRAVESABLE) {
                    //    fac *= 2.0;
                    //}
                    glo.lpr[gy*glo.wid+gx] = min (1.0, glo.lpr[gy*glo.wid+gx]*fac);
                    if (glo.lpr[gy*glo.wid+gx] < 0.2) {
                        glo.lab[gy*glo.wid+gx] = loc.lab[dy*loc.wid+dx];
                        glo.lpr[gy*glo.wid+gx] = loc.lpr[dy*loc.wid+dx];
                    }
                }
            }

            // 维护地面点
            if (glo.demgnum[gy*glo.wid+gx] && loc.demgnum[dy*loc.wid+dx]) {
                glo.demg[gy*glo.wid+gx] = (glo.demg[gy*glo.wid+gx]*glo.demgnum[gy*glo.wid+gx]+
                                           loc.demg[dy*loc.wid+dx]*loc.demgnum[dy*loc.wid+dx])/
                                          (double)(glo.demgnum[gy*glo.wid+gx]+loc.demgnum[dy*loc.wid+dx]);
                //9999的上限，防止长时间车辆静止时溢出
                glo.demgnum[gy*glo.wid+gx]= min(9999, glo.demgnum[gy*glo.wid+gx]+loc.demgnum[dy*loc.wid+dx]);
            }
            else if (loc.demgnum[dy*loc.wid+dx]) {
                glo.demg[gy*glo.wid+gx] = loc.demg[dy*loc.wid+dx];
                glo.demgnum[gy*glo.wid+gx]= loc.demgnum[dy*loc.wid+dx];
            }
            // 维护障碍点
            if (glo.demhnum[gy*glo.wid+gx]&&loc.demhnum[dy*loc.wid+dx]) {
                glo.demhmin[gy*glo.wid+gx] = min(glo.demhmin[gy*glo.wid+gx], loc.demhmin[dy*loc.wid+dx]);
                glo.demhmax[gy*glo.wid+gx] = max(glo.demhmax[gy*glo.wid+gx], loc.demhmax[dy*loc.wid+dx]);
                glo.demhnum[gy*glo.wid+gx]= min(9999, glo.demhnum[gy*glo.wid+gx]+loc.demhnum[dy*loc.wid+dx]);
            }
            else if (loc.demhnum[dy*loc.wid+dx]) {
                glo.demhmin[gy*glo.wid+gx] = loc.demhmin[dy*loc.wid+dx];
                glo.demhmax[gy*glo.wid+gx] = loc.demhmax[dy*loc.wid+dx];
                glo.demhnum[gy*glo.wid+gx]= loc.demhnum[dy*loc.wid+dx];
            }
        }
    }
}

void GenerateLocDem (DMAP &loc)
{
    int x, y;

    if (!loc.demg) loc.demg = new double[loc.wid*loc.len];
    memset (loc.demg, 0, sizeof (double)*loc.wid*loc.len);
    if (!loc.demhmin) loc.demhmin = new double[loc.wid*loc.len];
    memset (loc.demhmin, 0, sizeof (double)*loc.wid*loc.len);
    if (!loc.demhmax) loc.demhmax = new double[loc.wid*loc.len];
    memset (loc.demhmax, 0, sizeof (double)*loc.wid*loc.len);
    if (!loc.demgnum) loc.demgnum = new int[loc.wid*loc.len];
    memset (loc.demgnum, 0, sizeof (int)*loc.wid*loc.len);
    if (!loc.demhnum) loc.demhnum = new int[loc.wid*loc.len];
    memset (loc.demhnum, 0, sizeof (int)*loc.wid*loc.len);
    if (!loc.lab) loc.lab = new BYTE[loc.wid*loc.len];
    memset (loc.lab, 0, sizeof(BYTE)*loc.wid*loc.len);
    if (!loc.lpr) loc.lpr = new double[loc.wid*loc.len];
    memset (loc.lpr, 0, sizeof(double)*loc.wid*loc.len);

    for (int ry=0; ry<rm.len; ry++) {
        for (int rx=0; rx<rm.wid; rx++) {
            if (!rm.pts[ry*rm.wid+rx].i)
                continue;
            point3fi *p = &rm.pts[ry*rm.wid+rx];
            // 处理路面区域
            bool isroad;
            if (rm.regionID[ry*rm.wid+rx]<=0 || rm.regionID[ry*rm.wid+rx]>rm.regnum)
                isroad=false;
            else {
                // 区域太小认为不是路面
                SEGBUF *segbuf = &rm.segbuf[rm.regionID[ry*rm.wid+rx]];
                if (segbuf->ptnum)
                    isroad=true;
                else
                    isroad=false;
            }
            float ix, iy;
            ix = nint(p->x/PIXSIZ)+loc.wid/2;
            iy = nint(p->y/PIXSIZ)+loc.len/2;
            // 计算路面平均高度
            int x0, y0, x1, y1;
            x0 = int(ix); y0 = int(iy);
            x1 = int(ix)+1; y1 = int(iy)+1;
            for (y=y0; y<=y1; y++) {
                if (y<0 || y>=loc.len) continue;
                for (x=x0; x<=x1; x++) {
                    if (x<0 || x>=loc.wid) continue;
                    if (isroad) {
                        loc.demg[y*loc.wid+x] += p->z;
                        loc.demgnum[y*loc.wid+x] ++;
                    }
                    else {
                        if (!loc.demhnum[y*loc.wid+x]) {
                            loc.demhmin[y*loc.wid+x] = loc.demhmax[y*loc.wid+x] = p->z;
                        }
                        // 非路面区域维护z最大最小值
                        else {
                            loc.demhmin[y*loc.wid+x] = min (loc.demhmin[y*loc.wid+x],(double)p->z);
                            loc.demhmax[y*loc.wid+x] = max (loc.demhmax[y*loc.wid+x],(double)p->z);
                        }
                        loc.demhnum[y*loc.wid+x] ++;
                    }
                }
            }
        }
    }
    for (y=0; y<loc.len; y++) {
        for (x=0; x<loc.wid; x++) {
            // 没有路面点的区域设为INVALIDDOUBLE
            if (!loc.demgnum[y*loc.wid+x])
                loc.demg[y*loc.wid+x] = INVALIDDOUBLE;
            else
                loc.demg[y*loc.wid+x] /= (double)loc.demgnum[y*loc.wid+x];
            // 无非路面点的区域
            if (!loc.demhnum[y*loc.wid+x])
                loc.demhmin[y*loc.wid+x] = loc.demhmax[y*loc.wid+x] = INVALIDDOUBLE;
        }
    }


    for (y=0; y<loc.len; y++) {
        for (x=0; x<loc.wid; x++) {
            // 无观测，跳过
            if (!loc.demgnum[y*loc.wid+x] && !loc.demhnum[y*loc.wid+x])
                continue;
            else if (loc.demgnum[y*loc.wid+x] && !loc.demhnum[y*loc.wid+x]) {
                //可通行区域（有地面，无障碍）
                loc.lab[y*loc.wid+x] = TRAVESABLE;
            }
            else if (!loc.demgnum[y*loc.wid+x] && loc.demhnum[y*loc.wid+x]) {
                // 无地面点，有障碍区域，需要判断是不是小坑或者小草
                // gz记录邻域内的地面高度
                double gz=INVALIDDOUBLE;
                for (int yy=y-2; yy<=y+2; yy++) {
                    if (yy<0) continue;
                    if (yy>=loc.len) break;
                    for (int xx=x-2; xx<=x+2; xx++) {
                        if (xx<0) continue;
                        if (xx>=loc.wid) break;
                        if (loc.demgnum[yy*loc.wid+xx]) {
                            gz = loc.demg[yy*loc.wid+xx];
                            break;
                        }
                    }
                    if (gz!=INVALIDDOUBLE) break;
                }
                if (loc.demhmin[y*loc.wid+x]>=gz-POSOBSMINHEIGHT && loc.demhmax[y*loc.wid+x]<=gz+POSOBSMINHEIGHT) {
                    //可通行区域：路上的小坑/小草
                    loc.lab[y*loc.wid+x] = TRAVESABLE;
                }
                else {
                    //否则，不可通行区域
                    loc.lab[y*loc.wid+x] = NONTRAVESABLE;
                }
            }
            else if (loc.demgnum[y*loc.wid+x] && loc.demhnum[y*loc.wid+x]) {
                // 既有地面点，又有地上的点
                double dd = loc.demhmin[y*loc.wid+x]-loc.demg[y*loc.wid+x];
                if (dd > VEHICLEHEIGHT)	{			//larger than vehicle height
                    //可通行，悬浮物
                    loc.lab[y*loc.wid+x] = TRAVESABLE;
                }
                else {
                    dd = loc.demhmax[y*loc.wid+x]-loc.demg[y*loc.wid+x];
                    // 悬浮物最高处与地面的高度差
                    if (dd < POSOBSMINHEIGHT) {
                        //可通行区域：横在中间的树枝
                        loc.lab[y*loc.wid+x] = TRAVESABLE;
                    }
                    else {
                        //不可通行区域
                        loc.lab[y*loc.wid+x] = NONTRAVESABLE;
                    }
                }
            }
        }
    }

    // 计算每个栅格的概率
    for (y=0; y<loc.len; y++) {
        for (x=0; x<loc.wid; x++) {
            //lab还没有设置
            if (loc.lab[y*loc.wid+x]==UNKNOWN)
                continue;
            int tcnt=0;
            int cnt=0;
            for (int yy=y-1; yy<=y+1; yy++) {
                if (yy<0||yy>=loc.len) continue;
                for (int xx=x-1; xx<=x+1; xx++) {
                    if (xx<0||xx>=loc.wid) continue;
                    tcnt ++;
                    if (loc.lab[y*loc.wid+x]==loc.lab[yy*loc.wid+xx])
                        cnt ++;
                }
            }
            // 寻找和邻域不一致的标签，并去除
            if (cnt<2) {
                loc.lab[y*loc.wid+x]=UNKNOWN;   //remove irregular points
                continue;
            }
            else {
                loc.lpr[y*loc.wid+x] = (double)cnt/(double)tcnt*0.5+0.5;
            }
        }
    }
    loc.trans.ang = onefrm->dsv[0].ang.z;
    loc.trans.shv.x = onefrm->dsv[0].shv.x;
    loc.trans.shv.y = onefrm->dsv[0].shv.y;
    loc.dataon = true;
}

void InitDmap (DMAP *dm)
{
    dm->wid = WIDSIZ/PIXSIZ;
    dm->len = LENSIZ/PIXSIZ;
    dm->demg = NULL;
    dm->demhmin = NULL;
    dm->demhmax = NULL;
    dm->demgnum = NULL;
    dm->demhnum = NULL;
    dm->lab = NULL;
    dm->sublab = NULL;
    dm->groll = NULL;
    dm->gpitch = NULL;
    dm->lpr = NULL;
    dm->lmap = NULL;
    dm->smap = NULL;
    dm->WX = dm->WY = dm->WZ = NULL;
    dm->centerln = NULL;
    dm->dataon = false;
}

void ReleaseDmap (DMAP *dm)
{
    if (dm->demg) delete []dm->demg;
    if (dm->demhmin) delete []dm->demhmin;
    if (dm->demhmax) delete []dm->demhmax;
    if (dm->demgnum) delete []dm->demgnum;
    if (dm->demhnum) delete []dm->demhnum;
    if (dm->lab) delete []dm->lab;
    if (dm->sublab) delete []dm->sublab;
    if (dm->groll) delete []dm->groll;
    if (dm->gpitch) delete []dm->gpitch;
    if (dm->lpr) delete []dm->lpr;
    if (dm->lmap) cvReleaseImage(&dm->lmap);
    if (dm->smap) cvReleaseImage(&dm->smap);
    if (dm->WX) delete []dm->WX;
    if (dm->WY) delete []dm->WY;
    if (dm->WZ) delete []dm->WZ;
    if (dm->centerln) delete []dm->centerln;
}

#define MAXDEMPTNUM		1000

void LabelRoadSurface (DMAP &glo)
{
    double Equation[4];
    int num, cnt;

    if (!glo.sublab) glo.sublab = new BYTE[glo.wid*glo.len];
    memset (glo.sublab, 0, sizeof(BYTE)*glo.wid*glo.len);
    if (!glo.groll) glo.groll = new double[glo.wid*glo.len];
    memset (glo.groll, 0, sizeof(double)*glo.wid*glo.len);
    if (!glo.gpitch) glo.gpitch = new double[glo.wid*glo.len];
    memset (glo.gpitch, 0, sizeof(double)*glo.wid*glo.len);
    if (!glo.WX) glo.WX = new double[MAXDEMPTNUM];
    if (!glo.WY) glo.WY = new double[MAXDEMPTNUM];
    if (!glo.WZ) glo.WZ = new double[MAXDEMPTNUM];

    int x=0, y=0, xx, yy;

    for (y=0; y<glo.len; y++) {
        for (x=0; x<glo.wid; x++) {
            if (glo.lab[y*glo.wid+x]!=TRAVESABLE || glo.sublab[y*glo.wid+x])
                continue;
            num=0;
            // 用邻域内的点做平面拟合
            for (yy=y; yy<min(y+10,glo.len); yy++) {
                for (xx=x; xx<min(x+10,glo.wid); xx++) {
                    if (glo.lab[yy*glo.wid+xx]!=TRAVESABLE)
                        continue;
                    glo.WX[num]=(xx-glo.wid/2)*PIXSIZ;
                    glo.WY[num]=(yy-glo.len/2)*PIXSIZ;
                    glo.WZ[num]=glo.demg[yy*glo.wid+xx];
                    num ++;
                    if (num>=MAXDEMPTNUM)
                        break;
                }
                if (num>=MAXDEMPTNUM)
                    break;
            }

            BYTE sublab;
            double ax, cx, ay;
            if (num<10) {
                ax = ay = 0;
                sublab = EDGEPOINTS;
            }
            else {
                Calculate_Plane(num,glo.WX,glo.WY,glo.WZ,0,Equation);
                ax = asin (-Equation[1]);
                cx = cos (ax);
                ay = atan2 (Equation[0]/cx, Equation[2]/cx);
                if (fabs(ax)>fabs(ay)) {
                    if (ax>0.174 * 4)				//10deg
                        sublab = UPSLOPE;           // 上坡
                    else if (ax<-0.174 * 4)
                        sublab = DOWNSLOPE;         // 下坡
                    else
                        sublab = FLATGROUND;        // 平地
                }
                else {
                    if (ay>0.174 * 4)
                        sublab = RIGHTSIDESLOPE;    // 右斜坡
                    else if (ax<-0.174 * 4)
                        sublab = LEFTSIDESLOPE;     // 左斜坡
                    else
                        sublab = FLATGROUND;        // 平地
                }
            }
            for (yy=y; yy<min(y+10,glo.len); yy++) {
                for (xx=x; xx<min(x+10,glo.wid); xx++) {
                    if (glo.lab[yy*glo.wid+xx]!=TRAVESABLE)
                        continue;
                    glo.groll[yy*glo.wid+xx] = ax;
                    glo.gpitch[yy*glo.wid+xx] = ay;
                    glo.sublab[yy*glo.wid+xx] = sublab;
                }
            }
        }
    }
}

void LabelRoadSurface1 (DMAP &glo)
{
    double Equation[4];

    if (!glo.sublab) glo.sublab = new BYTE[glo.wid*glo.len];
    memset (glo.sublab, 0, sizeof(BYTE)*glo.wid*glo.len);
    if (!glo.groll) glo.groll = new double[glo.wid*glo.len];
    memset (glo.groll, 0, sizeof(double)*glo.wid*glo.len);
    if (!glo.gpitch) glo.gpitch = new double[glo.wid*glo.len];
    memset (glo.gpitch, 0, sizeof(double)*glo.wid*glo.len);
    if (!glo.WX) glo.WX = new double[MAXDEMPTNUM];
    if (!glo.WY) glo.WY = new double[MAXDEMPTNUM];
    if (!glo.WZ) glo.WZ = new double[MAXDEMPTNUM];

    int x=0, y=0, yy;

    for (y=0; y<glo.len; y++) {

        if ((glo.centerln[y].x1-glo.centerln[y].x0)<2)
            continue;

        int tcnt=0, lcnt;
        int num=0;
        for (yy=y; yy<min(y+10,glo.len); yy++) {

            if (glo.centerln[yy].x1==glo.centerln[yy].x0)
                continue;

            lcnt = 0;
            for (x=glo.centerln[yy].x0; x<=glo.centerln[yy].x1; x++) {

                if (glo.lab[yy*glo.wid+x]!=TRAVESABLE)
                    continue;

                lcnt ++;
                glo.WX[num]=(x-glo.wid/2)*PIXSIZ;
                glo.WY[num]=(yy-glo.len/2)*PIXSIZ;
                glo.WZ[num]=glo.demg[yy*glo.wid+x];
                num ++;
                if (num>=MAXDEMPTNUM)
                    break;
            }
            if (lcnt>2)
                tcnt ++;
            if (num>=MAXDEMPTNUM)
                break;
        }

        BYTE sublab;
        double ax, cx, ay;
        if (tcnt<2 || num<10) {
            ax = ay = 0;
            sublab = EDGEPOINTS;
        }
        else {
            Calculate_Plane(num,glo.WX,glo.WY,glo.WZ,0,Equation);
            ax = asin (-Equation[1]);
            cx = cos (ax);
            ay = atan2 (Equation[0]/cx, Equation[2]/cx);
            if (fabs(ax)>fabs(ay)) {
                if (ax>0.174)					//10deg
                    sublab = UPSLOPE;
                else if (ax<-0.174)
                    sublab = DOWNSLOPE;
                else
                    sublab = FLATGROUND;
            }
            else {
                if (ay>0.174)
                    sublab = RIGHTSIDESLOPE;
                else if (ax<-0.174)
                    sublab = LEFTSIDESLOPE;
                else
                    sublab = FLATGROUND;
            }
        }

        for (yy=y; yy<min(y+10,glo.len); yy++) {
            if (glo.centerln[yy].x1==glo.centerln[yy].x0)
                continue;
            for (x=glo.centerln[yy].x0; x<=glo.centerln[yy].x1; x++) {
                if (glo.lab[yy*glo.wid+x]!=TRAVESABLE)
                    continue;
                glo.groll[yy*glo.wid+x] = ax;
                glo.gpitch[yy*glo.wid+x] = ay;
                glo.sublab[yy*glo.wid+x] = sublab;
            }
        }

        y += 9;
    }
}

void LabelObstacle (DMAP &glo)
{
    if (!glo.sublab)
        return;

    int x, y, y0, y1, yy;
    for (y=0; y<glo.len; y++) {
        for (x=0; x<glo.wid; x++) {

            if (glo.lab[y*glo.wid+x]!=TRAVESABLE)
                continue;

            double dd = sqrt(sqr((y-glo.len/2.0)*PIXSIZ)+sqr((x-glo.wid/2.0)*PIXSIZ));
            if (dd<=NEARVEHICLEDIS)
                continue;

            y1=min(y+10,glo.len);
            for (y0=y+1; y0<y1; y0++)
                if (glo.lab[y0*glo.wid+x]==NONTRAVESABLE || glo.lab[y0*glo.wid+x]==TRAVESABLE)
                    break;
            // y0: 车前最近的不可通行点或者路面
            dd = sqrt(sqr((y0-glo.len/2.0)*PIXSIZ)+sqr((x-glo.wid/2.0)*PIXSIZ));
            if (dd<=NEARVEHICLEDIS)
                continue;
            // 如果车前最近的是 不可通行区域
            if (glo.lab[y0*glo.wid+x]==NONTRAVESABLE) {
                for (yy=y0; yy<min(y0+10,glo.len); yy++) {
                    // 连续的障碍
                    if (glo.lab[yy*glo.wid+x]==NONTRAVESABLE) {
                        // 如果障碍会被撞到
                        // Zhao used &&, Gao Biao changed to ||
                        if (glo.demhmin[yy*glo.wid+x]<(glo.centerln[yy].h+VEHICLEHEIGHT)
                         || glo.demhmax[yy*glo.wid+x]>(glo.centerln[yy].h+POSOBSMINHEIGHT))
                            glo.sublab[yy*glo.wid+x] = POSSIOBSTA;  // 正障碍
                        else
                        if (glo.demhmax[yy*glo.wid+x]<(glo.centerln[yy].h-POSOBSMINHEIGHT))
                            glo.sublab[yy*glo.wid+x] = NEGATOBSTA;  // 负障碍
                    }
                    else
                        break;
                }
            }
            // 如果车前最近的是 可通行区域
            else {
                double dis = (y0-y)*PIXSIZ;
                // 如果两条线距离过大，并且两线之间的点都是UNKNOWN，可能是一条沟
                if (dis > max(2.0, glo.centerln[y0].dl)) {
                    for (yy=y+1; yy<y0; yy++) {
                        if (glo.lab[yy*glo.wid+x]==NONTRAVESABLE || glo.lab[yy*glo.wid+x]==TRAVESABLE)
                            break;
                        glo.sublab[yy*glo.wid+x] = NEGATOBSTA;
                    }
//                    dis=dis;
                }
            }
        }
    }
}

void ExtractRoadCenterline (DMAP &glo)
{

    if (!glo.centerln) glo.centerln = new CENTERLN[glo.len];
    memset (glo.centerln, 0, sizeof(int)*glo.len);

    int x, y, k, yy, invcnt, num;
    double	h;

    for (k=0; k<2; k++) {

        int x0=glo.wid/2;
        for (yy=0; yy<=glo.len/2; yy++) {
            if (!k)
                y=glo.len/2+yy;
            else {
                if (!yy) {
                    x0= (glo.centerln[glo.len/2].x0 + glo.centerln[glo.len/2].x1)/2;
                    continue;
                }
                y=glo.len/2-yy;
            }
            if (y>=glo.len)
                break;
            glo.centerln[y].x0 = glo.centerln[y].x1 = x0;

            h=0;
            num = 0;
            invcnt = 0;
            for (x=x0; x<glo.wid; x++) {

                if (glo.lab[y*glo.wid+x]==TRAVESABLE) {
                    glo.centerln[y].x1=x;
                    h += glo.demg[y*glo.wid+x]*glo.demgnum[y*glo.wid+x];
                    num += glo.demgnum[y*glo.wid+x];
                    invcnt=0;
                }
                else {
                    invcnt ++;
                    if (invcnt>5) break;
                }
            }
            invcnt = 0;
            for (x=glo.wid/2; x>=0; x--) {

                if (glo.lab[y*glo.wid+x]==TRAVESABLE) {
                    glo.centerln[y].x0=x;
                    h += glo.demg[y*glo.wid+x]*glo.demgnum[y*glo.wid+x];
                    num += glo.demgnum[y*glo.wid+x];
                    invcnt=0;
                }
                else {
                    invcnt ++;
                    if (invcnt>5) break;
                }
            }

            if (num)
                glo.centerln[y].h = h/(double)num;
            else
                glo.centerln[y].h = INVALIDDOUBLE;

            x0 = (glo.centerln[y].x0+glo.centerln[y].x1)/2.0;
        }
    }

    int y0, y1;
    double h0, h1;

    h0=h1=INVALIDDOUBLE;
    y=0;
    while (y<glo.len) {

        h0=h1;
        y0=y++;
        for (; y<glo.len; y++)
            if (glo.centerln[y].h != INVALIDDOUBLE) {
                h1 = glo.centerln[y].h;
                break;
            }
        y1=y;
        if ((y1-y0)==1 && h0!=INVALIDDOUBLE && h1!=INVALIDDOUBLE)
            continue;

        if (h0==INVALIDDOUBLE && h1==INVALIDDOUBLE)
            break;
        else
        if (h0==INVALIDDOUBLE)
            h0 = h1;

        for (yy=y0; yy<y1; yy++)
            glo.centerln[yy].h = (h1-h0)*(yy-y0)/(double)(y1-y0)+h0;

    }

    double alpha, delta;	//alpha为前一条扫描线与垂线的夹角、delta扫描线间的夹角
    double dis1, dis2;		//dis1为前一条扫描线到激光器的水平距离、dis2为y对应扫描线到激光器的水平距离
    delta = (VMAXANG-VMINANG)/63.0;		//假设扫描线间夹角相等
    h = VEHICLEHEIGHT;				//激光雷达与地面高度
    glo.centerln[glo.len/2].dl = 0.3;
    for (y=1; y<=glo.len/2; y++) {
        dis1 = y*PIXSIZ;
        alpha = atan2 (dis1,h);
        dis2 = tan (alpha+delta*2.0)*h;
        if (y<glo.len/2)
            glo.centerln[glo.len/2+y].dl = glo.centerln[glo.len/2-y].dl = max (0.3, dis2-dis1);
        else
            glo.centerln[glo.len/2-y].dl = max (0.3, dis2-dis1);
    }
}
