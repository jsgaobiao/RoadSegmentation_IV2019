#include <utils.h>
#include <QCoreApplication>
#include <QSettings>
#include <QString>
#include <QDebug>
#include <cstdlib>

string CONFIG_FILE = "../DemLabelTool/config.ini";
string FILE_LIST;
string ANNOTATION_PATH;
string UNANNOTATED_PATH;
string GLOBAL_GT_FILE;
bool GLOBAL_GT;

string CALIB_FILE;
string DSV_FILE;
string AVI_FILE;
string NAV_FILE;
string CAM_CALIB_FILE;

long long START_TIME = 0;
long long END_TIME = 1E10;

FILE *dfp;
FILE* fFileNameList;
std::vector<NAVDATA> nav;
bool camCalibFlag = true;
TRANSINFO calibInfo;
int dsbytesiz = sizeof (point3d) * 2 + sizeof (ONEVDNDATA);
ONEDSVFRAME     *onefrm;
ONEDSVDATA preFrameDsv;

int dFrmNum;
int dFrmNo;
int startFrmNo = -1;
int waitkeydelay = 0;

vector<QString> fileName;
vector<cv::Scalar> colorTable;
vector<cv::Mat> undoList;
cv::Mat videoImg;
cv::Mat inputImg;               // input image (BGR)
cv::Mat inputImgForVis;         // input image (BGR) for label visualization
cv::Mat originGtImg;            // ground truth image (BGR)
cv::Mat newGtImg;               // new ground truth (BGR), ready to be written
cv::Mat mergeImg;               // merge input & gt for visualization
cv::Mat annotatedImg;           // annotated layer (Gray)

double SCALE_RATIO     = 2.0;
int ACTIVE_LABEL    = 0;               // annotation label class
int PEN_WIDTH       = 3;

_DMAP dm;

void LoadConfigFile()
{
    QSettings *configFile = new QSettings(QString(CONFIG_FILE.c_str()), QSettings::IniFormat);

    // Read data path
    ANNOTATION_PATH = configFile->value("Path/ANNOTATION_PATH").toString().toStdString();
    if (ANNOTATION_PATH[ANNOTATION_PATH.length() - 1] != '/') {
        ANNOTATION_PATH += '/';
    }
    UNANNOTATED_PATH = configFile->value("Path/UNANNOTATED_PATH").toString().toStdString();
    if (UNANNOTATED_PATH[UNANNOTATED_PATH.length() - 1] != '/') {
        UNANNOTATED_PATH += '/';
    }
    FILE_LIST = configFile->value("Path/FILE_LIST").toString().toStdString();
    CALIB_FILE = configFile->value("Path/CALIB_FILE").toString().toStdString();
    DSV_FILE = configFile->value("Path/DSV_FILE").toString().toStdString();
    AVI_FILE = configFile->value("Path/AVI_FILE").toString().toStdString();
    CAM_CALIB_FILE = configFile->value("Path/CAM_CALIB_FILE").toString().toStdString();
    NAV_FILE = configFile->value("Path/NAV_FILE").toString().toStdString();
    GLOBAL_GT_FILE = configFile->value("Path/GLOBAL_GT_FILE").toString().toStdString();
    START_TIME = atoi(configFile->value("Parameter/START_TIME").toString().toStdString().c_str());
    END_TIME = atoi(configFile->value("Parameter/END_TIME").toString().toStdString().c_str());
    GLOBAL_GT = atoi(configFile->value("Parameter/GLOBAL_GT").toString().toStdString().c_str());
    if (END_TIME == -1) END_TIME = 1E10;

    // Get label table
    configFile->beginGroup("Label");
    QStringList keys = configFile->allKeys();
    colorTable.clear();
    string lKey, bgr;
    int b, g, r;
    for (int i = 0; i < keys.size(); i ++) {
        lKey = keys.at(i).toLocal8Bit().constData();
        bgr = configFile->value(lKey.c_str()).toString().toStdString();
        sscanf(bgr.c_str(), "%d,%d,%d", &b, &g, &r);
        colorTable.push_back(cv::Scalar(b, g, r));
    }
}

void Init() {
    // Read file name list of dataset
    fFileNameList = fopen(FILE_LIST.c_str(), "r");
    fileName.clear();
    char fName[20];
    while (fscanf(fFileNameList, "%s\n", fName) != EOF) {
        fileName.push_back(fName);
    }
    if (!LoadCalibFile(CALIB_FILE.c_str())) {
        printf("Invalid calibration file\n");
        getchar();
        exit(1);
    }
    // dsv
    if ((dfp = fopen(DSV_FILE.c_str(), "r")) == NULL) {
        printf("DSV file open failure\n");
        getchar();
        exit(1);
    }
    // video
    cv::VideoCapture cap(AVI_FILE.c_str());
    FILE* tsFp = fopen((AVI_FILE + ".ts").c_str(), "r");
    if (!cap.isOpened()) {
        printf("Error opening video stream or file.\n");
        getchar();
        exit(1);
    }
    // Camera/Velodyne calib file
    if (!LoadCameraCalib(CAM_CALIB_FILE.c_str())) {
        printf("Open Camera Calibration files fails.\n");
        camCalibFlag = false;
    }
    // Global GT file
    if (GLOBAL_GT) {
        dm.lmap = cv::imread(GLOBAL_GT_FILE, cv::IMREAD_COLOR);
        if (!LoadNavFile (NAV_FILE, dm)) {
            printf ("Invalid nav file.\n");
            getchar ();
            exit (1);
        }
    }
    LONGLONG fileSize = myGetFileSize(dfp);
    dFrmNum = fileSize / (BKNUM_PER_FRM) / dsbytesiz;
    onefrm = new ONEDSVFRAME[1];
    dFrmNo = 0;
    preFrameDsv.millisec = 0;

    cv::namedWindow("input");
    cv::namedWindow("video & point cloud");
    cv::namedWindow("newGT");
    cv::namedWindow("Baseline");
    cv::namedWindow("annotation");
    cv::namedWindow("human annotation");
    cv::namedWindow("video");
    cv::moveWindow("input", 20, 20);
    cv::moveWindow("annotation", 680, 20);
    cv::moveWindow("video & point cloud", 1340, 400);
    cv::moveWindow("video", 1340, 20);
    cv::moveWindow("newGT", 20, 680);
    cv::moveWindow("human annotation", 390, 680);
    cv::moveWindow("Baseline", 700, 680);
}

void ReadDsv(int ts) {
    // Back up pre frame's dsv[0]
    if (preFrameDsv.millisec > 0) {
        preFrameDsv = onefrm->dsv[0];
    }
    if (onefrm->dsv[0].millisec == ts) {
        return;
    }

    ReadOneDsvFrame();
    dFrmNo++;
    while (onefrm->dsv[0].millisec < ts) {
        // Read next frame
        ReadOneDsvFrame();
        dFrmNo++;
    }
    while (onefrm->dsv[0].millisec > ts) {
        // Read previous frame
        dFrmNo = max(dFrmNo - 2, 0);
        fseeko64(dfp, (LONGLONG)dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
        ReadOneDsvFrame();
        dFrmNo++;
    }

    // if it's the first frame, set preFrameDsv to the current frame data
    if (preFrameDsv.millisec == 0) {
        preFrameDsv = onefrm->dsv[0];
    }
}

void LabelImage(int x, int y) {
    x = int(x / SCALE_RATIO);
    y = int(y / SCALE_RATIO);
    int c = ACTIVE_LABEL;
    cv::Scalar cs = colorTable[c];

    // Can't label the pixel without laser point
    if (inputImg.at<cv::Vec3b>(x, y)[0] == 0 &&
        inputImg.at<cv::Vec3b>(x, y)[1] == 0 &&
        inputImg.at<cv::Vec3b>(x, y)[2] == 0) {
        return;
    }
    cv::circle(annotatedImg, cv::Point(y, x), PEN_WIDTH, cv::Scalar(c), -1);
    cv::circle(inputImgForVis, cv::Point(y, x), PEN_WIDTH, cs, -1);
}

void UpdateVis() {
    // Update newGT
    for (int i = 0; i < annotatedImg.rows; i++) {
        for (int j = 0; j < annotatedImg.cols; j++) {
            int ac = annotatedImg.at<uchar>(i, j);
            if (ac == 255) {
                continue;
            }
            newGtImg.at<cv::Vec3b>(i, j)[0] = colorTable[ac][0];
            newGtImg.at<cv::Vec3b>(i, j)[1] = colorTable[ac][1];
            newGtImg.at<cv::Vec3b>(i, j)[2] = colorTable[ac][2];
        }
    }
    // Show image
    cv::Mat tmpImg;
    cv::addWeighted(inputImg, 0.7, newGtImg, 0.6, 0, mergeImg);
    cv::resize(mergeImg, tmpImg, cv::Size(mergeImg.cols * SCALE_RATIO, mergeImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
    cv::imshow("annotation", tmpImg);
    cv::imshow("human annotation", annotatedImg);
    cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
    cv::imshow("input", tmpImg);
}

void CallbackAnnotation(int event, int x, int y, int flags, void *param) {
    if (event == CV_EVENT_LBUTTONDOWN || (flags & CV_EVENT_FLAG_LBUTTON)) {
        LabelImage(y, x);
        UpdateVis();
    }
    if (event == CV_EVENT_LBUTTONUP) {
        // Show point cloud projected to the video
        cv::Mat tmpImg = videoImg.clone();
        pointCloudsProject(tmpImg, newGtImg, inputImg);
        cv::imshow("video & point cloud", tmpImg);
        cv::imshow("newGT", newGtImg);
        inputImgForVis = inputImg.clone();
        cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
        cv::imshow("input", tmpImg);
        undoList.push_back(annotatedImg.clone());
    }
    // Undo annotation
    if (event == CV_EVENT_RBUTTONUP) {
        if (undoList.size() > 1) undoList.pop_back();
        if (undoList.empty()) return;
        annotatedImg = undoList[undoList.size()-1].clone();
        newGtImg = originGtImg.clone();
        Gt2BGR(newGtImg);
        UpdateVis();
        // Show point cloud projected to the video
        cv::Mat tmpImg = videoImg.clone();
        pointCloudsProject(tmpImg, newGtImg, inputImg);
        cv::imshow("video & point cloud", tmpImg);
        cv::imshow("newGT", newGtImg);
        inputImgForVis = inputImg.clone();
        cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
        cv::imshow("input", tmpImg);
    }
}

void CallbackInput(int event, int x, int y, int flags, void *param) {
    if (event == CV_EVENT_LBUTTONDOWN || (flags & CV_EVENT_FLAG_LBUTTON)) {
        LabelImage(y, x);
        UpdateVis();
    }
    if (event == CV_EVENT_LBUTTONUP) {
        // Show point cloud projected to the video
        cv::Mat tmpImg = videoImg.clone();
        pointCloudsProject(tmpImg, newGtImg, inputImg);
        cv::imshow("video & point cloud", tmpImg);
        cv::imshow("newGT", newGtImg);
        inputImgForVis = inputImg.clone();
        cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
        cv::imshow("input", tmpImg);
        undoList.push_back(annotatedImg.clone());
    }
    // Undo annotation
    if (event == CV_EVENT_RBUTTONUP) {
        if (undoList.size() > 1) undoList.pop_back();
        if (undoList.empty()) return;
        annotatedImg = undoList[undoList.size()-1].clone();
        newGtImg = originGtImg.clone();
        Gt2BGR(newGtImg);
        UpdateVis();
        // Show point cloud projected to the video
        cv::Mat tmpImg = videoImg.clone();
        pointCloudsProject(tmpImg, newGtImg, inputImg);
        cv::imshow("video & point cloud", tmpImg);
        cv::imshow("newGT", newGtImg);
        inputImgForVis = inputImg.clone();
        cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
        cv::imshow("input", tmpImg);
    }
}

// Set gt image to global gt
void SetGtImg(_DMAP &m, cv::Mat &gtImg)
{
    MATRIX	rot;
    cv::Mat tmpImg;
    point3d	shv = onefrm->dsv[0].shv;
    double gz = shv.z-calibInfo.shv.z;
    tmpImg = gtImg.clone();
    cv::transpose(tmpImg, tmpImg);
    cv::flip(tmpImg, tmpImg, 1);

    createRotMatrix_ZYX(rot, 0, 0, onefrm->dsv[0].ang.z) ;

    for (int i = 0; i < tmpImg.rows; i ++) {
        for (int j = 0; j < tmpImg.cols; j ++) {
            point3fi p;
            p.x = (double)(i - tmpImg.rows/2.0) * PIXSIZ;
            p.y = (double)(j - tmpImg.cols/2.0) * PIXSIZ;
            rotatePoint3fi(p, rot);
            shiftPoint3fi(p, shv);

            int ix, iy;   // (ix, iy) in m
            ix = nint((p.x-m.x0)/PIXSIZ);
            iy = nint((p.y-m.y0)/PIXSIZ);
            iy = m.lmap.rows - iy - 1;
            for (int y = iy; y <= iy+1; y ++) {
                for (int x = ix; x <= ix+1; x ++) {
                    if (y<0 || y>=m.lmap.rows) continue;
                    if (x<0 || x>=m.lmap.cols) continue;

                    m.lmap.at<cv::Vec3b>(y, x)[0] = tmpImg.at<cv::Vec3b>(i, j)[0];
                    m.lmap.at<cv::Vec3b>(y, x)[1] = tmpImg.at<cv::Vec3b>(i, j)[1];
                    m.lmap.at<cv::Vec3b>(y, x)[2] = tmpImg.at<cv::Vec3b>(i, j)[2];
                }
            }
        }
    }
}

// Save new ground truth
void SaveNewGT(int idx) {
    if (GLOBAL_GT) {
        SetGtImg(dm, newGtImg);
        cv::imwrite(GLOBAL_GT_FILE + ".new.png" , dm.lmap);
        printf("%s gt saved!\n", fileName[idx].toStdString().c_str());
    }
    else {
        cv::Mat writtenGtImg(originGtImg.rows, originGtImg.cols, CV_8UC1);
        writtenGtImg.setTo(0);
        BGR2Gt(newGtImg, writtenGtImg);
        if (cv::imwrite((ANNOTATION_PATH + fileName[idx].toStdString() + "_gt.png").c_str(), writtenGtImg)) {
            printf("%s_gt.png saved!\n", fileName[idx].toStdString().c_str());
        } else {
            printf("save error!\n");
        }
    }
}

void CheckTimestampRange() {
    ReadOneDsvFrame();
    long long startTs = onefrm->dsv[0].millisec;
    int err = fseeko64(dfp, -(LONGLONG)dsbytesiz * BKNUM_PER_FRM, SEEK_END);
    ReadOneDsvFrame();
    long long endTs = onefrm->dsv[0].millisec;
    printf("DSV's time range: %lld ~ %lld\n\n", startTs, endTs);
    if (START_TIME == 0) {
        START_TIME = startTs;
    }
    if (END_TIME == 1E10) {
        END_TIME = endTs;
    }
    if (START_TIME < startTs) {
        printf(RED "[Warning] START_TIME (%lld) is out of DSV time range, please check DSV file path.\n\n" NONE, START_TIME);
        printf("Press [any key] toANNOTATION_PATH continue\n");
        getchar();
        START_TIME = startTs;
    }
    if (END_TIME > endTs) {
        printf(RED "[Warning] END_TIME (%lld) is out of DSV time range, please check DSV file path.\n\n" NONE, END_TIME);
        printf("Press [any key] to continue\n");
        getchar();
        END_TIME = endTs;
    }
    fseeko64(dfp, 0, SEEK_SET);
}

int FindStartFrame(int ts)
{
    int ret = 0;
    DWORD	dwReadBytes;
    int p3dByteSize = sizeof(point3d);
    int tsByteSize = sizeof(long long);
    long long tmpTs;
    point3d tmpP3d;
    printf("Reading DSV data, please waiting ......\n");
    while (true) {
        int err = fseeko64(dfp, (LONGLONG)ret * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
        dwReadBytes = fread(&tmpP3d, 1, p3dByteSize, dfp);
        dwReadBytes = fread(&tmpP3d, 1, p3dByteSize, dfp);
        dwReadBytes = fread(&tmpTs, 1, tsByteSize, dfp);
        if (ferror(dfp)) break;
        if (tmpTs == ts) break;
        ret++;
    }
    fseeko64(dfp, 0, SEEK_SET);
    return ret;
}

// Get gt image from global gt
void GetGtImg(_DMAP &m, cv::Mat &gtImg)
{
    MATRIX	rot;
    point3d	shv = onefrm->dsv[0].shv;
    double gz = shv.z-calibInfo.shv.z;

    createRotMatrix_ZYX(rot, 0, 0, onefrm->dsv[0].ang.z) ;

    gtImg = cv::Mat(WIDSIZ/PIXSIZ, LENSIZ/PIXSIZ, CV_8UC3);
    gtImg.setTo(0);

    for (int i = 0; i < gtImg.rows; i ++) {
        for (int j = 0; j < gtImg.cols; j ++) {
            point3fi p;
            p.x = (double)(i - gtImg.rows/2.0) * PIXSIZ;
            p.y = (double)(j - gtImg.cols/2.0) * PIXSIZ;
            rotatePoint3fi(p, rot);
            shiftPoint3fi(p, shv);

            int ix, iy;   // (ix, iy) in m
            ix = nint((p.x-m.x0)/PIXSIZ);
            iy = nint((p.y-m.y0)/PIXSIZ);
            if (iy<0 || iy>=m.lmap.rows) continue;
            if (ix<0 || ix>=m.lmap.cols) continue;

            int val = BGR2Gt(m.lmap.at<cv::Vec3b>(m.lmap.rows - iy - 1, ix));
            if (val < 0) fprintf(stderr, "Color Error!\n");
            gtImg.at<cv::Vec3b>(i, j)[0] = val;
            gtImg.at<cv::Vec3b>(i, j)[1] = val;
            gtImg.at<cv::Vec3b>(i, j)[2] = val;
        }
    }
    cv::flip(gtImg, gtImg, 1);
    cv::transpose(gtImg, gtImg);
}

int main(int argc, char* argv[]) {
    system("ulimit -s 16384");
    if (argc > 1) {
        CONFIG_FILE = argv[1];
    }
    LoadConfigFile();
    Init();
    CheckTimestampRange();
    startFrmNo = FindStartFrame(START_TIME);

    for (int idx = 0; idx < fileName.size(); idx++) {
        int ts = fileName[idx].toInt();
        if (ts < START_TIME) continue;
        if (ts > END_TIME) break;
        if (ts == START_TIME) {
            dFrmNo = startFrmNo;
            int err = fseeko64(dfp, (LONGLONG)dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
        }
        if (atoi(fileName[idx].toStdString().c_str()) > END_TIME) {
            printf(RED "End of DSV file. (Timestamp : %lld)\n" NONE, END_TIME);
            printf("Press [any key] to exit.\n");
            getchar();
            return 0;
        }

        // Read DSV data
        ReadDsv(ts);
        printf("no.%d, time: %s\n", dFrmNo, fileName[idx].toStdString().c_str());

        // Read image data
        inputImg = cv::imread(UNANNOTATED_PATH + fileName[idx].toStdString() + "_img.png", cv::IMREAD_COLOR);
        videoImg = cv::imread(UNANNOTATED_PATH + fileName[idx].toStdString() + "_video.png", cv::IMREAD_COLOR);
        inputImgForVis = inputImg.clone();

        // Read human annotated ground truth
        cv::Mat readImg;

        if (GLOBAL_GT) {
            GetGtImg(dm, originGtImg);
            readImg = originGtImg.clone();
            newGtImg = readImg.clone();
            Gt2BGR(newGtImg);
            // if change frame id, then empty the annotatedImg
            if (preFrameDsv.millisec != 0 && preFrameDsv.millisec != onefrm->dsv[0].millisec) {
                annotatedImg = cv::Mat(inputImg.rows, inputImg.cols, CV_8UC1);
                annotatedImg.setTo(255);
                undoList.clear();
                undoList.push_back(annotatedImg.clone());
            }
        }
        else {
            originGtImg = cv::imread(UNANNOTATED_PATH + fileName[idx].toStdString() + "_gt.png", cv::IMREAD_COLOR);
            readImg = cv::imread(ANNOTATION_PATH + fileName[idx].toStdString() + "_gt.png");
            if (readImg.empty()) newGtImg = originGtImg.clone();
                            else newGtImg = readImg.clone();
            Gt2BGR(newGtImg);
            if (preFrameDsv.millisec != 0) {    // not the 1st frame
                // transform the previous human annotation into the coordination of the newest position
                TransAnnotation(annotatedImg);
                if (!undoList.empty())
                    undoList[0] = annotatedImg.clone();
                for (int z = 1; z < undoList.size(); z ++)
                    TransAnnotation(undoList[z]);
            }
        }

        if (annotatedImg.empty()) {
            annotatedImg = cv::Mat(inputImg.rows, inputImg.cols, CV_8UC1);
            annotatedImg.setTo(255);
            undoList.push_back(annotatedImg.clone());
        }

        // Show image
        cv::Mat tmpImg;
        UpdateVis();
        cv::resize(inputImgForVis, tmpImg, cv::Size(inputImg.cols * SCALE_RATIO, inputImg.rows * SCALE_RATIO), 0, 0, INTER_NEAREST);
        cv::imshow("input", tmpImg);
        cv::imshow("video", videoImg);

        // Show point cloud projected to the video
        tmpImg = videoImg.clone();
        pointCloudsProject(tmpImg, newGtImg, inputImg);
        cv::imshow("newGT", newGtImg);
        cv::imshow("video & point cloud", tmpImg);
        tmpImg = originGtImg.clone();
        Gt2BGR(tmpImg);
        cv::imshow("Baseline", tmpImg);

        cv::setMouseCallback("annotation", CallbackAnnotation);
        cv::setMouseCallback("input", CallbackInput);

        // Catch keyboard event
        int WaitKey = cv::waitKey(waitkeydelay);
        if (WaitKey == 27) {      // ESC
            break;
        } else
        if (WaitKey == 'z') {     // ��������
            if (waitkeydelay == 1) {
                waitkeydelay = 0;
            } else {
                waitkeydelay = 1;
            }
        } else
        if (WaitKey == 'a') {     // Back
            idx = max(idx - 2, -1);
            continue;
        } else
        if (WaitKey == 'd') {     // Forword
//            SaveNewGT(idx);
            continue;
        } else
        if (WaitKey == 'r') {       // Reset human annotation
            annotatedImg.setTo(255);
            undoList.clear();
            undoList.push_back(annotatedImg.clone());
            if (GLOBAL_GT) {
//                SetGtImg(bm, originGtImg);
            }
            else {
                std::remove((ANNOTATION_PATH + fileName[idx].toStdString() + "_gt.png").c_str());
            }
            idx--;
        } else
        if (WaitKey >= '0' && WaitKey <= '9') {  // change active label class
            ACTIVE_LABEL = WaitKey - '0';
            if (ACTIVE_LABEL >= colorTable.size())
                ACTIVE_LABEL = 0;
            printf("Active Label : %d\n", ACTIVE_LABEL);
            idx--;
        } else
        if (WaitKey == 32) {        // space bar: save the human annotation & new ground truth
            SaveNewGT(idx);
        } else
        if (WaitKey == 'p') {        // change PEN_WIDTH
            printf("Please enter pen width (current PEN_WIDTH=%d): ", PEN_WIDTH);
            scanf("%d", &PEN_WIDTH);
            idx--;
        } else
        if (WaitKey != -1) {
            idx--;
        }
    }
    printf("\nDone.\n");
    fclose(fFileNameList);
    return 0;
}
