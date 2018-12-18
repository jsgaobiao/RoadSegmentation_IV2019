import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import matplotlib.image as mpimg
import pdb

PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1216_2label_weak/vis/'
NAME = '1216_2label_weak'
DATA_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/data_easy/'

IS_WEAK_LABEL = True
IMAGE_WIDTH = 300
IMAGE_HEIGHT = 300
NUM_OF_CLASSESS = 4

def ColorMap(data):
    if data[0] == 0:        # unknown
        data = [0, 0, 0]
    elif data[0] == 1:      # passable
        data = [0, 255, 0]
    elif data[0] == 2:      # iprecisionssable
        data = [0, 0, 255]
    elif data[0] == 3:      # uncertain
        data = [255, 0, 0]
    return data

def wgtProcess(wgt):
    for i in range(IMAGE_HEIGHT):
        for j in range(IMAGE_WIDTH):
            if (wgt[i, j, 0] == 3):
                wgt[i, j] = [0, 0, 0]
    return wgt

def preProcess(img, pre):
    for i in range(IMAGE_HEIGHT):
        for j in range(IMAGE_WIDTH):
            if (img[i, j, 0] == 0):
                pre[i,j] = [0, 0, 0]
    return pre

def LabelColor(img, gt, pre):
    table = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
    height, width, channel = gt.shape
    for i in range(height):
        for j in range(width):
            # caculate accuracy
            table[gt[i,j,0]][pre[i,j,0]] += 1
            # Label color of gt
            if (IS_WEAK_LABEL and gt[i,j,0] == 0):
                gt[i,j] = img[i,j]
            else:
                gt[i,j] = ColorMap(gt[i,j])
            # Label color of prediction
            pre[i,j] = ColorMap(pre[i,j])
    return table

def OutputResult(table, baseline):
    fout = open(NAME+'.result', 'w')
    for i in range(NUM_OF_CLASSESS):
        for j in range(NUM_OF_CLASSESS):
            fout.write('%d\t' % table[i,j])
        fout.write('\n')
    fout.write('---------------------------\n')
    fout.write('label\ttp\tfp\tfn\tIoU\trecall\tprecision\n')
    for i in range(1,NUM_OF_CLASSESS):
        tp = fp = fn = cn = 0
        tp = table[i,i].astype(int)
        fn += table[i,0]
        for j in range(1, NUM_OF_CLASSESS):
            if i != j:
                fp += table[j,i].astype(int)
                fn += table[i,j].astype(int)
        if (tp + fp + fn != 0):
            IoU = float(tp) / float(tp + fp + fn)
        else:
            IoU = 0

        if (tp + fn != 0):
            recall = float(tp) / float(tp + fn)
        else:
            recall = 0

        if (tp + fp != 0):
            precision = float(tp)/float(tp+fp)
        else:
            precision = 0
        fout.write('%d\t%d\t%d\t%d\t%.6f\t%.6f\t%.6f\n' % (int(i), int(tp), int(fp), int(fn), float(IoU), float(recall), float(precision)))

    fout.write('-------------Baseline--------------\n')
    for i in range(NUM_OF_CLASSESS):
        for j in range(NUM_OF_CLASSESS):
            fout.write('%d\t' % baseline[i,j])
        fout.write('\n')
    fout.write('---------------------------\n')
    fout.write('label\ttp\tfp\tfn\tIoU\trecall\tprecision\n')
    for i in range(1,NUM_OF_CLASSESS):
        tp = fp = fn = cn = 0
        tp = baseline[i,i].astype(int)
        fn += table[i,0]
        for j in range(1, NUM_OF_CLASSESS):
            if i != j:
                fp += baseline[j,i].astype(int)
                fn += baseline[i,j].astype(int)
        if (tp + fp + fn != 0):
            IoU = float(tp) / float(tp + fp + fn)
        else:
            IoU = 0

        if (tp + fn != 0):
            recall = float(tp) / float(tp + fn)
        else:
            recall = 0

        if (tp + fp != 0):
            precision = float(tp)/float(tp+fp)
        else:
            precision = 0
        fout.write('%d\t%d\t%d\t%d\t%.6f\t%.6f\t%.6f\n' % (int(i), int(tp), int(fp), int(fn), float(IoU), float(recall), float(precision)))
    fout.close()


# main
listName = os.listdir(PATH)
imgList = []
gtList = []
preList = []
costmapList = []
cntTable = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
cntTableBaseline = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))

for name in listName:
    # input image
    if name.split('/')[-1][0] == 'i':
        imgList.append(name)
    # ground truth
    elif name.split('/')[-1][0] == 'g':
        gtList.append(name)
    # prediction
    elif name.split('/')[-1][0] == 'p':
        preList.append(name)
    # costmap
    elif name.split('/')[-1][0] == 'c':
        costmapList.append(name)

imgList.sort()
gtList.sort()
preList.sort()
costmapList.sort()

videoWriter = cv2.VideoWriter(NAME+'.avi', cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'), 10, (IMAGE_WIDTH * 4, IMAGE_HEIGHT * 1), True)
#videoWriter = cv2.VideoWriter('test.avi', cv2.VideoWriter_fourcc(*'XVID'), 5, (IMAGE_WIDTH, IMAGE_HEIGHT * 2), True)

for i in range(len(imgList)):
    img = cv2.imread(PATH + imgList[i], cv2.IMREAD_COLOR)
    wgt = cv2.imread(DATA_PATH + "test_wgt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    wgt = wgtProcess(wgt)
    gt  = cv2.imread(DATA_PATH + "test_gt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    bgt = cv2.imread(DATA_PATH + "test_bgt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    videoImg = cv2.imread(DATA_PATH + "video/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    # gt = cv2.flip(gt, 0)
    pre = cv2.imread(PATH + preList[i], cv2.IMREAD_COLOR)
    gt = preProcess(img, gt)
    pre = preProcess(img, pre)
    costMap = cv2.imread(PATH + costmapList[i], cv2.IMREAD_COLOR)

    gtClone = gt.copy()
    gtClone1 = gt.copy()
    IS_WEAK_LABEL = False
    table = LabelColor(img, gt, pre)
    cntTable += table
    table = LabelColor(img, gtClone, bgt)
    cntTableBaseline += table
    IS_WEAK_LABEL = True
    none = LabelColor(img, wgt, gtClone1)

    colorizedImg = cv2.applyColorMap(costMap, cv2.COLORMAP_HOT)
    videoImg = cv2.resize(videoImg, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)
    mergeImg0 = np.concatenate((img, wgt, pre,gt), axis=1)
    # mergeImg1 = np.concatenate((videoImg, colorizedImg, gt), axis=1)
    # mergeImg = np.concatenate((mergeImg0, mergeImg1), axis=0)
    # mergeImg = cv2.flip(mergeImg, 0)
    videoWriter.write(mergeImg0)
    print('Frame: %d / %d' % (i, len(imgList)))

OutputResult(cntTable, cntTableBaseline)

videoWriter.release()
