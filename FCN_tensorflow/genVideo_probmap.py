import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import matplotlib.image as mpimg
import pdb

G_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1216_green_vs_others_weak/prob.txt'
R_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1216_red_vs_others_weak/prob.txt'
PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1216_red_vs_others_weak/vis/'
PROJECT_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1216_red_vs_others_weak/project_vis/'
NAME = '1216_vs_others_weak'
DATA_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/data_easy/'

IS_WEAK_LABEL = False
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

def readCost(fg, fr, costMap, pre):
    ts = fg.readline()
    ts = fr.readline()
    for i in xrange(IMAGE_HEIGHT):
        for j in xrange(IMAGE_WIDTH):
            sg = fg.readline()
            sr = fr.readline()
            sg = sg.replace('\n', '').split(' ')
            sr = sr.replace('\n', '').split(' ')
            p_g = float(sg[0])
            p_rb = float(sg[1])
            p_gb = float(sr[0])
            p_r = float(sr[1])
            if (sum(costMap[i, j, :]) != 0):
                if (p_g > p_rb):                    # passable
                    val = (0.5 - (p_g - 0.5)) * 112 + 85 + 30       # [85 ~ 141] HOT_MAP
                    # val = (p_g - 0.5) * 80 + 100                # [100 ~ 140] RAINBOW_MAP
                    costMap[i, j, :] = [val, val, val]
                    pre[i, j, :] = [1, 1, 1]
                elif (p_r > p_gb):                  # unpassable
                    val = (p_r - 0.5) * 114 + 198              # [198 ~ 255] HOT_MAP
                    # val = 40 - (p_r - 0.5) * 80                 # [0 ~ 40] RAINBOW_MAP
                    costMap[i, j, :] = [val, val, val]
                    pre[i, j, :] = [2, 2, 2]
                elif (p_g < p_rb and p_r < p_gb):   # uncertain
                    p_b = p_rb / (p_gb + p_rb)
                    val = p_b * 165 + 87                        # [142 ~ 197] HOT_MAP
                    # val = p_b * 120 + 40                         # [40 ~ 100] RAINBOW_MAP
                    costMap[i, j, :] = [val, val, val]
                    pre[i, j, :] = [3, 3, 3]
    return costMap, pre

# main
fg = open(G_PATH, "r")
fr = open(R_PATH, "r")

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

videoWriter = cv2.VideoWriter(NAME+'.avi', cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'), 10, (IMAGE_WIDTH * 5, IMAGE_HEIGHT), True)
#videoWriter = cv2.VideoWriter('test.avi', cv2.VideoWriter_fourcc(*'XVID'), 5, (IMAGE_WIDTH, IMAGE_HEIGHT * 2), True)

for i in range(len(imgList)):
    img = cv2.imread(DATA_PATH + "test/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    gt  = cv2.imread(DATA_PATH + "test_gt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    wgt = cv2.imread(DATA_PATH + "test_wgt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    wgt = wgtProcess(wgt)
    bgt = cv2.imread(DATA_PATH + "test_bgt/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    videoImg = cv2.imread(DATA_PATH + "video/" + imgList[i].split('.')[0].split('_')[1] + ".png", cv2.IMREAD_COLOR)
    # gt = cv2.flip(gt, 0)
    pre = cv2.imread(PATH + preList[i], cv2.IMREAD_COLOR)
    new_gt = gt.copy()
    gt = preProcess(img, gt)
    pre = preProcess(img, pre)
    # costMap = cv2.imread(PATH + costmapList[i], cv2.IMREAD_COLOR)
    costMap = img.copy()
    costMap, pre = readCost(fg, fr, costMap, pre)

    gtClone = gt.copy()
    gtClone1 = gt.copy()
    grayPre = pre.copy()
    
    IS_WEAK_LABEL = False
    table = LabelColor(img, gt, pre)
    cntTable += table
    table = LabelColor(img, gtClone, bgt)
    cntTableBaseline += table
    IS_WEAK_LABEL = True
    none = LabelColor(img, wgt, gtClone1)

    colorizedImg = cv2.applyColorMap(costMap, cv2.COLORMAP_HOT)
    colorizedImg = preProcess(img, colorizedImg)
    videoImg = cv2.resize(videoImg, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)
    # mergeImg0 = np.concatenate((img, wgt, pre), axis=1)
    # mergeImg1 = np.concatenate((videoImg, bgt, gt), axis=1)
    # mergeImg = np.concatenate((mergeImg0, mergeImg1), axis=0)
    mergeImg = np.concatenate((img, colorizedImg, pre, gt, wgt), axis=1)
    videoWriter.write(mergeImg)
    print('Frame: %d / %d' % (i, len(imgList)))

    cv2.imwrite(PROJECT_PATH + "pre_" + imgList[i].split('.')[0].split('_')[1] + ".png", grayPre)
    cv2.imwrite(PROJECT_PATH + "prob_" + imgList[i].split('.')[0].split('_')[1] + ".png", costMap)

OutputResult(cntTable, cntTableBaseline)

videoWriter.release()
fg.close()
fr.close()
