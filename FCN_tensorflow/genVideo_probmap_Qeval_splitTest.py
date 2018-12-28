import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import matplotlib.image as mpimg
import pdb

G_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1210_green_vs_others/prob.txt'
R_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1210_red_vs_others/prob.txt'
PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1210_red_vs_others/vis/'
PROJECT_PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/1210_red_vs_others/project_vis/'
NAME = '1210_vs_others_newdata_Qval_splitTest'
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

def LabelColor(img, ret, flag = False):
    # table = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
    height, width, channel = ret.shape
    for i in range(height):
        for j in range(width):
            # caculate accuracy
            # table[gt[i,j,0]][pre[i,j,0]] += 1

            # Label color of ret
            if (flag and ret[i,j,0] == 0):
                ret[i,j] = img[i,j]
            else:
                ret[i,j] = ColorMap(ret[i,j])

def OutputResult(table, tableWeak):
    # Q1 = TP(G+B) / Tot(preG)                  // precision
    # Q1_1 =  TP(G) / Tot(preG)                 // precision (tight)
    # Q2 = TP(weakG) / Tot(weakG)               // weak accuracy
    # Q2.1 = 1 - ( FP(weakG) / Tot(preG) )      // unsuitable for our problem
    # Q3 = TP(G) / Tot(G)                       // recall
    # Q4 = TP(G+B) / Tot(preG+preB)             // precision

    tot_preG = float(sum(table[:,1]) - table[0,1])              # table[1,1] + table[2,1] + table[3,1]
    tot_preGB = tot_preG
    tot_G = float(sum(table[1,:]))
    tp_GB = float(sum(table[:,1]) - table[2,1] - table[0,1])    # table[1,1] + table[3,1]
    tp_G = float(table[1,1])
    fp_G = float(sum(table[:,1]) - table[1,1])                  # table[0,1] + table[2,1] + table[3,1]
    tp_G_B = tp_GB
    if (NUM_OF_CLASSESS > 3):
        tp_G_B += float(sum(table[:,3]) - table[2,3] - table[0,3])
        tot_preGB += float(sum(table[:,3]))

    tp_WG = float(tableWeak[1,1])       
    tot_WG = float(sum(tableWeak[1,:]))
    fout.write('Q1: %.6f\n' % (tp_GB / tot_preG))
    fout.write('Q1_1: %.6f\n' % (tp_G / tot_preG))
    fout.write('Q2: %.6f\n' % (tp_WG / tot_WG))
    fout.write('Q3: %.6f\n' % (tp_G / tot_G))
    # fout.write('Q4: %.6f\n' % (tp_G_B / tot_preGB))
    fout.write('\n')
    
    fout.write('------------table (GT human)---------------\n')
    for i in range(NUM_OF_CLASSESS):
        for j in range(NUM_OF_CLASSESS):
            fout.write('%d\t' % table[i,j])
        fout.write('\n')
    fout.write('\nlabel\ttp\tfp\tfn\tIoU\trecall\tprecision\n')
    for i in range(1,NUM_OF_CLASSESS):
        tp = fp = fn = IoU = recall = precision = 0
        tp = table[i,i].astype(int)
        fn += table[i,0]
        for j in range(1, NUM_OF_CLASSESS):
            if i != j:
                fp += table[j,i].astype(int)
                fn += table[i,j].astype(int)
        if (tp + fp + fn != 0):
            IoU = float(tp) / float(tp + fp + fn)
        if (tp + fn != 0):
            recall = float(tp) / float(tp + fn)
        if (tp + fp != 0):
            precision = float(tp) / float(tp+fp)
        fout.write('%d\t%d\t%d\t%d\t%.6f\t%.6f\t%.6f\n' % (int(i), int(tp), int(fp), int(fn), float(IoU), float(recall), float(precision)))

    # fout.write('\n-------------table (Weak annotation)--------------\n')
    # for i in range(NUM_OF_CLASSESS):
    #     for j in range(NUM_OF_CLASSESS):
    #         fout.write('%d\t' % tableWeak[i,j])
    #     fout.write('\n')
    # fout.write('---------------------------\n')
    # fout.write('label\ttp\tfp\tfn\tIoU\trecall\tprecision\n')
    # for i in range(1,NUM_OF_CLASSESS):
    #     tp = fp = fn = IoU = recall = precision = 0
    #     tp = tableWeak[i,i].astype(int)
    #     fn += table[i,0]
    #     for j in range(1, NUM_OF_CLASSESS):
    #         if i != j:
    #             fp += tableWeak[j,i].astype(int)
    #             fn += tableWeak[i,j].astype(int)
    #     if (tp + fp + fn != 0):
    #         IoU = float(tp) / float(tp + fp + fn)
    #     if (tp + fn != 0):
    #         recall = float(tp) / float(tp + fn)
    #     if (tp + fp != 0):
    #         precision = float(tp)/float(tp+fp)
    #     fout.write('%d\t%d\t%d\t%d\t%.6f\t%.6f\t%.6f\n' % (int(i), int(tp), int(fp), int(fn), float(IoU), float(recall), float(precision)))

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

def CalcResult(pre, gt):
    table = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
    height, width, channel = gt.shape
    for i in range(height):
        for j in range(width):
            # caculate accuracy
            table[gt[i,j,0]][pre[i,j,0]] += 1
    return table

# main
fg = open(G_PATH, "r")
fr = open(R_PATH, "r")

listName = os.listdir(PATH)
imgList = []
gtList = []
preList = []
costmapList = []
cntTable = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
cntTableWeak = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
# cntTableBaseline = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))

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
fout = open(NAME+'.result', 'w')


length = int(len(imgList) / 10)

for batch in range(10):
    fout.write('@@@@@@@@@@@@@@@@@@@@@@@@ Batch[ %d ] start_ts [ %s ] @@@@@@@@@@@@@@@@@@@@@@@\n' % (batch, imgList[batch*length].split('.')[0].split('_')[1]))
    cntTable = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
    cntTableWeak = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
    for i in range(batch*length, min(len(imgList),(batch+1)*length)):
        time_stamp = imgList[i].split('.')[0].split('_')[1]
        img = cv2.imread(DATA_PATH + "test/" + time_stamp + ".png", cv2.IMREAD_COLOR)
        gt  = cv2.imread(DATA_PATH + "test_gt/" + time_stamp + ".png", cv2.IMREAD_COLOR)
        wgt = cv2.imread(DATA_PATH + "eval_wgt/" + time_stamp + ".png", cv2.IMREAD_COLOR)
        wgt = wgtProcess(wgt)
        bgt = cv2.imread(DATA_PATH + "test_bgt/" + time_stamp + ".png", cv2.IMREAD_COLOR)
        videoImg = cv2.imread(DATA_PATH + "video/" + time_stamp + ".png", cv2.IMREAD_COLOR)
        pre = cv2.imread(PATH + preList[i], cv2.IMREAD_COLOR)

        new_gt = gt.copy()
        gt = preProcess(img, gt)
        wgt = preProcess(img, wgt)
        pre = preProcess(img, pre)

        costMap = img.copy()
        costMap, pre = readCost(fg, fr, costMap, pre)

        gtClone = gt.copy()
        none = gt.copy()
        grayPre = pre.copy()
        
        cntTable += CalcResult(pre, gt)
        cntTableWeak += CalcResult(pre, wgt)

        LabelColor(img, gt, False)
        LabelColor(img, pre, False)
        LabelColor(img, bgt, False)
        LabelColor(img, wgt, True)

        colorizedImg = cv2.applyColorMap(costMap, cv2.COLORMAP_HOT)
        colorizedImg = preProcess(img, colorizedImg)
        videoImg = cv2.resize(videoImg, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)
        # mergeImg0 = np.concatenate((img, wgt, pre), axis=1)
        # mergeImg1 = np.concatenate((videoImg, bgt, gt), axis=1)
        # mergeImg = np.concatenate((mergeImg0, mergeImg1), axis=0)
        mergeImg = np.concatenate((img, colorizedImg, pre, gt, wgt), axis=1)

        # write time stamp to video
        cv2.putText(mergeImg, time_stamp, (20,20), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

        videoWriter.write(mergeImg)
        print('Frame: %d / %d' % (i+1, len(imgList)))

        cv2.imwrite(PROJECT_PATH + "pre_" + time_stamp + ".png", grayPre)
        cv2.imwrite(PROJECT_PATH + "prob_" + time_stamp + ".png", costMap)

    OutputResult(cntTable, cntTableWeak)

videoWriter.release()
fg.close()
fr.close()
fout.close()