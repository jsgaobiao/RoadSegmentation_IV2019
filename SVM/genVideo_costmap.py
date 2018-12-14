import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import matplotlib.image as mpimg
import pdb

PATH = '/home/gaobiao/Documents/RoadSegmentation_IV2019/results/svm/vis/'
NAME = 'svm_2label'
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
    elif data[0] == 2:      # impassable
        data = [0, 0, 255]
    elif data[0] == 3:      # uncertain
        data = [255, 0, 0]
    return data

def preProcess(img, pre):
    for i in range(IMAGE_HEIGHT):
        for j in range(IMAGE_WIDTH):
            if (img[i, j, 0] == 0):
                pre[i,j,:] = [0, 0, 0]

def LabelColor(img, gt):
    height, width, channel = gt.shape
    for i in range(height):
        for j in range(width):
            # Label color of gt
            if (IS_WEAK_LABEL and gt[i,j,0] == 0):
                gt[i,j] = img[i,j]
            else:
                gt[i,j] = ColorMap(gt[i,j])

# main
listName = os.listdir(PATH)
costmapList = []
cntTable = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))
cntTableBaseline = np.zeros((NUM_OF_CLASSESS, NUM_OF_CLASSESS))

for name in listName:
    # costmap
    if name.split('/')[-1][0] == 'c':
        costmapList.append(name)

costmapList.sort()

videoWriter = cv2.VideoWriter(NAME+'.avi', cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'), 10, (IMAGE_WIDTH * 2, IMAGE_HEIGHT * 2), True)

for i in range(len(costmapList)):
    fName = costmapList[i].split('.')[0].split('_')[1]
    img = cv2.imread(DATA_PATH + "test/" + fName + ".png", cv2.IMREAD_COLOR)
    gt  = cv2.imread(DATA_PATH + "test_gt/" + fName + ".png", cv2.IMREAD_COLOR)
    videoImg = cv2.imread(DATA_PATH + "video/" + fName + ".png", cv2.IMREAD_COLOR)
    costMap = cv2.imread(PATH + costmapList[i], cv2.IMREAD_COLOR)

    preProcess(img, gt)

    LabelColor(img, gt)

    videoImg = cv2.resize(videoImg, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)
    for j in range(IMAGE_HEIGHT):
        for k in range(IMAGE_WIDTH):
            if img[j, k, 0] == 0:
                videoImg[j, k] = [0, 0, 0]
            elif costMap[j, k, 0] > 0:
                videoImg[j, k] = [0, 255, 0]
            elif costMap[j, k, 0] <= 0:
                videoImg[j, k] = [0, 0, 255]
            if costMap[j, k, 0] != 0:
                costMap[j, k] = 255 - costMap[j, k]
    colorizedImg = cv2.applyColorMap(costMap, cv2.COLORMAP_HOT)

    mergeImg0 = np.concatenate((img, colorizedImg), axis=1)
    mergeImg1 = np.concatenate((videoImg, gt), axis=1)
    mergeImg = np.concatenate((mergeImg0, mergeImg1), axis=0)
    videoWriter.write(mergeImg)
    print('Frame: %d / %d' % (i, len(costmapList)))

videoWriter.release()
