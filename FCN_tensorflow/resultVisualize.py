import os
import cv2
import numpy as np

def Map(prob):
    ret = 0
    if prob > 0.7:
        ret = 0
    elif prob > 0.4:
        ret = 50
    elif prob > 0.2:
        ret = 100
    elif prob > 0.1:
        ret = 150
    elif prob > 0.05:
        ret = 180
    else:
        ret = 255
    return ret

NUM_OF_CLASSES = 0
fin = open('/media/gaobiao/SeagateBackupPlusDrive/Paper/MPR2018/result_seg.txt','r')
NUM_OF_CLASSES = int(fin.readline())
mat = [[0] * NUM_OF_CLASSES] * NUM_OF_CLASSES

for i in range(NUM_OF_CLASSES):
    s = fin.readline().replace(' \n','')
    s = s.split(' ')
    mat[i] = [int(x) for x in s]

fin.readline()
fin.readline()

tp = [0] * NUM_OF_CLASSES
fp = [0] * NUM_OF_CLASSES
fn = [0] * NUM_OF_CLASSES
IoU = [0] * NUM_OF_CLASSES
for i in range(1,NUM_OF_CLASSES):
    s = fin.readline().replace('\n','')
    s = s.split(' ')
    tp[i] = int(s[1])
    fp[i] = int(s[2])
    fn[i] = int(s[3])
    IoU[i] = float(s[4])

img = np.zeros(((NUM_OF_CLASSES-1) * 100, (NUM_OF_CLASSES-1) * 100), dtype=np.uint8)
p = np.zeros((NUM_OF_CLASSES, NUM_OF_CLASSES))
for i in range(1, NUM_OF_CLASSES):
    for j in range(1, NUM_OF_CLASSES):
        p[i][j] = float(mat[i][j]) / (sum(mat[i][1:])+0.00001)
        grey = Map(p[i][j])
        for x in range((i-1)*100, i*100):
            for y in range((j-1)*100, j*100):
                img[x][y] = grey
        cv2.putText(img, "%.0f"%(p[i][j]*100), (j*100-80,i*100-30), cv2.FONT_HERSHEY_COMPLEX, 1.5, 200, 2)

cv2.imshow("result", img)
cv2.waitKey(0)
cv2.imwrite("result.png", img)
fin.close()
