'''
    Split the data and labels into 'train' 'test' 'val'.
    Generate file list into 'train.txt' 'test.txt' 'val.txt'
'''

from sklearn.cross_validation import train_test_split
import os
import shutil

PATH                = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/data_weak_4L/'       # end with '/'
IMAGE_PATH          = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/guilin_weak/'     # end with '/'
TRAIN_FILE          = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/train.txt'
TEST_FILE           = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/test.txt'
TEST_FILE_WITH_GT   = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/test_gt.txt'
VAL_FILE            = '/home/gaobiao/Documents/RoadSegmentation_IV2019/data/val.txt'
f_train = open(TRAIN_FILE, 'w')
f_test = open(TEST_FILE, 'w')
f_val = open(VAL_FILE, 'w')
f_test_gt = open(TEST_FILE_WITH_GT, 'w')

data = []
label = []
video = []
listName = os.listdir(IMAGE_PATH)

for fileName in listName:
    if fileName[-7:-4] == 'img':    # input image
        data.append(fileName)
    elif fileName[-6:-4] == 'gt':   # ground truth
        label.append(fileName)
    elif fileName[-9:-4] == 'video':   # ground truth
        video.append(fileName)

data.sort()
label.sort()
video.sort()

p = int(len(data) * 0.6)
pp = int(len(data) * 0.7)
X_train = data[0:p]
X_val = data[p:pp]
X_test = data[pp:]

Y_train = label[0:p]
Y_val = label[p:pp]
Y_test = label[pp:]

V_train = video[0:p]
V_val = video[p:pp]
V_test = video[pp:]

# X_trainval, X_test, Y_trainval, Y_test = train_test_split(data, label, test_size=0.2)
# X_train, X_val, Y_train, Y_val = train_test_split(X_trainval, Y_trainval, test_size=0.2)

os.mkdir(PATH + 'train')
os.mkdir(PATH + 'train_gt')
os.mkdir(PATH + 'test')
os.mkdir(PATH + 'test_gt')
os.mkdir(PATH + 'val')
os.mkdir(PATH + 'val_gt')
os.mkdir(PATH + 'video')
for i in range(len(X_train)):
    shutil.copy(IMAGE_PATH + X_train[i], PATH + 'train/' + X_train[i][0:-8] + '.png')
    shutil.copy(IMAGE_PATH + Y_train[i], PATH + 'train_gt/' + Y_train[i][0:-7] + '.png')
    shutil.copy(IMAGE_PATH + V_train[i], PATH + 'video/' + V_train[i][0:-10] + '.png')
    f_train.write(PATH + 'train/' + X_train[i][0:-8] + '.png' + ' ' + PATH + 'train_gt/' + Y_train[i][0:-7] + '.png' + '\n')
for i in range(len(X_test)):
    shutil.copy(IMAGE_PATH + X_test[i], PATH + 'test/' + X_test[i][0:-8] + '.png')
    shutil.copy(IMAGE_PATH + Y_test[i], PATH + 'test_gt/' + Y_test[i][0:-7] + '.png')
    shutil.copy(IMAGE_PATH + V_test[i], PATH + 'video/' + V_test[i][0:-10] + '.png')
    f_test_gt.write(PATH + 'test/' + X_test[i][0:-8] + '.png' + ' ' + PATH + 'test_gt/' + Y_test[i][0:-7] + '.png' + '\n')
    f_test.write(PATH + 'test/' + X_test[i][0:-8] + '.png' + '\n')
for i in range(len(X_val)):
    shutil.copy(IMAGE_PATH + X_val[i], PATH + 'val/' + X_val[i][0:-8] + '.png')
    shutil.copy(IMAGE_PATH + Y_val[i], PATH + 'val_gt/' + Y_val[i][0:-7] + '.png')
    shutil.copy(IMAGE_PATH + V_val[i], PATH + 'video/' + V_val[i][0:-10] + '.png')
    f_val.write(PATH + 'val/' + X_val[i][0:-8] + '.png' + ' ' + PATH + 'val_gt/' + Y_val[i][0:-7] + '.png' + '\n')

f_train.close()
f_test.close()
f_val.close()
f_test_gt.close()
