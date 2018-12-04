import os

IMAGE_PATH = "/home/gaobiao/Documents/RoadSegmentation_IV2019/data/guilin_for_annotation/"
OUTPUT_PATH = "/home/gaobiao/Documents/RoadSegmentation_IV2019/data/annotated_gt/"
FILE_LIST_PATH = "/home/gaobiao/Documents/RoadSegmentation_IV2019/data/guilin_for_annotation.txt"

fout = open(FILE_LIST_PATH, "w")

data = []
label = []
wlabel = []
video = []
listName = os.listdir(IMAGE_PATH)

for fileName in listName:
    if fileName[-7:-4] == 'img':    # input image
        data.append(fileName)
    elif fileName[-7:-4] == 'wgt':   # weak ground truth
        wlabel.append(fileName)
    elif fileName[-6:-4] == 'gt':   # ground truth
        label.append(fileName)
    elif fileName[-9:-4] == 'video':   # ground truth
        video.append(fileName)

data.sort()
label.sort()
wlabel.sort()
video.sort()

for i in range(len(data)):
    fout.write(data[i][0:-8] + '\n')

fout.close()
