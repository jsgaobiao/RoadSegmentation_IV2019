# import the necessary packages
import os
import cv2

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
refColor = 0
newColor = 0
NUM_OF_CLASSESS = 7
HEIGHT = 160
WIDTH  = 1080

def ColorMap(data, _img):
    if data == 0:
        data = [0,0,0]
    elif data == 1:        # people
        data = [0,0,255]
    elif data == 2:      # car
        data = [255,0,0]
    elif data == 3:      # tree
        data = [0,255,0]
    elif data == 4:      # sign
        data = [255,0,255]
    elif data == 5:      # building
        data = [255,255,0]
#    elif data == 6:      # cyclist
#        data = [0,128,255]
#    elif data == 7:      # stop bicycle
#        data = [128,64,0]
    elif data == 6:      # road
        data = [208,149,117]
    else:
        data = [_img[0], _img[0], _img[0]]
    return data

def MergeImage(_img, _gt):
    height, width, channel = _gt.shape
    for i in range(height):
        for j in range(width):
            # Label color of gt
            _img[i,j] = ColorMap(_gt[i,j], _img[i,j])
    return _img

def replaceColor(gt, pt, color, newColor, newImg, mergeImg):
    for i in range(int(pt[0][1]), min(HEIGHT-1,int(pt[1][1]))):
        for j in range(int(pt[0][0]), min(WIDTH-1,int(pt[1][0]))):
            if (gt[i,j] == color).all():
                gt[i,j] = newColor
                newImg[i,j] = ColorMap(gt[i,j], newImg[i,j])
                mergeImg[i,j] = ColorMap(gt[i,j], mergeImg[i,j])
    return gt, newImg, mergeImg

def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, refColor, newColor, image, gt, mergeImg, newImg, curLog, strT
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPt.append((x, y))
        # draw a rectangle around the region of interestuntil the 'q' key is pressed
        cv2.rectangle(newImg, refPt[0], refPt[1], (0, 255, 0), 2)
        print("rect: " + str(refPt))
    elif event == cv2.EVENT_MBUTTONDOWN:
        refColor = gt[y, x].copy()
        print("refColor: " + str(refColor))
        gt, newImg, mergeImg = replaceColor(gt, refPt, refColor, newColor, newImg, mergeImg)
        curLog.append([strT, refPt[0][0], refPt[0][1], refPt[1][0], refPt[1][1], refColor, newColor])

def implement_log(x0, y0, x1, y1, ref_color, new_color):
    # grab references to the global variables
    global refPt, refColor, newColor, image, gt, mergeImg, newImg
    ref_point = [(x0, y0),(x1, y1)]
    gt, newImg, mergeImg = replaceColor(gt, ref_point, ref_color, new_color, newImg, mergeImg)
    return gt, newImg, mergeImg

INPUT_LOG   = '/home/gaobiao/Documents/SemanticSeg_IV2019/data/in.log'
OUTPUT_LOG  = '/home/gaobiao/Documents/SemanticSeg_IV2019/data/out.log'
f_in = open(INPUT_LOG, 'r')
f_out = open(OUTPUT_LOG, 'w')
logs = f_in.readlines()

PATH = '/home/gaobiao/Documents/SemanticSeg_IV2019/data/images/'     # end with '/'
allImgList = os.listdir(PATH)
imgList = []
gtList = []
mergeImgList = []
curLog = []
for i in allImgList:
    if i.split('_')[1][0] == 'g':
        gtList.append(i)
    elif i.split('_')[1][0] == 'i':
        imgList.append(i)
    elif i.split('_')[1][0] == 'm':
        mergeImgList.append(i)

gtList.sort()
imgList.sort()
mergeImgList.sort()

i = 0
print('imgList len:', len(imgList))
idx = 0
print('log length:', len(logs))

while i < len(imgList):
    imgFileName = PATH + imgList[i]
    gtFileName = PATH + gtList[i]
    mergeImgFileName = PATH + mergeImgList[i]

    # load the image, clone it, and setup the mouse callback function
    image = cv2.imread(imgFileName)
    gt = cv2.imread(gtFileName, cv2.IMREAD_GRAYSCALE)
    mergeImg = cv2.imread(mergeImgFileName)
    image = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)
    gt = cv2.resize(gt, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)
    mergeImg = cv2.resize(mergeImg, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)

    newImg = mergeImg.copy()
    gtClone = gt.copy()
    mergeImgClone = mergeImg.copy()
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", click_and_crop)

    #implement log
    curLog = []
    strT = imgList[i].split('_')[0]
    print(i, strT)
    idx = 0
    while (idx < len(logs)):
        strs = logs[idx].split(' ')
        if (strs[0] != strT):
            idx += 1
            continue
        if (strs[0] == strT):
            gt, newImg, mergeImg = implement_log(int(strs[1]), int(strs[2]), int(strs[3]), int(strs[4]), int(strs[5]), int(strs[6]))
            curLog.append([int(strs[0]), int(strs[1]), int(strs[2]), int(strs[3]), int(strs[4]), int(strs[5]), int(strs[6])])
            idx += 1

    # keep looping
    while True:
        # display the image and wait for a keypress
        # cv2.imshow("image", Merge/Image(image, gt))
        cv2.imshow("image", newImg)
        cv2.imshow("input", image)
        key = cv2.waitKey(1) & 0xFF
        # if the 'r' key is pressed, reset the cropping region
        if key == ord("r"):
            gt = gtClone.copy()
            mergeImg = mergeImgClone.copy()
            newImg = mergeImg.copy()
            curLog = []
            cv2.imshow("image", newImg)
            # cv2.imshow("image", MergeImage(image, gt))
        # if the 'c' key is pressed, break from the loop
        elif key == ord("d"):
            break
        elif key == ord("a"):
            if i > 0:
                i -= 2
            break
        elif key == ord(" "):
            image = cv2.resize(image, (1080, 32), interpolation=cv2.INTER_NEAREST)
            gt = cv2.resize(gt, (1080, 32), interpolation=cv2.INTER_NEAREST)
            mergeImg = cv2.resize(mergeImg, (1080, 32), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(imgFileName, image)
            cv2.imwrite(gtFileName, gt)
            cv2.imwrite(mergeImgFileName, mergeImg)
            for logLine in curLog:
                for s in logLine:
                    f_out.write(str(s))
                    f_out.write(' ')
                f_out.write('\n')
            break
        elif (key >= ord("0")) and (key <= ord("8")):
            newColor = key - ord("0")
            print("newColor : %d\n" % newColor)
        elif key == ord("q"):
            exit(0)
    i += 1

# close all open windows
cv2.destroyAllWindows()
f_in.close()
f_out.close()
