import matplotlib.pyplot as plt
import re
import sys

f = open(sys.argv[1], 'r')

trainIteration = []
trainLoss = []
testIteration = []
testLoss = []
valIteration = []
valLoss = []

for line in f.readlines():
    if line[0] == '*':
        continue
    line = line.replace('\n','')
    txt = re.split(r":|,", line)
    # Train loss
    if (txt[0][0] == 'S'):
        trainIteration.append(int(txt[1]))
        trainLoss.append(float(txt[-1]))
    # Validation_loss
    elif (txt[2][-6] == 'n'):
        valIteration.append(trainIteration[-1])
        valLoss.append(float(txt[-1]))
    # Test loss
    else:
        testIteration.append(trainIteration[-1])
        testLoss.append(float(txt[-1]))

plt.clf()
plt.plot(trainIteration, trainLoss, lw=1, color='navy', label='train loss')
plt.plot(valIteration, valLoss, lw=1, color='green', label='val loss')
plt.plot(testIteration, testLoss, lw=1, color='red', label='test loss')
plt.xlabel('iteration')
plt.ylabel('loss')
plt.xlim([0.0, 100000.0])
plt.ylim([0.0, 3.05])
plt.title('Loss')
plt.show()
