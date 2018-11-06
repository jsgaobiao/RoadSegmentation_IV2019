import numpy as np
import matplotlib.pyplot as plt

class PointSelector:
    def __init__(self, x, y, timestamp, fig):
        self.x = x
        self.y = y
        self.fig = fig
        self.timestamp = timestamp
        fig.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        if event.xdata is not None and event.ydata is not None:
            dx = np.array(self.x - event.xdata)
            dy = np.array(self.y - event.ydata)
            dist = dx**2 + dy**2
            idx = np.argmin(dist)
            print 'select id: %d\t%d'%(idx, self.timestamp[idx])

f0 = open("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/0.nav", 'r')
f1 = open("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/1.nav", 'r')
f2 = open("/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/2.nav", 'r')
data0 = np.genfromtxt(f0, delimiter='\t')
data1 = np.genfromtxt(f1, delimiter='\t')
data2 = np.genfromtxt(f2, delimiter='\t')
data = np.concatenate((data0, data1, data2), axis=0)

timestamp = [int(i) for i in data[:,0]]
x = data[:,4]
y = data[:,5]

fig = plt.figure()

plt.plot(x[:],y[:], 'r-')
# plt.plot(x[idx:end_idx],y[idx:end_idx], 'g-')

selector = PointSelector(x,y,timestamp, fig)

plt.show()
