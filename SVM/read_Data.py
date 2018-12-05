__author__ = 'gaobiao'
import numpy as np
import os
import random
from six.moves import cPickle as pickle
from tensorflow.python.platform import gfile
import glob

import TensorflowUtils as utils

def read_dataset(data_dir):
    fTrain = open(data_dir + '/train.txt', 'r')
    fVal = open(data_dir + '/val.txt', 'r')
    fTest = open(data_dir + '/test_gt.txt', 'r')
    train_records = []
    valid_records = []
    test_records = []

    line = fTrain.readline()
    while (line):
        lineSp = line.split(' ')
        filename = lineSp[0].split('/')[-1].split('.')[0]
        record = {'image': lineSp[0], 'annotation': lineSp[1].replace('\n',''), 'filename': filename}
        train_records.append(record)
        line = fTrain.readline()

    line = fVal.readline()
    while (line):
        lineSp = line.split(' ')
        filename = lineSp[0].split('/')[-1].split('.')[0]
        record = {'image': lineSp[0], 'annotation': lineSp[1].replace('\n',''), 'filename': filename}
        valid_records.append(record)
        line = fVal.readline()

    line = fTest.readline()
    while (line):
        lineSp = line.split(' ')
        filename = lineSp[0].split('/')[-1].split('.')[0]
        record = {'image': lineSp[0], 'annotation': lineSp[1].replace('\n',''), 'filename': filename}
        test_records.append(record)
        line = fTest.readline()

    fTrain.close()
    fVal.close()
    fTest.close()
    return train_records, valid_records, test_records
