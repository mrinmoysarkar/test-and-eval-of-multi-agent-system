#!/usr/bin/env python

from datetime import datetime
import tensorflow as tf
import pandas as pd
import numpy as np
from sklearn.preprocessing import MultiLabelBinarizer, MinMaxScaler, Binarizer, OneHotEncoder
import matplotlib.pyplot as plt
from tensorflow.keras import layers
from tensorflow.python.keras.callbacks import TensorBoard
from time import time
from tensorflow.python.client import device_lib
import cv2
from math import cos, sin, pi
from os import listdir
from os.path import isfile, join
from helper_functions import one_hot_labels, getNumericLabel, createStateDiagram, getgraph, plot_data, flatten_dataX, load_variable
import pickle
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


if __name__=="__main__":
    model = tf.keras.models.load_model("trainedModel.h5")
    # filename = 'scalar.p'
    # with open(filename, 'rb') as filehandler:
    #     scaler = pickle.load(filehandler)
    # scaler = MinMaxScaler()
    clf = load_variable("clf")
    scaler = load_variable("scaler")
    win_size = 1
    databasePath = "../flightData/new_test/"
    fileNames = [databasePath+f for f in listdir(databasePath) if isfile(join(databasePath, f))]
    plot_data(fileNames)
    plotindx = 1
    plt.figure(1)
    for filename in fileNames:
        # if -1 == filename.find("labeled"):
        test_data = pd.read_csv(filename, index_col=None)
        test_data = test_data.dropna()
        testX = test_data.iloc[:, [5, 7, 10, 11]].values
        testX = scaler.transform(testX)
        testX = flatten_dataX(testX, win_size)
        # testy = test_data['label'].values
        # testlabels = one_hot_labels(testy, 5)
        # print("evaluate:")
        # loss, acc = model.evaluate(testX, testlabels, batch_size=128)
        # labels = model.predict(testX)
        # labels = getNumericLabel(labels)
        labels = clf.predict(testX)
        # print(labels)
        # print(labels)
        plt.subplot(4, 3, plotindx)
        plt.plot(labels, label="predicted")
        plt.legend()
        plt.xlabel("sample no.")
        plt.ylabel("states")
        plotindx += 1
        connection = getgraph(labels)
        print(connection)
        states = [int(i) for i in connection.keys()]
        states.sort()
        states = [str(i) for i in states]
        # states = (connection.keys()).sort()
        createStateDiagram(states=states, connection=connection)
    plt.show()