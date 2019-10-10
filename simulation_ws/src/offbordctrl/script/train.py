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



def one_hot_labels(labels,classes):
    n = labels.size
    newlabels = np.zeros([n,classes])
    for i in range(n):
        newlabels[i,int(labels[i])-1]=1
    return newlabels



if __name__=="__main__":
    print(device_lib.list_local_devices())
    fileNames = ["flight_path_data_uav0_20190905-173452_labeled",
                 "flight_path_data_uav0_20190905-173802_labeled",
                 "flight_path_data_uav0_20190905-174145_labeled",
                 "flight_path_data_uav0_20190905-174326_labeled",
                 "flight_path_data_uav0_labeled"]
    fileNmae = "../flightData/"+fileNames[1]+".csv"
    tf.enable_eager_execution()
    # print("Eager execution:", tf.executing_eagerly())
    data = pd.read_csv(fileNmae, index_col=None)
    data = data.dropna()
    # print(data.columns.values)

    scaler = MinMaxScaler()
    X = data.iloc[:, [5,7,10,11]].values
    # print(X)
    X = scaler.fit_transform(X)
    y = data['label'].values
    labels = one_hot_labels(y, 5)
    # y_train = tf.keras.utils.to_categorical([[1],[2]], num_classes=3)
    # print(y_train)

    # config = tf.ConfigProto(device_count={'GPU': 1, 'CPU': 56})
    # sess = tf.Session(config=config)
    sess = tf.Session()
    tf.keras.backend.set_session(sess)
    logdir = "./temp/" + datetime.now().strftime("%Y%m%d-%H%M%S")
    tensorboard = TensorBoard(log_dir=logdir)
    with tf.name_scope('summaries'):
        model = tf.keras.Sequential()
        model.add(layers.Dense(64, activation='relu'))
        # model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(5, activation='softmax'))
        model.compile(optimizer=tf.train.RMSPropOptimizer(0.001),
                      loss=tf.keras.losses.categorical_crossentropy,
                      metrics=[tf.keras.metrics.categorical_accuracy])
        model.fit(X, labels, epochs=300, batch_size=128, callbacks=[tensorboard])

    for i in range(5):
        test_data = pd.read_csv("../flightData/"+fileNames[i]+".csv", index_col=None)
        test_data = test_data.dropna()
        testX = test_data.iloc[:, [5,7,10,11]].values
        testX = scaler.transform(testX)
        testy = test_data['label'].values
        testlabels = one_hot_labels(testy, 5)
        print("evaluate:")
        loss, acc = model.evaluate(testX, testlabels, batch_size=128)
        print(loss,acc)