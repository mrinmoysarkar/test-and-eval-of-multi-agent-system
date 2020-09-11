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
import os
from os import listdir
from os.path import isfile, join
from helper_functions import one_hot_labels, getNumericLabel, createStateDiagram, getgraph, concat_dataFiles, plot_data, flatten_data, save_variable
import pickle
from sklearn import tree

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'







if __name__=="__main__":
    # print(device_lib.list_local_devices())
    # print(tf.__version__)
    # tf.get_logger().setLevel('WARNING')
    # tf.autograph.set_verbosity(2)
    # fileNames = ["flight_path_data_uav0_20190905-173452_labeled",
    #              "flight_path_data_uav0_20190905-173802_labeled",
    #              "flight_path_data_uav0_20190905-174145_labeled",
    #              "flight_path_data_uav0_20190905-174326_labeled",
    #              "flight_path_data_uav0_labeled"]
    # fileNmae = "../flightData/"+fileNames[1]+".csv"
    # fileLists = ["../flightData/"+filename+".csv" for filename in fileNames]

    databasePath = "../flightData/lebeled_data/"
    fileNames = [databasePath+f for f in listdir(databasePath) if isfile(join(databasePath, f))]
    plot_data(fileNames)
    data = concat_dataFiles(fileNames)
    # print(data.shape)
    # tf.enable_eager_execution()
    # print("Eager execution:", tf.executing_eagerly())
    # if data is None:
    #     data = pd.read_csv(fileNmae, index_col=None)
    #     data = data.dropna()
    # print(data.columns.values)
    win_size = 1
    feature_size = 4
    scaler = MinMaxScaler()
    X = data.iloc[:, [5,7,10,11]].values
    print(type(X))
    X = scaler.fit_transform(X)
    save_variable("scaler", scaler)
    y = data['label'].values
    # labels = one_hot_labels(y, 5)
    flat_X, flat_y = flatten_data(X, y, win_size)
    X = flat_X
    y = flat_y
    labels = one_hot_labels(y, 5)
    # y_train = tf.keras.utils.to_categorical([[1],[2]], num_classes=3)
    # print(y_train)

    # config = tf.ConfigProto(device_count={'GPU': 1, 'CPU': 56})
    # sess = tf.Session(config=config)
    # sess = tf.Session()
    # tf.keras.backend.set_session(sess)
    logdir = "./temp/" + datetime.now().strftime("%Y%m%d-%H%M%S")
    tensorboard = TensorBoard(log_dir=logdir)
    with tf.name_scope('summaries'):
        clf = tree.DecisionTreeClassifier()
        clf = clf.fit(X, y)
        print(clf.score(X,y))
        save_variable("clf", clf)

        # # model = tf.keras.Sequential()
        # # model.add(layers.Dense(64, activation='relu'))
        # # # model.add(layers.Dense(64, activation='relu'))
        # # model.add(layers.Dense(64, activation='relu'))
        # # model.add(layers.Dense(5, activation='softmax'))
        # inputs = tf.keras.Input(shape=(win_size*feature_size,), name='features')
        # x = layers.Dense(128, activation='relu', name='dense_1')(inputs)
        # x = layers.Dense(128, activation='relu', name='dense_2')(x)
        # x = layers.Dense(256, activation='relu', name='dense_3')(x)
        # outputs = layers.Dense(5, activation='softmax', name='predictions')(x)
        #
        # model = tf.keras.Model(inputs=inputs, outputs=outputs, name='3_layer_mlp')
        # print(model.summary())
        #
        # model.compile(optimizer=tf.keras.optimizers.RMSprop(),
        #               loss=tf.keras.losses.categorical_crossentropy,
        #               metrics=[tf.keras.metrics.categorical_accuracy])
        # model.fit(X, labels, epochs=500, batch_size=64, callbacks=[tensorboard])

    # for i in range(5):
    #     test_data = pd.read_csv("../flightData/"+fileNames[i]+".csv", index_col=None)
    #     test_data = test_data.dropna()
    #     testX = test_data.iloc[:, [5,7,10,11]].values
    #     testX = scaler.transform(testX)
    #     testy = test_data['label'].values
    #     testlabels = one_hot_labels(testy, 5)
    #     print("evaluate:")
    #     loss, acc = model.evaluate(testX, testlabels, batch_size=128)
    #     labels = model.predict(testX)
    #     labels = getNumericLabel(labels)
    #     # print(labels)
    #     plt.subplot(2,3,i+1)
    #     plt.plot(labels,label="predicted")
    #     plt.plot(testy,label="true")
    #     plt.legend()
    #     print(loss,acc)
    #     connection = getgraph(labels)
    #     print(connection)
    #     states = [int(i) for i in connection.keys()]
    #     states.sort()
    #     states = [str(i) for i in states]
    #     # states = (connection.keys()).sort()
    #     createStateDiagram(states=states, connection=connection)
    # plt.show()

    # createStateDiagram(states=connection.keys(),connection=connection)
    # model.save('trainedModel.h5')
    # filename = 'scalar.p'
    # with open(filename, 'wb') as filehandler:
    #     pickle.dump(scaler, filehandler)

    plt.show()