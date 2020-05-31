#!/usr/bin/env python

import cv2
import pandas as pd
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt
from collections import Counter
import pickle


def concat_dataFiles(filelist):
    if filelist:
        df1 = pd.read_csv(filelist[0], index_col=None)
        df1 = df1.dropna()
        # print(df1.shape)
        del filelist[0]
        for filename in filelist:
            df2 = pd.read_csv(filename, index_col=None)
            print(df2.shape)
            df2 = df2.dropna()
            df1 = df1.append(df2, ignore_index=True)
        # print("******************************")
        return df1
    return None


def one_hot_labels(labels,classes):
    n = labels.size
    newlabels = np.zeros([n,classes])
    for i in range(n):
        newlabels[i,int(labels[i])-1]=1
    return newlabels


def getNumericLabel(labels):
    numlabels = []
    for label in labels:
        numlabels.append(np.argmax(label)+1)
    return numlabels


def createStateDiagram(states=["states"],connection={"s1":["s2"]}):
    imDimension = 512
    border = 100
    canvas = np.ones((imDimension, imDimension, 3), np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    cx = imDimension//2
    cy = imDimension//2
    radius = (imDimension-border)/2
    noofPos = len(states)
    deltheta = 2*pi/noofPos
    thetas = [i*deltheta for i in range(noofPos)]
    statePositions = {}
    for theta,text in zip(thetas,states):
        x = cx + int(radius*cos(theta))
        y = cy + int(radius*sin(theta))
        statePositions[text]=[x,y]
        cv2.putText(canvas,text,(x,y),font,1,(0,0,0),2,cv2.LINE_AA)
    for state in states:
        if state in connection:
            xy1 = statePositions[state]
            connList = connection[state]
            for conn in connList:
                xy2 = statePositions[conn]
                cv2.line(canvas,tuple(xy1),tuple(xy2),(0,0,0),2)
                x = (xy2[0] - xy1[0])*2//3 + xy1[0]
                y = (xy2[1] - xy1[1])*2//3 + xy1[1]
                cv2.circle(canvas,(x,y),5,(0,0,0),2)


    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.imshow('image', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def getgraph(labels):
    previousi=-1
    connections = {}
    for i in labels:
        if previousi != i and previousi != -1:
            #this is a connection
            parent = str(int(previousi))
            child = str(int(i))
            if parent in connections:
                if child in connections[parent]:
                    pass
                else:
                    connections[parent].append(child)
            else:
                connections[parent] = [child]
        previousi=i
    return connections


def plot_data(fileNames):
    plt.figure(-1)
    n = len(fileNames)//3+1
    i = 1
    for filename in fileNames:
        data = pd.read_csv(filename, index_col=None)
        x_v = data['x_v']
        y_v = data['y_v']
        z_v = data['z_v']
        z = data['z']

        plt.subplot(n,3,i)
        i=i+1
        plt.plot(np.arange(len(x_v)), x_v, 'o', label="x_v")
        plt.plot(np.arange(len(y_v)), y_v, 'o', label="y_v")
        plt.plot(np.arange(len(z_v)), z_v, 'o', label="z_v")
        plt.plot(np.arange(len(z)), z, 'o', label="z")
        plt.legend()
        plt.xlabel("sample no.")
        plt.ylabel("altitude/velocities(in m or m/s)")
        values = filename.split('/')
        values = values[-1]
        values = values.split('_')
        title = values[3]
        plt.title(title)
    # plt.show(block=False)


def flatten_data(data, y, win_size):
    row,col = data.shape
    n = row//win_size
    new_data = []
    new_y = []
    y=y.flatten('F')
    for i in range(n):
        start = i*win_size
        end = (i+1)*win_size if (i+1)*win_size<row else row
        data_frame = data[start:end,:]
        data_frame = data_frame.flatten('F')
        new_data.append(data_frame)
        a = y[start:end]
        b = Counter(a)
        new_y.append(b.most_common(1)[0][0])
    return np.array(new_data), np.array(new_y)


def flatten_dataX(data, win_size):
    row,col = data.shape
    n = row//win_size
    new_data = []
    for i in range(n):
        start = i*win_size
        end = (i+1)*win_size if (i+1)*win_size<row else row
        data_frame = data[start:end,:]
        data_frame = data_frame.flatten('F')
        new_data.append(data_frame)
    return np.array(new_data)


def save_variable(file_name, var):
    filename = file_name + '.p'
    with open(filename, 'wb') as file_handler:
        pickle.dump(var, file_handler)


def load_variable(file_name):
    filename = file_name + '.p'
    with open(filename, 'rb') as file_handler:
        return pickle.load(file_handler)

