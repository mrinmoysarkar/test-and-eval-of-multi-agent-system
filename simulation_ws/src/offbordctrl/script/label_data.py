#!/usr/bin/env python


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import threading


lastIndx = 0
labels = None
indx = 0
data = None

def getInput():
    global lastIndx,indx,labels,data,fileNmae,extension
    while True:
        key = input("Label: ")
        if key=='q':
            # labels[lastIndx:] = 5
            break
        elif key=='s':
            data["label"] = labels
            data.to_csv(fileNmae+"_labeled"+extension, index=None)
            print(data.head())
        else:
            print(key)
            print(lastIndx, indx)
            key = int(key)
            labels[lastIndx:indx] = key
            lastIndx = indx


def onclick(event):
    global indx
    # print(event)
    # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
    #       ('double' if event.dblclick else 'single', event.button,
    #        event.x, event.y, event.xdata, event.ydata))
    indx = int(event.xdata)
    print(indx)


if __name__=="__main__":

    threading.Thread(target=getInput).start()

    fileNmae = "../flightData/flight_path_data_uav0_20190905-174326"
    extension = ".csv"

    data = pd.read_csv(fileNmae+extension,index_col=None)
    # data.to_csv("../flightData/flight_path_data_uav0_labled.csv",index=None)
    # data.plot()
    x_v = data['x_v']
    y_v = data['y_v']
    z_v = data['z_v']
    z = data['z']
    labels = np.ones(len(x_v))
    # print(labels.shape)
    # print(data.head())

    fig, ax = plt.subplots()

    ax.plot(np.arange(len(x_v)), x_v,'o',label="x_v")
    ax.plot(np.arange(len(y_v)), y_v, 'o',label="y_v")
    ax.plot(np.arange(len(z_v)), z_v, 'o',label="z_v")
    ax.plot(np.arange(len(z)), z, 'o',label="z")
    plt.legend()
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    fig.canvas.mpl_disconnect(cid)
