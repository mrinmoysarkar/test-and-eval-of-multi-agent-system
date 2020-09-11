#!/usr/bin/env python

import numpy as np
import os
import tensorflow as tf
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler, OneHotEncoder
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from tensorflow.keras.layers import Input, LSTM, Dense, Bidirectional, Activation
from tensorflow.keras.models import Model
from tensorflow.keras import optimizers
from tensorflow.keras import losses
from tensorflow.keras import metrics
from tensorflow.keras.utils import plot_model, to_categorical
from tensorflow.keras.models import load_model
from joblib import dump, load
 
gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
tf.config.experimental.set_visible_devices(devices=gpus[0], device_type='GPU')
tf.config.experimental.set_memory_growth(device=gpus[0], enable=True)

def batch_gen(X,y,time_step,batch_size=64):
    i = 0
    m = X.shape[0]
    while True:
        x_batch = []
        y_batch = []
        for j in range(batch_size):
            if i+j+time_step == m:
                i = -1
                break
            x_batch.append(X[i+j:i+j+time_step,:])
            y_batch.append(y[i+j+time_step,:])
        yield (np.array(x_batch), np.array(y_batch), i)
        i = i+1


if __name__ == "__main__":
    pwd = os.getcwd()
    database_path = pwd[:-6] + "flightData/filtered_data"
    database_file_names = []
    for root, dirs, files in os.walk(database_path):
        if not dirs:
            database_file_names = database_file_names + [os.path.join(root,file) for file in files]
        
        
    
    model = load_model('my_model.h5')
    time_step = 128
    print(model.summary())
    
    #test
    scaler = load('scaler.joblib')
    encoder = load('encoder.joblib')
    for i,file_name in enumerate(database_file_names):
        data = pd.read_csv(file_name)
        X_test = data.drop(columns=['label', 'time'])
        X_test = X_test.apply(lambda x: pd.to_numeric(x, errors='coerce')).dropna()
        # print(X_test.head())
        X_test = X_test.values
        Y_test = data['label'].values
        Y_test = Y_test.reshape((-1,1))
        # print(scaler)
        X_test = scaler.transform(X_test)
        # print("no error")
        
        j = 0
        for (x,y_true,flag) in batch_gen(X_test,Y_test,time_step,Y_test.shape[0]):
            y = model.predict(x)
            y_pred = np.zeros(y.shape)
            y = np.argmax(y,axis=1)
            y_pred[np.arange(y_pred.shape[0]),y] = 1
            y_pred = encoder.inverse_transform(y_pred)
            if j==0:
#                 print(accuracy_score(y_true,y_pred))
                print(file_name)
                print(classification_report(y_true,y_pred))
                print(confusion_matrix(y_true,y_pred))
                print("<<<<<<<<<<<<<<<<<<<<<<<------->>>>>>>>>>>>>>>>>>>>>")
                break
    
