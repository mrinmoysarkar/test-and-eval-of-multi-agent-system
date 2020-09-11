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
    database_path = pwd[:-6] + "flightData/filtered_data/scenario5"
    database_file_names = []
    for root, dirs, files in os.walk(database_path):
        database_file_names = [os.path.join(root,file) for file in files]
        
        
    load_model = False
    if load_model:
        model = load_model('my_model.h5')
    else:
        #create model
        num_feature = 12
        time_step = 128
        num_class = 7
        x_in = Input(shape=(time_step,num_feature),dtype="float32")
        forward_layer = LSTM(units=128, return_sequences=True, activation='tanh')
        backward_layer = LSTM(units=128, return_sequences=True, activation='tanh', go_backwards=True)
        bid_layer = Bidirectional(layer=forward_layer,backward_layer=backward_layer, merge_mode='concat')(x_in)
        y = Dense(units=num_class,activation='softmax')(bid_layer[:,-1,:])
        model = Model(inputs=x_in,outputs=y)
        optimizer = optimizers.Adam(learning_rate=0.001, beta_1=0.9, beta_2=0.999, amsgrad=False)
        loss = losses.categorical_crossentropy
        model.compile(optimizer=optimizer, loss=loss, metrics=['CategoricalAccuracy'])
        
    print(model.summary())
    
    #train
    scaler = MinMaxScaler()
    encoder = OneHotEncoder()
    for i,file in enumerate(database_file_names):
        data = pd.read_csv(file)
        X_train = data.drop(columns=['label', 'time'])
        X_train = X_train.values
        Y_train = data['label'].values
        Y_train = Y_train.reshape((-1,1))
        if i == 0:
            X_train = scaler.fit_transform(X_train)
            Y_train = encoder.fit_transform(Y_train).toarray()
            dump(scaler, 'scaler.joblib')
            dump(encoder, 'encoder.joblib')
        else:
            X_train = scaler.transform(X_train)
            Y_train = encoder.transform(Y_train).toarray()

        epoch = 0
        j = 0
        for (x,y,flag) in batch_gen(X_train,Y_train,time_step,512):
            j = j+1
            #model.fit(x, y, epochs=1, verbose=2, shuffle=False)
            hist = model.train_on_batch(x,y)
            if flag==-1:
                epoch += 1
                print("epoch complete {}".format(epoch))
                model.save('my_model.h5')
            if j%10000==0:
                print(hist)
            if epoch>10:
                break
    model.save('my_model.h5')
    print("train complete!!!!")
