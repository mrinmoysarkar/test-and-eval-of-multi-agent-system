#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import pandas as pd
import numpy as np
import os
from datetime import datetime
from std_msgs.msg import String
import rosgraph
from pickle import dump, load
from sklearn.preprocessing import MinMaxScaler, OneHotEncoder
from tensorflow.keras.layers import Input, LSTM, Dense, Bidirectional, Activation, GRU
from tensorflow.keras.models import Model
from tensorflow.keras import optimizers
from tensorflow.keras import losses
from tensorflow.keras import metrics
from tensorflow.keras.models import load_model
import tensorflow as tf
import matplotlib.pyplot as plt

class evaluator():
    def __init__(self,uavno):
        self.uavno = uavno
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/mavros/local_position/velocity_local', TwistStamped, self.local_velocity_cb)
        rospy.Subscriber('/uav'+str(self.uavno)+'/data_label', String, self.label_cb)
        self.currentPosition = None
        self.currentVelocity = None
        self.currentLabel = "Hold"
        self.stopThread = False

    def local_position_cb(self,msg):
        self.currentPosition = msg.pose

    def local_velocity_cb(self,msg):
        self.currentVelocity = msg.twist

    def label_cb(self,msg):
        self.currentLabel = msg.data

    def quaternion_to_euler(self,w, x, y, z):
        """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.where(np.abs(sinp) >= 1,
                         np.sign(sinp) * np.pi / 2,
                         np.arcsin(sinp))

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def loop(self, model, scaler, encoder, time_step, features=7):
        input_seq = np.zeros(shape=(time_step,7))
        rate = rospy.Rate(60.0)
        
        weights = {'Hold':0.05,
                   'Takeoff':0.05,
                   'Hover':0.05,
                   'Search':0.8,
                   'Loiter':0.2, 
                   'Obstacleavoid':0.2,
                   'Land':0.05}

        weights_1 = {1:0.05,
                   2:0.05,
                   3:0.05,
                   4:0.8,
                   5:0.2, 
                   6:0.2,
                   7:0.05}

        label_dic = {'Hold':1,
                   'Takeoff':2,
                   'Hover':3,
                   'Search':4,
                   'Loiter':5, 
                   'Obstacleavoid':6,
                   'Land':7}

        weight_sum = 0
        weight_acc_sum = 0
        true_label = []
        predicted_label = []
        
        try:
            while rosgraph.is_master_online():
                if self.currentPosition and self.currentVelocity:
                    
                    x = self.currentPosition.position.x
                    y = self.currentPosition.position.y
                    z = self.currentPosition.position.z

                    # quaternion = (self.currentPosition.orientation.x,
                    #     self.currentPosition.orientation.y,
                    #     self.currentPosition.orientation.z,
                    #     self.currentPosition.orientation.w)
                    # euler_angles = euler_from_quaternion(quaternion)

                    euler_angles = self.quaternion_to_euler(self.currentPosition.orientation.w,
                                                            self.currentPosition.orientation.x,
                                                            self.currentPosition.orientation.y,
                                                            self.currentPosition.orientation.z)

                    roll = euler_angles[0]
                    pitch = euler_angles[1]
                    yaw = euler_angles[2]

                    x_v = self.currentVelocity.linear.x
                    y_v = self.currentVelocity.linear.y
                    z_v = self.currentVelocity.linear.z

                    # roll_v.append(self.currentVelocity.angular.x)
                    # pitch_v.append(self.currentVelocity.angular.y)
                    # yaw_v.append(self.currentVelocity.angular.z)

                    label = self.currentLabel
                    if label not in weights:
                        # alpha = 1 - weight_acc_sum/(weight_sum+np.finfo(float).eps)
                        # print('alpha = ', alpha)
                        plt.plot(true_label, label='TL')
                        plt.plot(predicted_label, label='PL')
                        plt.legend()
                        plt.title('UAV'+str(self.uavno))
                        plt.show()
                        d_arr = np.diff(np.array(true_label))
                        # print("working well-1")
                        non_zero_indx = np.nonzero(d_arr)
                        # print("working well-2")
                        filter = np.ones(shape=(1,len(true_label)))
                        window_len = 40
                        # print(non_zero_indx)
                        # print(non_zero_indx[0].shape)
                        # print(non_zero_indx[0])
                        for idx in non_zero_indx[0]:
                            filter[0,idx-window_len:idx+window_len] = 0
                        
                        weight_sum = 0
                        weight_acc_sum = 0
                        # print(filter)
                        for i in range(filter.shape[1]):
                            if filter[0,i]==1:
                                weight_sum += weights_1[true_label[i]]
                                if true_label[i] != predicted_label[i]:
                                    weight_acc_sum += weights_1[true_label[i]]

                        alpha = 1 - weight_acc_sum/(weight_sum+np.finfo(float).eps)
                        print('*******************************************************************')
                        print('UAV '+str(self.uavno)+', alpha = ', alpha)
                        print('*******************************************************************')


                        return

                    input_seq[:-1] = input_seq[1:]
                    input_seq[-1] = [pitch, roll, x_v, y_v, yaw, z, z_v]
                    input = scaler.transform(input_seq)
                    input = np.array([input])
                    output = model.predict(input)
                    output = encoder.inverse_transform(output)

                    if output:
                        output = output[0][0]
                        true_label.append(label_dic[label])
                        predicted_label.append(label_dic[output])
                        # print(output)
                        # weight_sum += weights[label]
                        # if output != label:
                        #     weight_acc_sum += weights[label]


                rate.sleep()
        except Exception as e:
            print(e)


if __name__ == '__main__':
    gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
    tf.config.experimental.set_visible_devices(devices=gpus[0], device_type='GPU')
    tf.config.experimental.set_memory_growth(device=gpus[0], enable=True)

    rospy.init_node('evaluator_uav', anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    scaler_path = rospy.get_param('~scaler_path')
    encoder_path = rospy.get_param('~encoder_path')
    model_name = rospy.get_param('~model_name')
    time_step = rospy.get_param('~time_step') 


    eval = evaluator(uav_num)
    scaler = load(open(scaler_path, 'rb'))
    encoder = load(open(encoder_path, 'rb'))
    model = load_model(model_name)
    eval.loop(model, scaler, encoder, time_step)
