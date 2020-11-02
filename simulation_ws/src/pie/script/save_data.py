#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import pandas as pd
import numpy as np
import os
from datetime import datetime
from std_msgs.msg import String
import rosgraph


class datalogger():
    def __init__(self,uavno,data_log_path):
        self.uavno = uavno
        self.data_log_path = data_log_path
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

    def loop(self):

        rate = rospy.Rate(60.0)
        t = []
        x = []
        y = []
        z = []
        roll = []
        pitch = []
        yaw = []
        x_v = []
        y_v = []
        z_v = []
        roll_v = []
        pitch_v = []
        yaw_v = []
        label = []
        try:
            while rosgraph.is_master_online():
                if self.currentLabel == 'Finished':
                    break
                if self.currentPosition and self.currentVelocity:
                    t.append(time.time())
                    x.append(self.currentPosition.position.x)
                    y.append(self.currentPosition.position.y)
                    z.append(self.currentPosition.position.z)
                    quaternion = (self.currentPosition.orientation.x,
                        self.currentPosition.orientation.y,
                        self.currentPosition.orientation.z,
                        self.currentPosition.orientation.w)
                    euler_angles = euler_from_quaternion(quaternion)
                    roll.append(euler_angles[0])
                    pitch.append(euler_angles[1])
                    yaw.append(euler_angles[2])
                    x_v.append(self.currentVelocity.linear.x)
                    y_v.append(self.currentVelocity.linear.y)
                    z_v.append(self.currentVelocity.linear.z)
                    roll_v.append(self.currentVelocity.angular.x)
                    pitch_v.append(self.currentVelocity.angular.y)
                    yaw_v.append(self.currentVelocity.angular.z)
                    label.append(self.currentLabel)

                rate.sleep()
        except Exception as e:
            print(e)

        d = {'time':t,'x':x, 'y':y, 'z':z,
             'roll':roll,'pitch':pitch,'yaw':yaw,
             'x_v':x_v,'y_v':y_v,'z_v':z_v,
             'roll_v':roll_v,'pitch_v':pitch_v,'yaw_v':yaw_v,
             'label':label}
        df = pd.DataFrame(data=d)
        fileName = self.data_log_path + "flight_path_data_uav" + str(self.uavno) + "_" + datetime.now().strftime("%Y%m%d-%H%M%S") + "_labeled.csv"
        df.to_csv(fileName,index=False)

        self.stopThread = False


if __name__ == '__main__':
 
    # dirpath = os.getcwd()
    # print(dirpath)
    # noofUav = 4
    rospy.init_node('data_logger_uav', anonymous=True)
    uav_num = rospy.get_param('~uav_num')
    data_log_path = rospy.get_param('~data_log_path')
    dlogger = datalogger(uav_num,data_log_path)
    dlogger.loop()

    # uavs = []
    # for i in range(noofUav):
    #     uavs.append(datalogger(i))
    #     threading.Thread(target=uavs[i].loop).start()
    # rospy.spin()
    # time.sleep(5)
    # while not rospy.is_shutdown():
    #     cmd = raw_input("command: \"q\" for exit the main loop: ")
    #     if cmd=='q':
    #         for i in range(noofUav):
    #             uavs[i].stopThread = True
    #         break
    
    # counter = noofUav
    # while counter != 0:
    #     counter = noofUav
    #     for i in range(noofUav):
    #             if not uavs[i].stopThread:
    #                 counter -= 1
    #     rospy.sleep(1)
