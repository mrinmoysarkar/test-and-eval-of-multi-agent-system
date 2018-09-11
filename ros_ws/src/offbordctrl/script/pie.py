#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from sklearn import tree  
from sklearn.externals import joblib

# style.use('fivethirtyeight')
# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)

buffersize = 20
alt = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],dtype = np.float32)
xVel = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],dtype = np.float32)
yVel = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],dtype = np.float32)
zVel = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],dtype = np.float32)
currentState = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],dtype = np.float32)
Statedic={'Hold':1.0,'Takeoff':2.0,'Hover':3.0,'Search':4.0,'Land':5.0}

# def animate(i):
#     global x,y
#     ax1.clear()
#     ax1.plot(x,y)


xPose = 0.0
yPose = 0.0
zPose = 0.0
quatx = 0.0
quaty = 0.0
quatz = 0.0

vel_xLineTwist = 0.0
vel_yLineTwist = 0.0
vel_zLineTwist = 0.0
vel_xAngTwist = 0.0
vel_yAngTwist = 0.0
vel_zAngTwist = 0.0


def pose_cb(msg):
    global xPose
    global yPose 
    global zPose 
    global quatx 
    global quaty 
    global quatz 
    xPose = msg.pose.position.x
    yPose = msg.pose.position.y
    zPose = msg.pose.position.z
    quatx= msg.pose.orientation.x
    quaty= msg.pose.orientation.y
    quatz= msg.pose.orientation.z
    quatw= msg.pose.orientation.w


def velocity_cb(msg):
    global vel_xLineTwist
    global vel_yLineTwist
    global vel_zLineTwist
    global vel_xAngTwist
    global vel_yAngTwist
    global vel_zAngTwist
    vel_xLineTwist = msg.twist.linear.x
    vel_yLineTwist = msg.twist.linear.y
    vel_zLineTwist = msg.twist.linear.z
    vel_xAngTwist = msg.twist.angular.x
    vel_yAngTwist = msg.twist.angular.y
    vel_zAngTwist = msg.twist.angular.z



def pie():
    clf = joblib.load('/home/mrinmoy/ros-intel-uav-rpeo/ros_ws/src/offbordctrl/script/trainedModel.pkl') 
    rospy.init_node('PIE', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb, queue_size=100)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, velocity_cb, queue_size=100)
    rate = rospy.Rate(20) # 10hz
    #rospy.spin()
    
    #ani  = animation.FuncAnimation(fig, animate, interval=1000)
    
    fig = plt.gcf()
    fig.show()
    fig.canvas.draw()
    plt.axis([0,20,-2,2])
    i = 0
    while not rospy.is_shutdown():
        #print('altitude: ', zPose, ' xVel: ', vel_xLineTwist, ' yVel: ', vel_yLineTwist, ' zVel: ', vel_zLineTwist)
        alt[:-1] = alt[1:]
        alt[-1] = zPose

        xVel[:-1] = xVel[1:]
        xVel[-1] = vel_xLineTwist

        yVel[:-1] = yVel[1:]
        yVel[-1] = vel_yLineTwist

        zVel[:-1] = zVel[1:]
        zVel[-1] = vel_zLineTwist

        fig.clear()
        plt.subplot(321)
        plt.ylim([-2,2])
        plt.plot(alt, linewidth=5)
        plt.grid(True)
        plt.xlabel('Time Sequence', fontsize=20)
        plt.ylabel('altitude', fontsize=20)

        plt.subplot(322)
        plt.ylim([-2,2])
        plt.plot(xVel, linewidth=5)
        plt.grid(True)
        plt.xlabel('Time Sequence', fontsize=20)
        plt.ylabel('xVel', fontsize=20)

        plt.subplot(323)
        plt.ylim([-2,2])
        plt.plot(yVel, linewidth=5)
        plt.grid(True)
        plt.xlabel('Time Sequence', fontsize=20)
        plt.ylabel('yVel', fontsize=20)

        plt.subplot(324)
        plt.ylim([-2,2])
        plt.plot(zVel, linewidth=5)
        plt.grid(True)
        plt.xlabel('Time Sequence', fontsize=20)
        plt.ylabel('zVel', fontsize=20)

        input_data = [[alt[-1],xVel[-1],yVel[-1],zVel[-1]]]
        output_state = clf.predict(input_data)
        print(output_state)
        
        currentState[:-1] = currentState[1:]
        currentState[-1] = Statedic[output_state[0]]
        plt.subplot(325)
        plt.ylim([0,7])
        plt.plot(currentState,label=output_state[0], linewidth=5)
        plt.grid(True)
        plt.xlabel('Time Sequence', fontsize=20)
        plt.ylabel('Current State', fontsize=20)
        plt.legend(loc=9, fontsize=24)


        plt.pause(0.0001)
        fig.canvas.draw()
        rate.sleep()
        

if __name__ == '__main__':
    print("PIE node\n")
    try:
        pie()
    except rospy.ROSInterruptException:
        pass
    
