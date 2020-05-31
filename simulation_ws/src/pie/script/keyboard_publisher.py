#!/usr/bin/env python

import rospy
import time
import threading
#import keyboard

from pynput.keyboard import Key, Listener, KeyCode

from std_msgs.msg import Float32MultiArray
import rosgraph


def on_press(key):
    global listener, key_pub, data
    
    #print('{0} pressed'.format(key))tt
    flag = False
    if key == KeyCode.from_char('u'):
        data[0] = 1.0 - data[0]
        flag = True
    elif key == KeyCode.from_char('t'):
        data[1] = 1.0 - data[1]
        flag = True
    elif key == KeyCode.from_char('l'):
        data[2] = 1.0 - data[2]
        flag = True
    elif key == KeyCode.from_char('d'):
        data[3] = 1.0 - data[3]
        flag = True
    elif key == KeyCode.from_char('w'):
        data[4] = 1.0 - data[4]
        flag = True
    elif key == KeyCode.from_char('f'):
        data[5] = 1.0 - data[5]
        flag = True
    elif key == KeyCode.from_char('q'):
        data[6] = 1.0 - data[6]
        flag = True
    elif key == KeyCode.from_char('e'):
        data[7] = 1.0 - data[7]
        flag = True
    elif key == Key.right:
        data[8] = 1.0 - data[8]
        flag = True
    elif key == Key.left:
        data[9] = 1.0 - data[9]  
        flag = True  
    elif key == Key.up:
        data[10] = 1.0 - data[10] 
        flag = True
    elif key == Key.down:   
        data[11] = 1.0 - data[11]
        flag = True

    #print(data)
    if flag:
        msg = Float32MultiArray()
        msg.data = data
        key_pub.publish(msg) 

    if not rosgraph.is_master_online():
        print("shutting down key listener")
        listener.stop()
        return False

def on_release(key):  
    global listener, key_pub
    #data = [0.0]*12
    #print('{0} release'.format(key))
    #msg = Float32MultiArray()
    #msg.data = data
    #key_pub.publish(msg)
    if key == Key.esc:
        # Stop listener
        listener.stop()
        return False

    if not rosgraph.is_master_online():
        listener.stop()
        return False

# Collect events until released         


if __name__ == '__main__':
    data = [0.0]*12
    rospy.init_node('keyboard_publisher_node', anonymous=True)
    key_pub = rospy.Publisher('/keyboard_data', Float32MultiArray, queue_size=10)

    #while not rospy.is_shutdown():
        
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
