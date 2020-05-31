#!/usr/bin/env python

import rospy
import time


from std_msgs.msg import Float32MultiArray
import rosgraph

import tkinter as tk 
import tkinter.font as font

def change_uav_pressed(event):
    global key_pub, data
    data[0] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def change_uav_released(event):
    global key_pub, data
    data[0] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def takeoff_pressed(event):
    global key_pub, data
    data[1] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def takeoff_released(event):
    global key_pub, data
    data[1] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def land_pressed(event):
    global key_pub, data
    data[2] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def land_released(event):
    global key_pub, data
    data[2] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def disarm_pressed(event):
    global key_pub, data
    data[3] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def disarm_released(event):
    global key_pub, data
    data[3] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def up_pressed(event):
    global key_pub, data
    data[4] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def up_released(event):
    global key_pub, data
    data[4] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def rotate_cc_pressed(event):
    global key_pub, data
    data[5] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def rotate_cc_released(event):
    global key_pub, data
    data[5] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def rotate_cw_pressed(event):
    global key_pub, data
    data[6] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def rotate_cw_released(event):
    global key_pub, data
    data[6] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def down_pressed(event):
    global key_pub, data
    data[7] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def down_released(event):
    global key_pub, data
    data[7] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def forward_pressed(event):
    global key_pub, data
    data[8] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def forward_released(event):
    global key_pub, data
    data[8] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def left_pressed(event):
    global key_pub, data
    data[9] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def left_released(event):
    global key_pub, data
    data[9] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def right_pressed(event):
    global key_pub, data
    data[10] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def right_released(event):
    global key_pub, data
    data[10] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def backward_pressed(event):
    global key_pub, data
    data[11] = 1.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

def backward_released(event):
    global key_pub, data
    data[11] = 0.0
    msg = Float32MultiArray()
    msg.data = data
    key_pub.publish(msg) 
    return

if __name__ == '__main__':
    data = [0.0]*12
    rospy.init_node('uav_control_pad_publisher_node', anonymous=True)
    key_pub = rospy.Publisher('/uav_control_pad_data', Float32MultiArray, queue_size=10)
    
    master = tk.Tk() 
    # master.geometry("500x200")
    master.title('UAV Control Pad') 

    myFont = font.Font(size=16)
    
    b1 = tk.Button(master, text='Change UAV', font=myFont) 
    b1.grid(row=0, column=0, columnspan=1, rowspan=3, padx=2, pady=2) 
    b1.bind("<ButtonPress-1>",change_uav_pressed)
    b1.bind("<ButtonRelease-1>",change_uav_released)


    b2 = tk.Button(master, text='Takeoff', font=myFont) 
    b2.grid(row=0, column=1, columnspan=1, rowspan=1, padx=50, pady=2)
    b2.bind("<ButtonPress-1>",takeoff_pressed)
    b2.bind("<ButtonRelease-1>",takeoff_released)

    b3 = tk.Button(master, text='Land',  font=myFont) 
    b3.grid(row=1, column=1, columnspan=1, rowspan=1, padx=50, pady=2)
    b3.bind("<ButtonPress-1>",land_pressed)
    b3.bind("<ButtonRelease-1>",land_released)

    b4 = tk.Button(master, text='Disarm',  font=myFont) 
    b4.grid(row=2, column=1, columnspan=1, rowspan=1, padx=50, pady=2)
    b4.bind("<ButtonPress-1>",disarm_pressed)
    b4.bind("<ButtonRelease-1>",disarm_released)
    

    b5 = tk.Button(master, text='Up', font=myFont) 
    b5.grid(row=0, column=2, columnspan=2, rowspan=1, padx=10, pady=2)
    b5.bind("<ButtonPress-1>",up_pressed)
    b5.bind("<ButtonRelease-1>",up_released)

    b6 = tk.Button(master, text='Rotate CC', font=myFont) 
    b6.grid(row=1, column=2, columnspan=1, rowspan=1, padx=5, pady=2)
    b6.bind("<ButtonPress-1>",rotate_cc_pressed)
    b6.bind("<ButtonRelease-1>",rotate_cc_released)

    b7 = tk.Button(master, text='Rotate CW', font=myFont) 
    b7.grid(row=1, column=3, columnspan=1, rowspan=1, padx=5, pady=2)
    b7.bind("<ButtonPress-1>",rotate_cw_pressed)
    b7.bind("<ButtonRelease-1>",rotate_cw_released)

    b8 = tk.Button(master, text='Down', font=myFont) 
    b8.grid(row=2, column=2, columnspan=2, rowspan=1, padx=10, pady=2)
    b8.bind("<ButtonPress-1>",down_pressed)
    b8.bind("<ButtonRelease-1>",down_released)

    

    b9 = tk.Button(master, text='Forward', font=myFont) 
    b9.grid(row=0, column=4, columnspan=2, rowspan=1, padx=10, pady=2)
    b9.bind("<ButtonPress-1>",forward_pressed)
    b9.bind("<ButtonRelease-1>",forward_released)

    b10 = tk.Button(master, text='Left', font=myFont) 
    b10.grid(row=1, column=4, columnspan=1, rowspan=1, padx=5, pady=2)
    b10.bind("<ButtonPress-1>",left_pressed)
    b10.bind("<ButtonRelease-1>",left_released)

    b11 = tk.Button(master, text='Right', font=myFont) 
    b11.grid(row=1, column=5, columnspan=1, rowspan=1, padx=5, pady=2)
    b11.bind("<ButtonPress-1>",right_pressed)
    b11.bind("<ButtonRelease-1>",right_released)

    b12 = tk.Button(master, text='Backward', font=myFont) 
    b12.grid(row=2, column=4, columnspan=2, rowspan=1, padx=10, pady=2)
    b12.bind("<ButtonPress-1>",backward_pressed)
    b12.bind("<ButtonRelease-1>",backward_released)

    

    master.mainloop() 