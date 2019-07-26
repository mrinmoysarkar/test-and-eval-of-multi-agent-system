#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import pandas as pd
import threading


stopThread = False
pub = None

def move(pub,x,y,z):
    twist = Twist()
    speed = 0.1   
    th = 0
    twist.linear.x = x*speed 
    twist.linear.y = y*speed
    twist.linear.z = z*speed
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    pub.publish(twist)

def loop():
  global stopThread
  global pub
  loopCount = 400
  rate = rospy.Rate(10.0)
  
  while not rospy.is_shutdown() and not stopThread:
    for i in range(loopCount):
      move(pub,1,0,0) 
      rate.sleep()
    move(pub,0,0,0)  
    rospy.sleep(5)
    for i in range(loopCount):
      move(pub,-1,0,0) 
      rate.sleep()
    move(pub,0,0,0)  
    rospy.sleep(5)

  move(pub,0,0,0)  
  rospy.sleep(2)  
  print("stopping the loop")


if __name__ == '__main__':          
  try:
    rospy.init_node('move_jackle_node', anonymous=True)

    pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  
    rospy.sleep(5)

    threading.Thread(target=loop).start()

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        rate.sleep()
        cmd = raw_input("command: \"s\" to stop the UGV and \"exit\" for exit the main loop: ")
        # print(cmd)
        if cmd=='s':
            stopThread = True
        elif cmd=='exit':
            break

  except rospy.ROSInterruptException:
    pass



