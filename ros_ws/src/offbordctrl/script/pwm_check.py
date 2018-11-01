#!/usr/bin/python

import thread
import time
import rospy

from mavros_msgs.msg import ActuatorControl
from mavros_msgs.srv import CommandBool


pwm_value = 0

# Define a function for the thread
def publish_pwm(threadName):
    global pwm_value
    while pwm_value != -1:
       pwm_value=input("pwm value:")




if __name__ == '__main__':
# Create two threads as follows
    try:
       thread.start_new_thread( publish_pwm, ("PWM-check-thread", ) )
    except:
       print "Error: unable to start thread"

    
    rospy.init_node('pwm_check', anonymous=True)
    pub  = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=100)
   
    msgActrl = ActuatorControl()
    rate = rospy.Rate(20) # 10hz
    seq = -1
    # Arm
    print("ARM")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armResponse = armService(True)
        rospy.loginfo(armResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)



    while (not rospy.is_shutdown()) and (pwm_value != -1):
        seq += 1 
        motor1 = pwm_value #range 0..1
        motor2 = pwm_value
        motor3 = pwm_value
        motor4 = pwm_value

        msgActrl.header.stamp = rospy.Time.now()
        msgActrl.header.seq=seq
        msgActrl.header.frame_id = "base"
        msgActrl.group_mix = msgActrl.PX4_MIX_FLIGHT_CONTROL
        msgActrl.controls[0] = motor1
        msgActrl.controls[1] = motor2
        msgActrl.controls[2] = motor3
        msgActrl.controls[3] = motor4
        msgActrl.controls[4] = 0
        msgActrl.controls[5] = 0
        msgActrl.controls[6] = 0
        msgActrl.controls[7] = 0
        pub.publish(msgActrl)
        rate.sleep()
