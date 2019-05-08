#!/usr/bin/python

import thread
import time
import rospy
import tf

from mavros_msgs.msg import ActuatorControl
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped



class state():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.xdot = 0
        self.ydot = 0
        self.zdot = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0
        self.phidot = 0
        self.thetadot = 0
        self.psidot = 0
        self.t = 0
    def copy(self,target):
        target.x = self.x
        target.y = self.y
        target.x = self.z
        target.xdot = self.xdot
        target.ydot = self.ydot
        target.zdot = self.zdot
        target.phi = self.phi
        target.theta = self.theta
        target.psi = self.psi
        target.phidot = self.phidot
        target.thetadot = self.thetadot
        target.psidot = self.psidot
        target.t = self.t
        return target
    def setValue(self,x=0,y=0,z=0,xdot=0,ydot=0,zdot=0,phi=0,theta=0,psi=0,phidot=0,thetadot=0,psidot=0):
        self.x = x
        self.y = y
        self.z = z
        self.xdot = xdot
        self.ydot = ydot
        self.zdot = zdot
        self.phi = phi
        self.theta = theta
        self.psi = psi
        self.phidot = phidot
        self.thetadot = thetadot
        self.psidot = psidot


class PID():
    def __init__(self,kp_x=0,kp_y=0,kp_z=0,kp_phi=0,kp_theta=0,kp_psi=0):
        self.kp_x = kp_x
        self.kp_y = kp_y
        self.kp_z = kp_z
        self.kp_phi = kp_phi
        self.kp_theta = kp_theta
        self.kp_psi = kp_psi
    def calculate_omega(self,current_state,desired_state):
        u1 = self.kp_z*(desired_state.z-current_state.z)+.6
        u2 = self.kp_phi*(desired_state.phi-current_state.phi)
        u3 = self.kp_theta*(desired_state.theta-current_state.theta)
        u4 = self.kp_psi*(desired_state.psi-current_state.psi)
        omega=[0,0,0,0]
        omega[0] = self.limit_output(u1+u3+u4)
        omega[1] = self.limit_output(u1-u2-u4)
        omega[2] = self.limit_output(u1-u3+u4)
        omega[3] = self.limit_output(u1+u2-u4)
        return omega
    def limit_output(self,x):
        if x<0:
            x=0
        elif x>=0.6:
            x=0.6
        return x
         

pwm_value = 0
current_state = state()
previous_state = state()


# Define a function for the thread
def publish_pwm(threadName):
    global pwm_value
    while pwm_value != -1:
       pwm_value=input("pwm value:")


def pos_callback(data):
    global current_state
    global previous_state
    previous_state = current_state.copy(previous_state)
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    quaternion = (data.pose.orientation.x,
                  data.pose.orientation.y,
                  data.pose.orientation.z,
                  data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    t = data.header.stamp.to_sec()#float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*1e-9
    dt = t - previous_state.t
    current_state.x = x
    current_state.y = y
    current_state.z = z
    current_state.xdot = (x-previous_state.x)/float(dt)
    current_state.ydot = (y-previous_state.y)/float(dt)
    current_state.zdot = (z-previous_state.z)/float(dt)
    current_state.phi = roll
    current_state.theta = pitch
    current_state.psi = yaw
    current_state.phidot = (roll-previous_state.phi)/float(dt)
    current_state.thetadot = (pitch-previous_state.theta)/float(dt)
    current_state.psidot = (yaw-previous_state.psi)/float(dt)
    current_state.t = t
    #print("current_state : ", current_state)



if __name__ == '__main__':
# Create two threads as follows
    try:
       thread.start_new_thread( publish_pwm, ("PWM-check-thread", ) )
    except:
       print "Error: unable to start thread"

    
    rospy.init_node('pwm_check', anonymous=True)
    pub  = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=100)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pos_callback)
   
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


    desired_state = state()
    desired_state.setValue(z=1.0)
    
    kp_z = 0.08
    kp_phi = 0.01
    kp_theta = 0.01
    kp_psi = 0.01
    
    pid_control = PID(kp_z=kp_z,kp_phi=kp_phi,kp_theta=kp_theta,kp_psi=kp_psi)
    
    mg = 0.566
    while (not rospy.is_shutdown()) and (pwm_value != -1):
        seq += 1
        
        omega = pid_control.calculate_omega(current_state,desired_state)

        motor1 = omega[0]#pwm_value #range 0..1
        motor2 = omega[1]#pwm_value
        motor3 = omega[2]#pwm_value
        motor4 = omega[3]#pwm_value

        msgActrl.header.stamp = rospy.Time.now()
        #msgActrl.header.seq=seq
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
