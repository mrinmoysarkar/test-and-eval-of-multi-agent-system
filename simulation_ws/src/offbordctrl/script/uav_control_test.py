#!/usr/bin/env python

from __future__ import print_function

import rospy
import time
import threading
from uav_control import uavControl

if __name__ == '__main__':
    noofUav = 1
    rospy.init_node('hover_and_land_scenario_uav', anonymous=True)
    uavs = []
    for i in range(noofUav):
        uavs.append(uavControl(i))
        rospy.sleep(5)
        threading.Thread(target=uavs[i].executeVerticalScenario).start()
    while not rospy.is_shutdown():
        try:
            uavNo = raw_input('input uav no 0 to '+str(noofUav-1)+': ')
            uavNo = int(uavNo)
            cmd = raw_input("command: \"l\" for land and then \"q\" for exit the main loop: ")
            if cmd=='l':
                uavs[uavNo].stopThread = True
            elif cmd=='q':
                for i in range(noofUav):
                    uavs[i].stopThread = True
                rospy.sleep(20)
                break
        except Exception as e:
            pass
