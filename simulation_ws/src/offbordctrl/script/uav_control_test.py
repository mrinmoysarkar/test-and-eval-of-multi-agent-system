#!/usr/bin/env python

from __future__ import print_function

import rospy
import time
import threading
from uav_control import uavControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import cv2
import numpy as np
from astar import main
import octomap
import inspect
from octomap_msgs.msg import Octomap
from pylib import get_path
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import queue as Q
import rosgraph

class Node(object):
    def __init__(self, parent, cost, pos):
        self.parent = parent
        self.cost = cost
        self.pos = pos

    def __cmp__(self, other):
        return cmp(self.cost, other.cost)

    def __eq__(self, other):
        return self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]

    def __str__(self):
        return 'cost:'+str(self.cost)+', (x,y)=('+str(self.pos[0])+','+str(self.pos[1])+')'


noofUav = 1
current_uav = 0
uavs = []
bridge_rgb = CvBridge()
count = 0
map_msg = None
t = time.time()

# 384 pixel is equivalent to 100m

def find_path(msg):
    map_h = msg.info.height
    map_w = msg.info.width
    map = np.array(msg.data).reshape((map_h, map_w))
    print(msg.info.origin)
    x_origin = msg.info.origin.position.x
    y_origin = msg.info.origin.position.y
    resolution = msg.info.resolution
    start_pos = [0,0]
    end_pos = [15,0]
    start_pos_in_map=[int((start_pos[0]-x_origin)/resolution), int((start_pos[1]-y_origin)/resolution)]
    end_pos_in_map=[int((end_pos[0]-x_origin)/resolution), int((end_pos[1]-y_origin)/resolution)]
    print(start_pos_in_map,end_pos_in_map,map_w,map_h)
    start_pos_in_map = [min(start_pos_in_map[0],map_w-1), min(start_pos_in_map[1],map_h-1)]
    end_pos_in_map = [min(end_pos_in_map[0],map_w-1), min(end_pos_in_map[1],map_h-1)]
    
    print(start_pos_in_map,end_pos_in_map,map_w,map_h)
    #visited_nodes = []
    #priority_queue = []
    priority_queue = Q.PriorityQueue()
    ######
    # map_h,map_w = 10,10
    # map = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    # start_pos_in_map = [0, 0]
    end_pos_in_map = [9, 8]
    ####
    
    #priority_queue.append(Node(None, 0, start_pos_in_map))
    priority_queue.put(Node(None, 0, start_pos_in_map))

    end_node = Node(None, 0, end_pos_in_map)

    total_node_processed = 0
    path = []
    processed_node = np.zeros((map_h,map_w),dtype=np.uint8)
    # print("while loop start")
    # print(priority_queue.empty())
    plt.imshow(np.abs(map),cmap='gray')
    plt.show()
    while not priority_queue.empty():
        try:
            #current_node = priority_queue[0]
            current_node = priority_queue.get()
            # print(current_node)
            # print(type(current_node.pos[0]))
            
            processed_node[current_node.pos[0], current_node.pos[1]] = 1
            
            # print("wtf")
            
            #del priority_queue[0]
            #visited_nodes.append(current_node)

            if current_node == end_node:
                print('found')
                
                while current_node.parent is not None:
                    pos = (current_node.pos[0]*resolution+x_origin, current_node.pos[1]*resolution+y_origin)
                    path.append(pos)
                    current_node = current_node.parent
                path.append(current_node.pos)
                print(path)
                return path
                # break

            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.pos[0] + new_position[0], current_node.pos[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (map_h - 1) or node_position[0] < 0 or node_position[1] > (map_w -1) or node_position[1] < 0:
                    continue

                #print(node_position)
                # Make sure walkable terrain
                if map[node_position[0],node_position[1]] > 40:
                    continue

                

                if processed_node[node_position[0],node_position[1]] == 1:
                    continue

                new_node = Node(current_node, 0, node_position)
                # if new_node in visited_nodes:
                #     continue
                # if new_node in priority_queue:
                #     continue
                g_cost = (current_node.pos[0] - node_position[0])**2 + (current_node.pos[1] - node_position[1])**2
                cost = current_node.cost + abs(map[node_position[0]][node_position[1]]) + 1 + g_cost
                new_node.cost = cost
                #priority_queue.append(new_node)
                priority_queue.put(new_node)
                processed_node[new_node.pos[0],new_node.pos[1]] = 1
            #priority_queue.sort()

            total_node_processed += 1
            # if total_node_processed % 1000 == 0:
            #     print('total_node: ', total_node_processed)
            if total_node_processed > map_h*map_w:
                print("more node is being processed.")
                break
        except Exception as e:
            print(e)
    plt.imshow(processed_node,cmap='gray')
    plt.show()
    print(priority_queue.empty())
    print('total node processed:',total_node_processed)
    return None


def map_2D_cb(msg):
    global t, map_msg
    if time.time()-t>10:
        map_msg = msg
        #print(map)
        t = time.time()

def octomap_cb(msg):
    global count, map_msg, path_cpp, path_pub
    map_msg = msg
    if count==-1:
        map_msg = msg

    # if count<7:
    #     # print('python data****************************************************************')
    #     # print(msg)
    #     # print('**************************************************************************')
    #     # print('cpp data****************************************************************')
        path = get_path(msg.header.seq,
            msg.header.stamp.secs, 
            msg.header.stamp.nsecs, 
            msg.header.frame_id,
            msg.binary, 
            msg.id, 
            msg.resolution,
            msg.data,
            (0.0,0.0,0.0),
            (15.0,0.0,5.0))
        print("from Python: ", path)
        path_cpp.header = msg.header
        for p in path:
            x = PoseStamped()
            x.header = msg.header
            x.pose.position.x = p[0]
            x.pose.position.y = p[1]
            x.pose.position.z = p[2]
            path_cpp.poses.append(x)
            
        path_pub.publish(path_cpp)

    #     # print(type(''.join(msg.data)))
    #     print('count: ', count)
    #     # print('from python: ', tree.getResolution())
    # count += 1

def keyboard_controll_cb(msg):
    global uavs, current_uav, noofUav, count
    print(msg.data)
    velocities = [0]*4
    if msg.data[0] == 1.0: # u
        current_uav += 1
        current_uav %= noofUav
        print('current UAV no. is ', current_uav)
    elif msg.data[1] == 1.0: # t
        uavs[current_uav].takeoff()
    elif msg.data[2] == 1.0: # l
        uavs[current_uav].land_custom()
    elif msg.data[3] == 1.0: # d
        uavs[current_uav].disarm()
    elif msg.data[4] == 1.0: # w
        velocities[2] = 0.5
    elif msg.data[5] == 1.0: # f
        velocities[2] = -0.5
    elif msg.data[6] == 1.0: # q
        velocities[3] = 0.5
        #count = -1
    elif msg.data[7] == 1.0: # e
        velocities[3] = -0.5
    elif msg.data[8] == 1.0: # ->
        velocities[1] = 0.5
    elif msg.data[9] == 1.0: # <-
        velocities[1] = -0.5
    elif msg.data[10] == 1.0: # ^
        velocities[0] = 0.5
    elif msg.data[11] == 1.0: # v
        velocities[0] = -0.5

    uavs[current_uav].set_velocity(velocities)

def uav_controll_pad_cb(msg):
    global uavs, current_uav, noofUav, count
    #print(msg.data)
    velocities = [0]*4
    if msg.data[0] == 1.0: 
        current_uav += 1
        current_uav %= noofUav
        print('current UAV no. is ', current_uav)
    elif msg.data[1] == 1.0: 
        uavs[current_uav].takeoff()
    elif msg.data[2] == 1.0: 
        uavs[current_uav].land_custom()
    elif msg.data[3] == 1.0:
        uavs[current_uav].disarm()
    elif msg.data[4] == 1.0: 
        velocities[2] = 0.5
    elif msg.data[5] == 1.0:
        velocities[3] = -0.5
    elif msg.data[6] == 1.0:
        velocities[3] = 0.5
    elif msg.data[7] == 1.0:
        velocities[2] = -0.5
    elif msg.data[8] == 1.0: 
        velocities[0] = 0.5
    elif msg.data[9] == 1.0: 
        velocities[1] = 0.5
    elif msg.data[10] == 1.0: 
        velocities[1] = -0.5
    elif msg.data[11] == 1.0: 
        velocities[0] = -0.5

    uavs[current_uav].set_velocity(velocities)

def joy_controll_cb(msg):
    global uavs, current_uav, noofUav, map_msg, count
    if 1 == msg.buttons[6]:
        current_uav += 1
        current_uav %= noofUav
        print('current UAV no. is ', current_uav)
    elif 1 == msg.buttons[4]: # takeoff
        uavs[current_uav].takeoff()
    elif 1 == msg.buttons[5]: # land
        uavs[current_uav].land_custom()
    elif 1 == msg.buttons[7]: # disarm
        uavs[current_uav].disarm()
    elif 1 == msg.buttons[3]: # go forward
        uavs[current_uav].go(1)
        count = 0
        # if None is not map_msg:
            # path = get_path(map_msg.header.seq,
            #     map_msg.header.stamp.secs, 
            #     map_msg.header.stamp.nsecs, 
            #     map_msg.header.frame_id,
            #     map_msg.binary, 
            #     map_msg.id, 
            #     map_msg.resolution,
            #     map_msg.data,
            #     (0.0,0.0,0.0),
            #     (10.0,10.0,10.0))
        count = -1
    elif 1 == msg.buttons[1]: # go backward
        uavs[current_uav].go(-1)
    elif 1 == msg.buttons[0]: # go left
        uavs[current_uav].go(2)
    elif 1 == msg.buttons[2]: # go right
        uavs[current_uav].go(-2)
    elif 1.0 == msg.axes[5]: # go up
        uavs[current_uav].go(3)
    elif -1.0 == msg.axes[5]: # go down
        uavs[current_uav].go(-3)
    else: #hover
        uavs[current_uav].go(0)
        count = 0
    uavs[current_uav].set_velocity([msg.axes[3], msg.axes[2], msg.axes[1], msg.axes[4]])

def map_image_cb(msg):
    global count, bridge_rgb, map_pub
    if count<7:
        cv_image = bridge_rgb.imgmsg_to_cv2(msg, msg.encoding)
        # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        cv_image = cv_image[48:432,128:512,:]
        _,img_red = cv2.threshold(cv_image[:,:,0], 200, 255, cv2.THRESH_BINARY_INV)
        _,img_green = cv2.threshold(cv_image[:,:,1], 200, 255, cv2.THRESH_BINARY_INV)
        _,img_blue = cv2.threshold(cv_image[:,:,2], 200, 255, cv2.THRESH_BINARY)
        # img_green_inv = cv2.bitwise_not(img_green)
        # img_blue_inv = cv2.bitwise_not(img_blue)
        obj_img = cv2.bitwise_and(img_green, img_blue)
        obj_img = cv2.bitwise_and(obj_img, img_red)
        
        rows,cols = obj_img.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
        obj_img = cv2.warpAffine(obj_img,M,(cols,rows))

        # plt.imshow(obj_img,cmap='gray')
        # plt.show()

        _, contours, _ = cv2.findContours(obj_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        grid_map = np.zeros([100,100],dtype=np.uint8)
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            if M["m00"] != 0.0:
               # calculate x,y coordinate of center
               cX = int(M["m10"] / M["m00"])
               cY = int(M["m01"] / M["m00"])
               # print(cX,cY)
               # cX = (cX-obj_img.shape[0]/2)*100/obj_img.shape[0]
               # cY = (cY-obj_img.shape[1]/2)*100/obj_img.shape[1]
               cX = cX*100/obj_img.shape[0]
               cY = cY*100/obj_img.shape[1]
               cX,cY = cY,cX
               # print(cX,cY)
               for i in range(-3,4,1):
                for j in range(-3,4,1):
                    occu_i = cX+i
                    occu_j = cY+j
                    if occu_i >= 0 and occu_i < 100 and occu_j >= 0 and occu_j < 100:
                        grid_map[occu_i,occu_j] = 1#255
        # plt.imshow(grid_map,cmap='gray')
        # plt.show()
        image_message = bridge_rgb.cv2_to_imgmsg(grid_map, encoding="passthrough")
        map_pub.publish(image_message)
               

    # count += 1

if __name__ == '__main__':
    # print(inspect.getmembers(octomap))
    rospy.init_node('controller_test_uav', anonymous=True)
    #rospy.Subscriber('/joy', Joy, joy_controll_cb)
    #rospy.Subscriber('/keyboard_data', Float32MultiArray, keyboard_controll_cb)
    # rospy.Subscriber('/uav_control_pad_data',Float32MultiArray, uav_controll_pad_cb)
    # rospy.Subscriber('/sensor_stand/r200_ir/image_raw', Image, map_image_cb)
    #rospy.Subscriber('/octomap_binary', Octomap, octomap_cb)
    # rospy.Subscriber('/projected_map',OccupancyGrid, map_2D_cb)
    # path_pub = rospy.Publisher('/path_from_map_cpp', Path, queue_size=10)
    # path_cpp = Path()


    #map_pub = rospy.Publisher('/map', Image, queue_size=10)
    
    for i in range(noofUav):
        uavs.append(uavControl(i))
        rospy.sleep(5)
        threading.Thread(target=uavs[i].planning_loop).start()
        # threading.Thread(target=uavs[i].control_loop).start()
        # threading.Thread(target=uavs[i].object_detection).start()
    # uavs[0].set_param(param_id='NAV_RCL_ACT', param_val=0, timeout=5) 
    # uavs[0].set_param(param_id='COM_OBL_RC_ACT', param_val=2, timeout=5)
    # uavs[0].get_param(param_id='COM_OBL_RC_ACT', timeout=5)

    rospy.spin()
    # while rosgraph.is_master_online():
    #     try:
    #         rospy.sleep(5)
    #     #     # uavNo = raw_input('input uav no 0 to '+str(noofUav-1)+': ')
    #     #     # uavNo = 0 #int(uavNo)
    #     #     cmd = raw_input("command: \"x\" for exit the main loop: ")
    #     #     if cmd=='x':
    #     #         for i in range(noofUav):
    #     #             uavs[i].stopThread = True
    #     #         rospy.sleep(10)
    #     #         break
    #     #     elif cmd=='m':
    #     #         if map_msg is not None:
    #     #             path = find_path(map_msg)
    #     #             if path is not None:
    #     #                 path_cpp.header = map_msg.header
    #     #                 for p in path:
    #     #                     x = PoseStamped()
    #     #                     x.header = map_msg.header
    #     #                     x.pose.position.x = p[0]
    #     #                     x.pose.position.y = p[1]
    #     #                     x.pose.position.z = 5#p[2]
    #     #                     path_cpp.poses.append(x)
                            
    #     #                 path_pub.publish(path_cpp)
 
                
    #             # path = get_path(map_msg.header.seq,
    #             # map_msg.header.stamp.secs, 
    #             # map_msg.header.stamp.nsecs, 
    #             # map_msg.header.frame_id,
    #             # map_msg.binary, 
    #             # map_msg.id, 
    #             # map_msg.resolution,
    #             # map_msg.data,
    #             # (0.0,0.0,0.0),
    #             # (15.0,0.0,5.0))
    #             # print("from Python: ", path)
                    
                

    #     except Exception as e:
    #         print(e)
