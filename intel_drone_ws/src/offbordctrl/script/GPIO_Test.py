#!/usr/bin/env python
import time
from periphery import GPIO
import spidev
import rospy
from std_msgs.msg import String

# spi = spidev.SpiDev()
# bus=1
# device=1
# spi.open(bus, device)
# to_send = [0x8F, 0x8F]
# spi.xfer(to_send)
# print(spi.bits_per_word)
# spi.close()
# print("Top LED Blue")
# gpio = GPIO(403, "out")
# gpio.write(bool(1))
# time.sleep(10)
# gpio.write(bool(0))
# gpio.close()

# print("Top LED Green")
# gpio = GPIO(397, "out")
# gpio.write(bool(1))
# time.sleep(10)
# gpio.write(bool(0))
# gpio.close()

# print("Top LED Red")
# gpio = GPIO(437, "out")
# gpio.write(bool(1))
# time.sleep(10)
# gpio.write(bool(0))
# gpio.close()

# print("Bottom LED Orange")
# gpio = GPIO(507, "out")
# gpio.write(bool(10))
# time.sleep(1)
# gpio.write(bool(0))
# gpio.close()


# gpio = GPIO(485, "out") # GPIO 04
# gpio.write(bool(1))
# time.sleep(10)
# gpio.write(bool(0))
# gpio.close()

# gpio = GPIO(481, "out") # GPIO 03
# gpio.write(bool(1))
# time.sleep(5)
# gpio.write(bool(0))
# gpio.close()

# gpio = GPIO(487, "out") # GPIO 01
# gpio.write(bool(1))
# time.sleep(5)
# gpio.write(bool(0))
# gpio.close()

# trigPin = 487
# echoPin = 481


# trig = GPIO(trigPin, "out") 



# echo = GPIO(echoPin, "in") 
# # echo.edge = "rising" #"falling" #"rising" #"both"
# print(echo.read())
# for i in range(1000000):
#     trig.write(bool(0))
#     time.sleep(1/10000000.0)
#     trig.write(bool(1))
#     time.sleep(1/10000000.0)
#     trig.write(bool(0))
#     time.sleep(1)

# # echo.poll(1.0)
# # t1=time.time()
# # echo.edge = "falling"
# # print(echo.poll(1.0))
# # t2=time.time()
# # d = (t2-t1)*1000000.0/58.0
# # print("cm: ",d)
# for i in range(10):
#     print(echo.read())

# trig.close()
# echo.close()


rospy.init_node('hover_and_land_scenario', anonymous=True)
distsensorpub = rospy.Publisher('/distsensor',String,queue_size=100)
    
rate = rospy.Rate(2.0)

obstaclePin = 485

obstacle = GPIO(obstaclePin, "in") 
# echo.edge = "rising" #"falling" #"rising" #"both"
while not rospy.is_shutdown():
    obstacleflag = obstacle.read()
    if obstacleflag:
        distsensorpub.publish("obstacle")
    else:
        distsensorpub.publish("noobstacle")
    rate.sleep()

obstacle.close()   