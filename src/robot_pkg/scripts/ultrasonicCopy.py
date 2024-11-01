#!/usr/bin/python3 
import rospy
from sensor_msgs.msg import LaserScan

def clb(data):
    print('____________') 
    print(min(data.ranges))
    minArr = [i for i, j in enumerate(data.ranges) if j == min(data.ranges)]
    print(minArr)

rospy.init_node("Test")
pub = rospy.Subscriber('scan', LaserScan, clb)

rospy.spin()
rospy.sleep(0.01)