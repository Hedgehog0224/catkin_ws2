#!/usr/bin/python3
# Библиотеки
import sys
from numpy import round
from math import cos, sin, pi
from time import sleep,time

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from robot_pkg.msg import Speeds

class lidar():
    def __init__(self):
        """
        Инициализация нод и подписчиков
        """
        rospy.init_node('lidarColcul')
        rospy.Subscriber("scan", LaserScan, self.callbackScan)
        rospy.Subscriber("distance", Float32, self.callbackUltraZd)
        self.pub = rospy.Publisher('lidarData', Speeds, queue_size=10)
        self.msg = Speeds()
        self.stopAll = False # False = move; True = stop
        rospy.loginfo("lidarColcul correct")
    
    def callbackScan(self, data) -> None:
        """
        Функция обратной связи лидара и обработка данных
        """
        new_data = data.ranges[0:int(len(data.ranges)*0.4)] + data.ranges[int(len(data.ranges)*0.7):int(len(data.ranges))]
        size_nd = len(new_data)
        temp = min(new_data)

        if (temp > 0.6) or (dataFloat < 20):
            self.msg.x = 0
            self.msg.y = 0

        else:
            minArr = [i for i, j in enumerate(new_data) if j == temp]
            border = [min(minArr), max(minArr)]
            if border[1] - border[0] > size_nd/2:
                sr = abs(((border[0] + border[1])/2)-size_nd/2)
            else:
                sr = (border[0] + border[1])/2
            x = cos((sr/size_nd)*2*pi)
            y = sin((sr/size_nd)*2*pi)
            self.msg.x = x
            self.msg.y = y
        self.pub.publish(self.msg)
        rospy.loginfo_once("lidarData correct")

    def callbackUltraZd(self, data) -> None:
        """
        Обратная связь дальномера
        """
        dataFloat = data.data
        rospy.loginfo_once("distance correct")

lidarOb = lidar()
rospy.sleep(0.05)
rospy.spin()