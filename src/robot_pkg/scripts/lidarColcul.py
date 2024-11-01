#!/usr/bin/python3
# Библиотеки
import sys
from numpy import round
from math import cos, sin, pi
from time import sleep,time

import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from robot_pkg.msg import Speeds

# Локальная библиотека для вычислений
from MatMotors import Route, Motor

# Главный класс
class lidar():
    # Переменные класса
    def __init__(self):
        """
        Инициализация нод и подписчиков
        """
        rospy.init_node('NodeForLidar')
        rospy.Subscriber("scan", LaserScan, self.callbackScan)
        rospy.Subscriber("distance", Float32, self.callbackUltraZd)
        pub = rospy.Publisher('LidarData', Speeds, queue_size=10)
        msg = Speeds()
    
    def callbackScan(self, data) -> None:
        """
        Функция обратной связи лидара и обработка данных
        """
        new_data = data.ranges[0:int(len(data.ranges)*0.4)] + data.ranges[int(len(data.ranges)*0.7):int(len(data.ranges))]
        size_nd = len(new_data)
        temp = min(new_data)

        # stop
        if temp > 0.6:
            self.msg.x = 0
            self.msg.y = 0

        # move
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
        self.publish(msg)

    def callbackUltraZd(self, data) -> None:
        """
        Обратная связь дальномера
        """
        dataFloat = data.data
        if dataFloat < 20.0:
            robotcl.varStopAll = True
        else:
            robotcl.varStopAll = False

lidarOb = lidar()
rospy.sleep(0.05)
rospy.spin()