#!/usr/bin/python3
# import sys
# from numpy import round
# from math import cos, sin, pi
# from time import sleep,time
# import RPi.GPIO as GPIO
# import board
# from adafruit_pca9685 import PCA9685
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from robot_pkg.msg import Servodata, Speeds, Joysteack
# GPIO.setwarnings(False)
# if GPIO.getmode() == None:
#     GPIO.setmode(GPIO.BOARD)  
from motorsClass import Route, Motor

class mainClass():
    def __init__(self):
        rospy.init_node('mainNode')
        rospy.Subscriber('enterData', Servodata, self.callbackEnter)
        rospy.Subscriber('cameraData', Servodata, self.callbackCamera)
        rospy.Subscriber('lidarData', Speeds, self.callbackLidarUz)
        rospy.Subscriber('modeData', Joysteack, self.callbackMode)

        self.motorsPub = rospy.Publisher('motors', Speeds, queue_size=10)
        self.motorsPub = rospy.Publisher('manipulatorData', Servodata, queue_size=10)

        self.I2Cpins = [0,1,2,5,3,4,6,8,7,11,9,10]
        self.mode = 0
        self.varStopAll = 0
        self.JoySpeed = [0.0, 0.0]
        self.JoyAngle = 0
        self.PredArrForMove = [0,0,0,0]
        self.A = Motor( 1,  0)
        self.B = Motor( 0,  1)
        self.C = Motor(-1,  0)
        self.D = Motor( 0, -1)
        self.abcd = Route(self.A, self.B, self.C, self.D)

        rospy.loginfo_once("Node init correct")

    def callbackEnter(self, data):
        self.r = data.servo0
        self.theta = data.servo1
        self.fi = data.servo2
        self.xOfGripper = data.servo3
        self.zOfGripper = data.servo4
        self.gripper = data.servo5

    def callbackCamera(self, data):
        self.r = data.servo0
        self.theta = data.servo1
        self.fi = data.servo2
        self.xOfGripper = data.servo3
        self.zOfGripper = data.servo4
        self.gripper = data.servo5

    def callbackLidarUz(self, data):
        self.ultraSonicDist = data.data

    def callbackMode(self, data):
        self.x = data.x
        self.y = data.y
        self.angle = data.angle
        self.mode = data.mode

cringebot = mainClass()
rospy.sleep(0.05)
rospy.spin()