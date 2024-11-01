#!/usr/bin/python3
from math import pi, cos, sin, atan2, atan, acos
from numpy import zeros, matmul, deg2rad, array, ndarray, rad2deg
from typing import Union, Any, Optional
import rospy
import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from time import sleep

class potateKinem(
):
    def __init__(self, l0=185, l1=175, q=array([0,0,0,0,0])) -> None:
        self.l0 = l0
        self.l1 = l1
        self.q = q

    def computeDecart(self, x, y, z):
        r = (x**2 + y**2 + z**2)**0.5
        theta = atan((x**2 + y**2)**0.5/z)
        fi = atan(y/x)
        return self.computePolar(r, theta, fi)

    def computePolar(self, r, theta, fi):
        # print("Debug 1:", r, rad2deg(theta), rad2deg(fi))
        if r > self.l0 + self.l1:
            print("ERROR")
            return None
        angls = zeros(3)
        if fi < pi:
            angls[0] = fi
            # print("Debug 2:", angls[0])
        else: angls[0] = pi - fi
        if r == self.l0 + self.l1:
            angls[1] = pi - theta
            angls[2] = pi/2
        else:
            # print(rad2deg(theta-acos((self.l0**2 - self.l1**2 + r**2)/(2*self.l0*r))))
            try: 
                angls[1] = pi-theta-acos((self.l0**2 - self.l1**2 + r**2)/(2*self.l0*r))
            except:
                return None
            # print(rad2deg(acos((self.l0**2 + self.l1**2 - r**2)/(2*self.l0*self.l1))))
            try:
                angls[2] = acos((self.l0**2 + self.l1**2 - r**2)/(2*self.l0*self.l1)) - pi/2
            except:
                return None
            # print("Debug 3:", angls[1], angls[2])
        # print("Debug 4:", rad2deg(angls[0]), rad2deg(angls[1]), rad2deg(angls[2]))
        self.q[0] = angls[0]
        self.q[1] = angls[1]
        self.q[2] = angls[2]
        return angls

def main():
    motors = potateKinem()
    motors.q = [0,0,0,0,0]
    motors.move()        
    sleep(3)
        
if __name__=="__main__":
    main()