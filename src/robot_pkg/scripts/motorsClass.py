from numpy import arange, deg2rad, round, array
from math import sin, cos, atan
from random import random
from typing import Tuple

import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685

from rospy import logerr, loginfo, logwarn

class Motor():
    def __init__(self, x_multi, y_multi, center2Wheel = 19.09):
        GPIO.setwarnings(False)
        if GPIO.getmode() == None:
            GPIO.setmode(GPIO.BOARD)  
        
        self.center2Wheel = center2Wheel
        self.speed = 0
        self.speed_shim = 0
        self.x_multi = x_multi
        self.y_multi = y_multi

    def _singleFrontPotate(self, V, fid) -> None:
        """
        Вычисление скорости колеса (экземпляра) из скоростей (x, y)
        """
        self.speed =  self.x_multi*V[0] + self.y_multi*V[1] + fid

    def setSpeedShim(self, newSpeed) -> None:
        """
        Установка скорости вращения колеса для ШИМ-а
        """
        self.speed_shim = newSpeed


class Route(Motor):
    def __init__(self, a, b, c, d) -> None:
        self.ListOfMotors = [a, b, c, d]
        self.xy_speeds = [0, 0]

    def _reversPotate(self, FuncOfAngel) -> None:
        """
        Расчёт скоростей (x, y) из скоростей колёс (a, b, c, d)
        """
        x = (self.ListOfMotors[0] - self.ListOfMotors[2] - 2*Motor.center2Wheel*FuncOfAngel)*0.5
        y = (self.ListOfMotors[1] - self.ListOfMotors[3] - 2*Motor.center2Wheel*FuncOfAngel)*0.5
        self.xy_speeds = [x, y]

    def _frontPotate(self, ModeOfAngles, FuncOfAngel) -> None:
        """
        Расчёт скоростей колёс (экземпляров) из скоростей (x, y)
        """
        for i in self.ListOfMotors:
            i._singleFrontPotate(self.xy_speeds, self.__differencial(ModeOfAngles, FuncOfAngel))

    def setSpeed(self, *args, ModeOfAngles = 0, FuncOfAngel = 0, turnOsSys = 0) -> list:
        """
        Расчёт скорости колеса (экземпляра) из скоростей (x, y)
        """
        if turnOsSys:
            args = self.potateOfSys(args[0],args[1],deg2rad(turnOsSys))
        
        self.xy_speeds = args
        self._frontPotate(ModeOfAngles, FuncOfAngel)
        
        return(self.getAllMotorsSpeeds())

    def getAllMotorsSpeeds(self) -> list:
        """
        Возвращает массив скоростей колёс
        """
        res = []
        for i in self.ListOfMotors:
            res.append(round(i.speed, 2))
        return(res)

    @staticmethod
    def __differencial(mode, args) -> float:
        """
        Статичный метод для численного диференцирования (для поворота)
        """
        if not mode:
            return 0
        else:
            dif = (args[1]-args[0])/(0.5)
        return dif
    
    @staticmethod
    def potateOfSys(a,b,alfa) -> list:
        """
        Статичный метод для вращения точек (a,b) на угол alfa
        """
        pole_phi =0
        if a and b:
            pole_phi = atan(b/a)
            if a < 0: pole_phi = pole_phi + 3.14
        elif a == 0:
            if b > 0: pole_phi = deg2rad(90)
            if b < 0: pole_phi = deg2rad(270)
        elif b == 0:
            if a > 0: pole_phi = deg2rad(0)
            if a < 0: pole_phi = deg2rad(180)
        else: 
            rospy.logerr("ERROR INCORRECT DATA")
            return([0, 0])
        pole_len = ((a**2+b**2)**0.5)/(2**0.5)
            
        xr = round(pole_len*cos(pole_phi+alfa), 2)
        yr = round(pole_len*sin(pole_phi+alfa), 2)
        return([xr, yr])

def main() -> None:
    """
    Тестовая часть программы.
    Проверка работоспособности методов
    """
    A = Motor(1,   0)
    B = Motor(0,   1)
    C = Motor(-1,  0)
    D = Motor(0,  -1)
    abcd = Route(A, B, C, D)
    angle = 315
    listTest = [[0.5,0],[-0.5,0]]
    for i in listTest:
        joy = i
        print(abcd.setSpeed(joy[0], joy[1], turnOsSys=angle))

if __name__=="__main__":
    main()
