#!/usr/bin/python3
import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from numpy import array, rad2deg
from motorsClass import Route, Motor

class movePWM():
    def __init__(self, ListOfMotors, numsOfControllers = 1, frequency = 100):
        self.ListOfMotors = ListOfMotors
        self.numsOfControllers = numsOfControllers
        self.__setUpI2C(frequency)

    def __setUpI2C(self, frequency) -> None:
        GPIO.setwarnings(False)
        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = frequency
        if self.numsOfControllers > 1:
            self.pca1 = PCA9685(self.i2c, address = 0x41)
            self.pca1.frequency = frequency 

    def setUpServos(self, numsPins) -> None:
        self.servo0 = servo.Servo(self.pca.channels[numsPins[0]], actuation_range = 180, min_pulse = 450, max_pulse=2750)
        self.servo1 = servo.Servo(self.pca.channels[numsPins[1]], actuation_range = 180, min_pulse = 500, max_pulse=2550)
        self.servo2 = servo.Servo(self.pca.channels[numsPins[2]], actuation_range = 180, min_pulse = 450, max_pulse=2750)
        self.servo3 = servo.Servo(self.pca.channels[numsPins[3]], actuation_range = 180, min_pulse = 450, max_pulse=2750)
        if self.numsOfControllers > 1:
            self.servo4 = servo.Servo(self.pca1.channels[numsPins[4]], actuation_range = 180, min_pulse = 450, max_pulse=2750)
            self.servoGripper = servo.Servo(self.pca1.channels[numsPins[5]], actuation_range = 180, min_pulse = 750, max_pulse=2250)
            self.servoCamera = servo.Servo(self.pca1.channels[numsPins[6]], actuation_range = 180, min_pulse = 750, max_pulse=2250)

    def moveMotors(self, SpeedsMotors, numsPins) -> None:
        maxSpeed = max(max(SpeedsMotors), abs(min(SpeedsMotors)))
        if maxSpeed: SpeedsMotors = array(SpeedsMotors)/maxSpeed
            
        for n, i in enumerate(self.ListOfMotors):
            try: 
                i.setSpeedShim(int(hex(int(abs(SpeedsMotors[n]**4)*65535)), 16))
            except: 
                logerr("ERROR OF MATH")
                print(int(hex(int(abs(SpeedsMotors[n]**4)*65535)), 16))
        
        self.pca.channels[numsPins[0]].duty_cycle = self.ListOfMotors[0].speed_shim
        self.pca.channels[numsPins[1]].duty_cycle = int(SpeedsMotors[0] > 0)*65535
        self.pca.channels[numsPins[2]].duty_cycle = int(SpeedsMotors[0] < 0)*65535
        
        self.pca.channels[numsPins[3]].duty_cycle = self.ListOfMotors[1].speed_shim
        self.pca.channels[numsPins[4]].duty_cycle = int(SpeedsMotors[1] > 0)*65535
        self.pca.channels[numsPins[5]].duty_cycle = int(SpeedsMotors[1] < 0)*65535
        
        self.pca.channels[numsPins[6]].duty_cycle = self.ListOfMotors[2].speed_shim
        self.pca.channels[numsPins[7]].duty_cycle = int(SpeedsMotors[2] > 0)*65535
        self.pca.channels[numsPins[8]].duty_cycle = int(SpeedsMotors[2] < 0)*65535
        
        self.pca.channels[numsPins[9]].duty_cycle = self.ListOfMotors[3].speed_shim
        self.pca.channels[numsPins[10]].duty_cycle = int(SpeedsMotors[3] > 0)*65535
        self.pca.channels[numsPins[11]].duty_cycle = int(SpeedsMotors[3] < 0)*65535  

    def moveServos(self, angles) -> None:
        self.servo0.angle = rad2deg(angles[0])
        self.servo1.angle = rad2deg(angles[1])
        self.servo2.angle = rad2deg(angles[2])
        self.servo3.angle = rad2deg(angles[3])
        if self.numsOfControllers > 1:
            self.servo4.angle = rad2deg(angles[4])
            self.servoGripper.angle = rad2deg(angles[5])
            self.servoCamera.angle = rad2deg(angles[6])
        print("MOVE ANG:", angles)

def main():
    A = Motor(1,   0)
    B = Motor(0,   1)
    C = Motor(-1,  0)
    D = Motor(0,  -1)
    abcd = Route(A, B, C, D)

    o = movePWM(abcd.ListOfMotors, numsOfControllers = 2)

    o.setUpServos([1,2,3,4,5,6,7])
    o.moveMotors([1,1,1,1],[1,2,3,4,5,6,7,8,9,10,11,12])
    o.moveServos([0,0,0,0,0,0,0])
    
if __name__=="__main__":
    main()