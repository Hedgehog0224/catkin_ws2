#!/usr/bin/python3

import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import rospy
from robot_pkg.msg import Servodata
from numpy import rad2deg
from kinemClasses import potateKinem

class startServs():
  start = [0,0,0,0,0,0]
  maxAcceleration = 0.5
  maxSpeed = 1
  preAngls = [0,0,0,0,0,0]
  angls = [0,0,0,0,0,0,0]    # Первый элемент - выбранный сервопривод. Остальные - углы сервоприводов.
  stTime = time.time()
  def __init__(self):
    rospy.Subscriber("manipulatorData", Servodata, startServs.callBack)
    rospy.init_node('servos')
    rospy.loginfo("INIT NODE")
    startServs.i2c = board.I2C()  # uses board.SCL and board.SDA
    # startServs.i2c1 = board.I2C()
    startServs.pca = PCA9685(startServs.i2c)
    # startServs.pca1 = PCA9685(startServs.i2c, address = 0x41)
    
    startServs.pca.frequency = 200

    startServs.servo10 = servo.Servo(startServs.pca.channels[12], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo11 = servo.Servo(startServs.pca.channels[13], actuation_range = 180, min_pulse = 500, max_pulse=2550)
    startServs.servo12 = servo.Servo(startServs.pca.channels[14], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo13 = servo.Servo(startServs.pca.channels[15], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo14 = servo.Servo(startServs.pca.channels[0], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo15 = servo.Servo(startServs.pca.channels[1], actuation_range = 180, min_pulse = 750, max_pulse=2250)
    # startServs.cameraServo = servo.Servo(startServs.pca1.channels[2], actuation_range = 180, min_pulse = 750, max_pulse=2250)

  @staticmethod
  def move(angles) -> None:
    print("MOVE ANG:", angles)
    startServs.servo10.angle = rad2deg(angles[0])
    startServs.servo11.angle = rad2deg(angles[1])
    startServs.servo12.angle = rad2deg(angles[2])
    startServs.servo13.angle = rad2deg(angles[3])
    startServs.servo14.angle = rad2deg(angles[4])
    startServs.servo15.angle = rad2deg(angles[5])
    # startServs.cameraServo.angle = 180

  @staticmethod
  def updateposition(targetPos: float, currentPos: float, speed: float, delta: float):
      err = targetPos - currentPos
      if abs(err) > 0.01:
          thisDir = (speed * speed / startServs.maxAcceleration / 2.0 >= abs(err))
          speed += startServs.maxAcceleration * delta * (thisDir * -1 if thisDir else err)
          speed = max(-startServs.maxSpeed, min(speed, startServs.maxSpeed))
          currentPos += speed * delta
      return currentPos, speed

  @staticmethod
  def calculSpeeds(targetAngle: list, startAngle: list, frequency = 200) -> list:
    """
    Расчёт плавных скоростей сервы
    """
    currentPos = [0,0,0,0,0,0]
    predCur = currentPos
    speed = [0,0,0,0,0,0]
    for i in [0, 1, 2, 3, 4, 5]:
      currentPos[i], speed[i] = startServs.updateposition(targetAngle[i], startAngle[i], 0, 1/frequency)
    bSwich = 0
    while not bSwich:
      for i in [0, 1, 2, 3, 4, 5]:
        currentPos[i], speed[i] = startServs.updateposition(targetAngle[i], currentPos[i], speed[i], 1/frequency)
      bSwich = 1
      try:
        startServs.move(currentPos)
      except:
        rospy.logerr("ERROR!!! INVILID VOLUME (move)!!! %s", currentPos)
        break

      for i in [0,1,2,3,4,5]:
        if abs(round(currentPos[i], 2) - round(targetAngle[i],2)) < 0.1: bSwich = 1 * bSwich
        else: bSwich = 0
      predCur = currentPos
      time.sleep(1/frequency)
    return currentPos

  @staticmethod
  def publis2topic() -> None:
    """
    Публикация в топик
    """
    startServs.srvData.servo0 = startServs.servo10.angle
    startServs.srvData.servo1 = startServs.servo11.angle
    startServs.srvData.servo2 = startServs.servo12.angle
    startServs.srvData.servo3 = startServs.servo13.angle
    startServs.srvData.servo4 = startServs.servo14.angle
    startServs.srvData.servo5 = startServs.servo15.angle
    # startServs.srvData.servo5 = 180 #startServs.servo15.angle
    startServs.pubServ.publish(startServs.srvData)

  @staticmethod
  def callback_mode(data) -> None:
    pass

  @staticmethod
  def callBack(data):
    startServs.angls[0] = data.servo0
    startServs.angls[1] = data.servo1
    startServs.angls[2] = data.servo2
    startServs.angls[3] = data.servo3
    startServs.angls[4] = 0.5
    startServs.angls[5] = 0.5

    ObCalul = potateKinem()
    target = ([startServs.angls[0]/100,
              startServs.angls[1]/100,
              startServs.angls[2]/100])
    if not (type(target) == None):
        target = ObCalul.computePolar(target[0], target[1], target[2])
        target = [abs(target[0]), abs(target[1]), abs(target[2]+0.5), 0,0,0]
        
        if( abs(round(target[0], 1) -  round(startServs.start[0], 1)) <= 0.1 and
            abs(round(target[1], 2) ==  round(startServs.start[1], 2)) <= 0.1 and
            abs(round(target[2], 1) ==  round(startServs.start[2], 1)) <= 0.1):
          target = [abs(target[0]), abs(target[1]), abs(target[2]+0.5), 0,2,1]
    else:
        target = [0,0,0,0,0,0]
    startServs.start = startServs.calculSpeeds(target, startServs.start)

def main():
  Ob = startServs()

  rospy.sleep(0.01)
  rospy.spin()

if __name__ == "__main__":
  main()