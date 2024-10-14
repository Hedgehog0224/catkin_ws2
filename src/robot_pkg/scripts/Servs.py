#!/usr/bin/python3

import RPi.GPIO as GPIO
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import rospy
from robot_pkg.msg import servodata, xy
from sensor_msgs.msg import Joy
from numpy import rad2deg

class startServs():
  maxAcceleration = 0.1
  maxSpeed = 0.2
  preAngls = [0,0,0,0,0,0]
  angls = [0,0,0,0,0,0,0]    # Первый элемент - выбранный сервопривод. Остальные - углы сервоприводов.
  stTime = time.time()
  def __init__(self):
    startServs.i2c = board.I2C()  # uses board.SCL and board.SDA
    startServs.pca = PCA9685(startServs.i2c)
    startServs.pca.frequency = 100

    startServs.servo10 = servo.Servo(startServs.pca.channels[12], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo11 = servo.Servo(startServs.pca.channels[13], actuation_range = 180, min_pulse = 500, max_pulse=2550)
    startServs.servo12 = servo.Servo(startServs.pca.channels[14], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    startServs.servo13 = servo.Servo(startServs.pca.channels[15], actuation_range = 180, min_pulse = 450, max_pulse=2750)
    #startServs.servo14 = servo.Servo(startServs.pca.channels[16], actuation_range = 180, min_pulse = 750, max_pulse=2250)
    #startServs.servo15 = servo.Servo(startServs.pca.channels[17], actuation_range = 180, min_pulse = 750, max_pulse=2250)
    rospy.init_node('Servos')
    rospy.Subscriber("joy", Joy, self.callback_joy)

  @staticmethod
  def move(angles) -> None:
    print("MOVE ANG:", angles)
    startServs.servo10.angle = rad2deg(angles[0])
    startServs.servo11.angle = rad2deg(angles[1])
    startServs.servo12.angle = rad2deg(angles[2])
    startServs.servo13.angle = rad2deg(angles[3])
    # startServs.servo14.angle = angles[4]
    # startServs.servo15.angle = angles[5]

  @staticmethod
  def updateposition(targetPos: float, currentPos: float, speed: float, delta: float):
      print("updateposition")
      err = targetPos - currentPos
      if abs(err) > 0.1:
          thisDir = (speed * speed / startServs.maxAcceleration / 2.0 >= abs(err))
          speed += startServs.maxAcceleration * delta * (thisDir * -1 if thisDir else err)
          speed = max(-startServs.maxSpeed, min(speed, startServs.maxSpeed))
          currentPos += speed * delta
      return currentPos, speed

  @staticmethod
  def calculSpeeds(targetAngle: list, startAngle: list, frequency = 10) -> list:
    """
    Расчёт плавных скоростей сервы
    """
    print("calculSpeeds")
    currentPos = [0,0,0,0.0,0]
    speed = [0,0,0,0,0,0]
    for i in [0, 1, 2, 3, 4, 5]:
      print(i, ":", startServs.updateposition(targetAngle[i], startAngle[i], 0, 1/frequency))
      currentPos[i], speed[i] = startServs.updateposition(targetAngle[i], startAngle[i], 0, 1/frequency)
    bSwich = 0
    while not bSwich:
      print("1 currentPos:", currentPos)
      print("speed:", speed)
      for i in [0, 1, 2, 3, 4, 5]:
        currentPos[i], speed[i] = startServs.updateposition(targetAngle[i], currentPos[i], speed[i], 1/frequency)
      print("2 currentPos:", currentPos)
      print("speed:", speed)
      startServs.move(currentPos)
      print("CurrentPos:", currentPos)
      print("TargetAngle", targetAngle)
      bSwich = 1
      for i in [0,1,2,3,4,5]:
        if round(currentPos[i], 0) == targetAngle[i]: bSwich = 1 * bSwich
        else: bSwich = 0
      time.sleep(1/frequency)

  @staticmethod
  def callback_joy(data) -> None:
    """
    Получение данных с джойстика, их обработка
    """
    print(startServs.angls[0])
    print("Pre:", startServs.preAngls)
    print("Tar:", startServs.angls)

    print("calback")
    if data.buttons[0] == 1:
      startServs.angls[0] = 1
      startServs.stTime = time.time()
    elif data.buttons[1] == 1:
      startServs.angls[0] = 2
      startServs.stTime = time.time()
    elif data.buttons[2] == 1:
      startServs.angls[0] = 3
      startServs.stTime = time.time()
    elif data.buttons[3] == 1:
      startServs.angls[0] = 4
      startServs.stTime = time.time()
    elif time.time() - startServs.stTime > 10:
      startServs.angls[0] = 0

    if startServs.angls[0] == 0:
      pass
    else:
      if not (data.axes[1] == 0):
          print("Pre:", startServs.preAngls[startServs.angls[0]+1]," Angl:" ,startServs.angls[startServs.angls[0]])
          startServs.preAngls[startServs.angls[0]+1] = startServs.angls[startServs.angls[0]]
          startServs.angls[startServs.angls[0]] = startServs.angls[startServs.angls[0]] + 0.0698
          print("+")
      # elif data.axes[1] < 0:
      #     startServs.preAngls[startServs.angls[0]+1] = startServs.angls[startServs.angls[0]]
      #     startServs.angls[startServs.angls[0]] = startServs.angls[startServs.angls[0]] - 0.0698
      #     print("-")
          startServs.calculSpeeds(startServs.angls[1:7], startServs.preAngls)

  @staticmethod
  def publis2topic() -> None:
    """
    Публикация в топик
    """
    startServs.srvData.servo0 = startServs.servo10.angle
    startServs.srvData.servo1 = startServs.servo11.angle
    startServs.srvData.servo2 = startServs.servo12.angle
    startServs.srvData.servo3 = startServs.servo13.angle
    startServs.srvData.servo4 = 0 #startServs.servo14.angle
    startServs.srvData.servo5 = 0 #startServs.servo15.angle
    startServs.pubServ.publish(startServs.srvData)

  @staticmethod
  def callback_mode(data) -> None:
    pass

def main():
  Ob = startServs()
  # Ob.move()
  rospy.spin()

if __name__=="__main__":
  main()
