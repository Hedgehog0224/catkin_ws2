#!/usr/bin/python3

import cv2
from matplotlib import pyplot as plt
import numpy as np
import argparse
from robot_pkg.msg import servodata
import rospy
# from sensor_msgs.msg import LaserScan
import random
#version = "1.02"

class Camera():
   def __init__(self):
      self.predData = [0,0,0,0,0,0]
      # rospy.init_node('cameraNode')
      self.pub = rospy.Publisher('cameraData', servodata, queue_size=10)
      self.msg = servodata()

      self.height = 800
      self.weight = 800
      self.colorBorders =[["red",  np.array([150,50,10]),    np.array([190,255,255])],
                          ["blue", np.array([100,100,0]),    np.array([125,255,255])],
                          ["yellow", np.array([20,165,100]), np.array([60,255,255])]]
      self.lower = np.array([150,60,50])
      self.upper = np.array([20,255,255])
      self.cam = cv2.VideoCapture(0)
      self.res = None
      #self.setDistMask("res")
   def show(self, nameWindow, img):
      cv2.imshow(nameWindow, img)
   def close(self):
      self.cam.release()
      cv2.destroyAllWindows()
   def onTestTrackbar(self, val):
      pass

   def getCoordColorContours(self, contours):
      for c in contours:
         rect  = cv2.boundingRect(c)
         if rect[2] < 40 or rect[3] < 40: continue
         x, y, w, h = rect
         return x, y, x + w, y + h
   def setDistMask(self, nameWindow):
      cv2.namedWindow(nameWindow)
      cv2.createTrackbar("dist", nameWindow, 0, 1000, self.onTestTrackbar)
   def setColorMask(self, nameWindow):
      cv2.namedWindow(nameWindow)
      cv2.createTrackbar("h_lowwer", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.createTrackbar("s_lowwer", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.createTrackbar("v_lowwer", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.createTrackbar("h_upper", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.createTrackbar("s_upper", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.createTrackbar("v_upper", nameWindow, 0, 255, self.onTestTrackbar)
      cv2.setTrackbarPos("h_lowwer", nameWindow, 125)
      cv2.setTrackbarPos("s_lowwer", nameWindow, 25)
      cv2.setTrackbarPos("v_lowwer", nameWindow, 0)
      cv2.setTrackbarPos("h_upper", nameWindow, 190)
      cv2.setTrackbarPos("s_upper", nameWindow, 255)
      cv2.setTrackbarPos("v_upper", nameWindow, 255)
   def distanceCalibration(self, dist, name):
      return cv2.getTrackbarPos("dist", name)
   def colorCalibration(self, colorName, name):
      h_l = cv2.getTrackbarPos("h_lowwer", name)
      s_l = cv2.getTrackbarPos("s_lowwer", name)
      v_l = cv2.getTrackbarPos("v_lowwer", name)
      h_u = cv2.getTrackbarPos("h_upper", name)
      s_u = cv2.getTrackbarPos("s_upper", name)
      v_u = cv2.getTrackbarPos("v_upper",name)
      return np.array([h_l, s_l, v_l]), np.array([h_u, s_u, v_u])
   def readData(self):
      ret, img = self.cam.read()
      img = cv2.flip(img, 0)
      self.res = img.copy()
      return img

   def getHSVimg(self, img):
      return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   def getMask(self, color, img):
      if color == "red":
         self.lower = self.colorBorders[0][1]
         self.upper = self.colorBorders[0][2]
      elif color == "blue":
         self.lower = np.array([100, 100,  0])
         self.upper = np.array([125, 255, 255])
      self.lower, self.upper = self.colorCalibration("red", "res")
      res = img.copy()
      color_img  = self.getHSVimg(img)
      color_mask =  cv2.inRange(color_img, self.lower, self.upper)
      res        =  cv2.bitwise_and(res, res, mask=color_mask)
       
      contours, hierarchy = cv2.findContours(color_mask, 1, 2)
      try:
         cnt = contours[0]
         M = cv2.moments(cnt)

         rect = cv2.minAreaRect(cnt)
         box  = cv2.boxPoints(rect)
         box  = np.int0(box)
         a1, a2, b1, b2 = self.getCoordColorContours(contours)
         #print((a1+a2)/2,(b1+b2)/2)
         cv2.rectangle(res, (a1,a2), (b1,b2), (0,255,0), 2)
      except:
         pass
      return color_mask, res
   def setMask(self, color, img, heightConst):
      #print(self.colorBorders[0][0])
      if color == "red":
         self.lower = self.colorBorders[0][1]
         self.upper = self.colorBorders[0][2]
      elif color == "blue":
         self.lower = self.colorBorders[1][1]
         self.upper = self.colorBorders[1][2]
      elif color == "yellow":
         #self.lower, self.upper = self.colorCalibration("test", "res")
         self.lower = self.colorBorders[2][1]
         self.upper = self.colorBorders[2][2]
      color_img  = self.getHSVimg(img)
      color_mask =  cv2.inRange(color_img, self.lower, self.upper)
      #res        =  cv2.bitwise_and(res, res, mask=color_mask)
      contours, hierarchy   = cv2.findContours(color_mask, 1, 2)
      centerX, centerY = 10, 50
      a1, a2, b1, b2 = 0, 0, 0, 0 
      try:
         a1, a2, b1, b2 = self.getCoonturs(contours)
         centerX = int((a1+b1)/2)
         centerY = int((a2+b2)/2)
         #print(a1, a2, b1, b2, sep=" ")
         cv2.rectangle(self.res, (a1,a2), (b1,b2), (0,255,0), 2)
      except:
         pass
      dist, h, w = self.getDistation(a1, a2, b1, b2, heightConst)
      dist = round(dist,2)
      cv2.putText(self.res, 'd' + str(dist) + 'mm w' + str(h),
                 (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX,
                  1, (255,255,255))
      
      return color_mask, dist, np.array([centerX, centerY])
   def getRes(self):
      return self.res
   def getDistation(self, a1, a2, b1, b2, weightC):
      #height = 100px, heightTrue = 26 мм, distantion ~ 200 мм  
      height = round((abs(b1 - a1) + abs(b2 - a2))/2,0)
      weight = round((abs(a2 - a1) + abs(b2 - b1))/2,0)
      distantion = 0
      try:
         distantion = (100/height)*200
         #distantion = (100/weight)*200
      except:
         pass
      return distantion, height, weight

   def getCoonturs(self, contours):
      a1, a2, b1, b2 = self.getCoordColorContours(contours)
      return a1, a2, b1, b2
   def paramOnDistationAndPos(self, dist, centers):
      #random.random(150,360)
      #weight 640 height 480
      rMinVector, rMaxVector = 150, 360
      currentDist = dist/250 * rMaxVector
      if(currentDist < 150 ):
         currentDist = 150
      if(currentDist > 360):
         currentDist = 360

      centerX, centerY = centers[0], centers[1]
      currentPhi = [0,0,0,0,0,0,0,0,0,0]
      # for i in range(10):
      currentPhi = 0.00127 * centerX + 0.76
      rPhiMin, rPhiMax = 0, 1
      # currentPhi = sum(currentPhi) / 10
      # if( currentPhi > 2.2):
      #    currentPhi = 2.2
      # if currentPhi < 0.8:
      #    currentPhi = 0.8
      print("centerX = ", centerX)

      print( " Phi = ", currentPhi, " r = ", currentDist)
      # rThetaMin, rThetaMax = 0.5, 1.57
      # currentThera = centerY / 240
      # print("centerY = ", centerY)
      # if( currentThera > 2):
      #    currentThera = 2
      # if currentThera < 0.5:
      #    currentThera = 0.5

      vectors = np.array([320,
                          0,
                          currentPhi,0,0,0])
                          
      # vectors = np.array([random.uniform(300,360),
      #                     random.uniform(1.57,3.14),
      #                     random.uniform(0,3.14),
      #                     0,0,0])
      return vectors

   # @staticmethod
   def pubData(self):
      while not rospy.is_shutdown():
         # rospy.loginfo("Send loop")
         img = self.readData()
         img_test, dist, centers = self.setMask("yellow",img, 100)
         arr = self.paramOnDistationAndPos(dist, centers)
         if not(all(self.predData == arr)):
            self.msg.servo0 = int(arr[0]*100)
            self.msg.servo1 = int(arr[1]*100)
            self.msg.servo2 = int(arr[2]*100)
            self.msg.servo3 = int(arr[3]*100)
            self.msg.servo4 = int(arr[4]*100)
            self.msg.servo5 = int(arr[5]*100)
            # print("SEND", arr)
            try:
               self.pub.publish(self.msg)
               # print("Pub sucsses")
            except:
               print("ERROR")
            self.predData = arr
         rospy.sleep(0.4)

rospy.init_node('cameraNode')
cam = Camera()
# rospy.loginfo("cam create")
# rospy.loginfo(rospy.is_shutdown())
cam.pubData()

# cam.close()
