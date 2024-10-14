import cv2
from matplotlib import pyplot as plt
import numpy as np
import argparse

#version = "1.0a"
cam = cv2.VideoCapture(0)

def on_test_trackbar(val):
    pass

def get_coord_contours(contours):
   for c in contours:
      rect  = cv2.boundingRect(c)
      if rect[2] < 100 or rect[3] < 100: continue
      x, y, w, h = rect
      return x, y, x + w, y + h

img1 = np.zeros((300,512,3), np.uint8)
cv2.namedWindow("red_mask")

cv2.createTrackbar("rang_value", "red_mask", 0, 255, on_test_trackbar)

while(1):
   ret, img = cam.read()
   img_gray  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   res = img.copy()

   speed = cv2.getTrackbarPos("rang_value", "red_mask")
   #parog red
   lower = np.array([0,  60,  50])
   upper = np.array([20, 255, 255])

   # mask
   red_mask = cv2.inRange(img_gray, lower, upper)
   res   = cv2.bitwise_and(res, res, mask=red_mask)

   contours, hierarchy = cv2.findContours(red_mask, 1, 2)
   cnt = contours[0]
   M = 	cv2.moments(cnt)
   #if (int(M['m00']) != 0):
   #   print( int(M['m10']/M['m00']), int(M['m01']/M['m00']))
   rect = cv2.minAreaRect(cnt)
   box = cv2.boxPoints(rect)
   box = np.int0(box)
   a1, a2, b1, b2  = get_coord_contours(contours)
   cv2.rectangle(res, (a1,a2), (b1,b2), (0, 255, 0),2)
   cv2.imshow("red_mask", red_mask)

   cv2.imshow("res", res)
   k = cv2.waitKey(30);
   if k == 27:
      break
cam.release() 
cv2.destroyAllWindows()
