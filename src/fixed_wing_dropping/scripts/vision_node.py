#!/usr/bin/env python
 
from ctypes import sizeof
from turtle import distance, up
from std_msgs.msg import Int64
import rospy
import numpy as np
import cv2
  
def main():
  rospy.init_node('vision')
  
  pub = rospy.Publisher('vision/error_dist', Int64, queue_size=10)
  cap = cv2.VideoCapture("/home/arise/Dev/ros/teknofest_2022_ws/src/fixed_wing_dropping/vid/vid.mp4")
  while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
      lower = np.array([0, 120, 20], dtype = "uint8")
      upper = np.array([25, 255, 255], dtype = "uint8")
      
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower, upper)

      im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      width = np.size(frame, 1)

      bigContours = []
      for cnt in contours:
        if cv2.contourArea(cnt) > 2200:
          bigContours.append(cnt)

      # cv2.drawContours(frame, bigContours, -1, (0,255,0), 3)
      print("Jumlah contour: ", len(bigContours))
      

      centerOfContour = []
    
      cv2.line(frame,(1920 / 2, 0),(1920 / 2, 1080), (0, 255, 255), 2)

      distance = 0

      for bigCnt in bigContours:
        M = cv2.moments(bigCnt)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        distance = cX - (1920 / 2)

        cv2.line(frame,(cX, cY),(1920 / 2, cY), (0, 255, 255), 2)
        distanceTextX = cX + ((960 - cX) / 2) if cX < 960 else cX - ((cX - 960) / 2)

        cv2.circle(frame, (cX, cY), 7, (0, 255, 255), -1)
        cv2.putText(frame, "center", (cX - 20, cY - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

      pub.publish(distance)

      cv2.imshow("image", frame)
      if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    else:
      break
         
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass