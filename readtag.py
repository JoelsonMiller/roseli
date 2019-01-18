#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2, cv_bridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pytesseract as ocr
from PIL import Image as imagePil
import os

class ReadTag:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/cropTag', Image, self.image_callback) #/cropTag
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pose_pub = rospy.Publisher('pose_twoD', String, queue_size=1)
		self.twist=Twist()
		self.string = String()
		self.rate = rospy.Rate(1)

	def image_callback (self, msg):
		self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
		self.rate.sleep()
		try:
			img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print ("Error: Imagem da Tag nao recebida")
			print(e)
		lowerBound=np.array([65,45,40]) #lower boundary of the HSV image
		upperBound=np.array([100,75,80]) #Upper boundary of the HSV image
		#img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img,lowerBound,upperBound,1)
		#cv2.imshow("picamera",img)
		#cv2.waitKey(1)
		kernel = np.ones((2.5,2.5), np.uint8)
		imgFilter=cv2.morphologyEx(imgThresholder, cv2.MORPH_OPEN, kernel)
		#imgThresholder=cv2.adaptiveThreshold(imgThresholder, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
		cv2.imshow("window_tag", imgFilter)
		cv2.waitKey(3)
		filename = "{}.png".format(os.getpid())
		cv2.imwrite(filename, imgThresholder)
		text = ocr.image_to_string(imagePil.open(filename))
		os.remove(filename)
		print(text)
		if text:
			self.twist.linear.x=0.2
			self.twist.angular.z=0
			self.cmd_vel_pub.publish(self.twist)
			self.rate.sleep()
			self.rate.sleep()
#			self.twist.linear.x=0
#			self.twist.angular.z=0
#			self.string.data = "map"
#			self.pose_pub.publish(self.string)
		else:
			print ("It doesn't read a tag!")

rospy.init_node('readtag')
readtag = ReadTag()
rospy.spin()
