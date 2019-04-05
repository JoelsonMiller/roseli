#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2, cv_bridge
from geometry_msgs.msg import Twist
import pytesseract as ocr
import std_msgs.msg import String
from PIL import Image as imagePil
import os

class ReadTag:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('cropTag', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pose_pub = rospy.Publisher('2Dpose', String, queue_size=1)
		self.twist=Twist()
		self.string=String()
		self.rate = rospy.Rate(1)		

	def image_callback (self, msg):
		try:
			img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print ("Deu um erro!")
			print(e)
		lowerBound=np.array([110,50,50]) #lower boundary of the HSV image
		upperBound=np.array([130,255,255]) #Upper boundary of the HSV image
		img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img_HSV,lowerBound,upperBound,1)
		cv2.imshow("window_tag", imgThresholder)
		cv2.waitKey('c')
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
			self.twist.linear.x=0
			self.twist.angular.z=0
			self.string.data = "Mapa"
		else:
			print ("It doesn't read a tag!")

rospy.init_node('readtag')
readtag = ReadTag()
rospy.spin()
