#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2, cv_bridge
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
import pytesseract as ocr
from PIL import Image as imagePil
import os
import time
from roseli.srv import CreateMap, CreateMapResponse

class ReadTag:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/cropTag', Image, self.image_callback) #/cropTag
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pose_pub = rospy.Publisher('pose_twoD', String, queue_size=1)
		self.twist=Twist()
		self.string = String()
		self._pose2d_ = Pose2D()
		self.rate = rospy.Rate(1)

	def creating_map_client(self, pose2d, ip):
		rospy.wait_for_service('pose2d')
		try:
			create_map = rospy.ServiceProxy('pose2d', CreateMap)
			resp = create_map(pose2d, ip)
			return resp.intr_pnt_brd
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

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
		lowerBound=np.array([40, 0, 0]) #lower boundary of the HSV image
		upperBound=np.array([155, 60, 30]) #Upper boundary of the HSV image
		#img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img,lowerBound,upperBound,1)
		#cv2.imshow("picamera",img)
		#cv2.waitKey(1)
		kernel = np.ones((2.7,2.7), np.uint8)
		imgFilter=cv2.morphologyEx(imgThresholder, cv2.MORPH_OPEN, kernel)
		#imgThresholder=cv2.adaptiveThreshold(imgThresholder, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
		cv2.imshow("window_tag", imgFilter)
		cv2.waitKey(3)
		filename = "{}.png".format(os.getpid())
		cv2.imwrite(filename, imgFilter)
		text = ocr.image_to_string(imagePil.open(filename),config="-c tessedit_char_whitelist=1234567890.")
		os.remove(filename)
		print(text)
		separated= text.split(' ')
		self._pose2d_.x = separated[0]
		self._pose2d_.y = separated[1]
		self._pose2d_.theta = separated[2]
		_resp_ = self.creating_map_client(self._pose2d_, 0)
		if text:
			self.twist.linear.x = 0.4
			self.twist.angular.z = 0
			for x in range(0, 10):
				self.cmd_vel_pub.publish(self.twist)
				time.sleep(0.5)
#			self.twist.linear.x=0
#			self.twist.angular.z=0
#			self.string.data = "map"
#			self.pose_pub.publish(self.string)
		else:
			print ("It doesn't read a tag!")

rospy.init_node('readtag')
readtag = ReadTag()
rospy.spin()
