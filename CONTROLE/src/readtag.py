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
from roseli.srv import CreateMap, CreateMapRequest
from dynamic_reconfigure.server import Server
from roseli.cfg import ocr_tagConfig

class ReadTag:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/cropTag', Image, self.image_callback) #/cropTag
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pose_pub = rospy.Publisher('pose_twoD', String, queue_size=1)
		self.range_param = Server(ocr_tagConfig, self.reconfigure)
		self.twist=Twist()
		self.string = String()
		self._pose2d_ = Pose2D()
		self.rate = rospy.Rate(1)

	def reconfigure(self, config, level):
		#print(config)
		self.min_h = config.min_hue_ocr
		self.min_s = config.min_saturation_ocr
		self.min_v = config.min_value_ocr
		self.max_h = config.max_hue_ocr
		self.max_s = config.max_saturation_ocr
		self.max_v = config.max_value_ocr
		return config

	def creating_map_client(self, pose2d, ip):

		rospy.wait_for_service('/pose2D')

		try:
			create_map = rospy.ServiceProxy('/pose2D', CreateMap)
			resp = CreateMapRequest(pose2d, ip)
			return create_map(resp)
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

		lowerBound1=np.array([self.min_h, self.min_s, self.min_v]) #lower boundary of the HSV image
		
		upperBound1=np.array([self.max_h, self.max_s, self.max_v]) #Upper boundary of the HSV image
		
		img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img_HSV,lowerBound1,upperBound1,1)

		#cv2.imshow('picamera', img)
		#cv2.waitKey(500)
		kernel = np.ones((3, 3), np.uint8)
		imgFilter=cv2.morphologyEx(imgThresholder, cv2.MORPH_DILATE, kernel)
		#imgFilter=cv2.adaptiveThreshold(imgThresholder, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 1)

		cv2.imshow('window_tag', imgFilter)
		cv2.waitKey(500)

		filename = "{}.png".format(os.getpid())
		cv2.imwrite(filename, imgFilter)
		text = ocr.image_to_string(imagePil.open(filename),config="-c tessedit_char_whitelist=1234567890.")
		os.remove(filename)
		print(text)
		separated= text.split(' ')

		if (not len(separated) == 3):
			print("It doesn't read a tag!")
			return
		else:
			self._pose2d_.x = float(separated[0])
			self._pose2d_.y = float(separated[1])
			self._pose2d_.theta = float(separated[2])
			_resp_ = self.creating_map_client(self._pose2d_, 0)

			self.twist.linear.x = 0.4
			self.twist.angular.z = 0
			for x in range(0, 10):
				self.cmd_vel_pub.publish(self.twist)
				time.sleep(0.5)


rospy.init_node('readtag')
readtag = ReadTag()
rospy.spin()
