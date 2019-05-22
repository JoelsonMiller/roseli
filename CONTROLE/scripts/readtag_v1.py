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
from roseli.srv import TagImage, TagImageResponse
from roseli.srv import ResetEnc, ResetEncRequest
from dynamic_reconfigure.server import Server
from roseli.cfg import ocr_tagConfig
import re

class ReadTag:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.twist=Twist()
		self.image_server = rospy.Service('/cropTag', TagImage, self.image_callback) #/cropTag
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.range_param = Server(ocr_tagConfig, self.reconfigure)
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

	def reset_enc_func(self):

                rospy.wait_for_service('/reset_enc_server')

                try:
                        reset = rospy.ServiceProxy('/reset_enc_server', ResetEnc)
                        resp = ResetEncRequest()
                        return reset(resp)
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

	def image_callback (self, msg):
		self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
		self.rate.sleep()
		try:
			img = self.bridge.imgmsg_to_cv2(msg.tag, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print ("Error: Imagem da Tag nao recebida")
			print(e)

		img = cv2.resize(img, None, fx=2, fy=2)
		img_Gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    		img_Gray = cv2.bilateralFilter(img_Gray, 5, 20, 20)

		#cv2.imshow('grayscale_image', img_Gray)
		#cv2.waitKey(500)
		kernel = np.ones((30, 30), np.uint8)
		kernel_1 = np.ones((2, 2), np.uint8)
		kernel_2 = np.ones((3, 3), np.uint8)
		imgFilter_ATGC = cv2.adaptiveThreshold(img_Gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 25, 12)

		#cv2.imshow('Threshold Image', imgFilter_ATGC)
		#cv2.waitKey(2000)
		imgFilter_ATGC = cv2.morphologyEx(imgFilter_ATGC, cv2.MORPH_CLOSE, kernel_2)
		#imgFilter_ATGC = cv2.morphologyEx(imgFilter_ATGC, cv2.MORPH_DILATE, kernel_1)

		#cv2.imshow('Primeiro Filtro', imgFilter_ATGC)
		#cv2.waitKey(2000)

		lowerBound1=np.array([self.min_h, self.min_s, self.min_v]) #lower boundary of the HSV image
		upperBound1=np.array([self.max_h, self.max_s, self.max_v]) #Upper boundary of the HSV image
		img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img_HSV,lowerBound1,upperBound1,1)
		imgFilter_IR=cv2.morphologyEx(imgThresholder, cv2.MORPH_DILATE, np.ones((30, 30), np.uint8))	
		imgFilter_IR=cv2.morphologyEx(imgFilter_IR, cv2.MORPH_ERODE, np.ones((17, 17), np.uint8))	

		#cv2.imshow('Segundo Filtro', imgFilter_IR)
		#cv2.waitKey(2000)

		output = cv2.bitwise_or(imgFilter_ATGC, cv2.bitwise_not(imgFilter_IR))
		#cv2.imshow('OUTPUT', output)
	#	cv2.waitKey(2000)

		filename = "{}.png".format(os.getpid())
		cv2.imwrite(filename, output)
		text = ocr.image_to_string(imagePil.open(filename),config="-c tessedit_char_whitelist=1234567890. --user-patterns patterns.txt")
		os.remove(filename)		
		print(text)
		separated= text.split(' ')

		if (not len(separated) == 3):
			rospy.logerr("It doesn't read a tag!")
			return TagImageResponse
		
		else:
			for i in range(len(separated)):
				matchObj = re.match("\d\d\d.\d", separated[i])				
				if (matchObj == None):
					rospy.logerr("It doesn't read a tag!")
					return TagImageResponse
				
			if(not 0.0 <= float(separated[2]) <= 360.0):
				rospy.logerr("It doesn't read a tag!")
				return TagImageResponse
			
			self._pose2d_.x = float(separated[0])
			self._pose2d_.y = float(separated[1])
			self._pose2d_.theta = float(separated[2])

			_resp_ = self.creating_map_client(self._pose2d_, 0)
			flag = self.reset_enc_func()

			self.twist.linear.x = 0.2
			self.twist.angular.z = 0
			for x in range(0, 4):
				self.cmd_vel_pub.publish(self.twist)
				time.sleep(0.5)
		return TagImageResponse()

if __name__=='__main__':
	try:
		rospy.init_node('readtag')
		readtag = ReadTag()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
