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
		lowerBound1=np.array([100, 150, 0]) #lower boundary of the HSV image
		upperBound1=np.array([140, 255, 255]) #Upper boundary of the HSV image
		#lowerBound2=np.array([160, 50, 50]) #lower boundary of the HSV image
                #upperBound2=np.array([170, 255, 255]) #Upper boundary of the HSV image
		img_HSV=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		imgThresholder=cv2.inRange(img_HSV,lowerBound1,upperBound1,1)
		#imgThresholder2=cv2.inRange(img,lowerBound2,upperBound2,1)
		#imgThresholder =  imgThresholder1|imgThresholder2
		#cv2.imshow("picamera",img)
		#cv2.waitKey(1)
		#kernel = np.ones((2, 2), np.uint8)
		#imgFilter=cv2.morphologyEx(imgThresholder, cv2.MORPH_OPEN, kernel)
		#imgThresholder=cv2.adaptiveThreshold(imgThresholder, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
		cv2.imshow("window_tag", imgThresholder)
		cv2.waitKey(3)
		filename = "{}.png".format(os.getpid())
		cv2.imwrite(filename, imgThresholder)
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
