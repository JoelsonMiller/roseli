#!/usr/bin/env python

import rospy, numpy
from sensor_msgs.msg import Image
import cv2, cv_bridge
from geometry_msgs.msg import Twist

class Follower:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		#cv2.namedWindow("window",1)
		self.image_sub = rospy.Subscriber('camera1/image_raw',Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.twist=Twist()		

	def image_callback (self, msg):
		try:
			img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print "Deu um erro!"       
			print(e)
		imgGrayScaled=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		__, imgThresholder=cv2.threshold(imgGrayScaled,127,255, cv2.THRESH_BINARY_INV)
		
		h, w, d = img.shape
		search_top=h/2
		search_bot=search_top+20
		imgThresholder[0:search_top, 0:w]=0
		imgThresholder[search_bot:h, 0:w]=0
		M=cv2.moments(imgThresholder)
		if M['m00']>0:
			cx=int(M['m10']/M['m00'])
			cy=int(M['m01']/M['m00'])		
			cv2.circle(img, (cx, cy), 5, (0,0,255),-1)
			erro=cx-w/2
			self.twist.linear.x=0.5
			self.twist.angular.z = float(erro)/100
			self.cmd_vel_pub.publish(self.twist)
		cv2.imshow("window", img)
		cv2.waitKey(8)

rospy.init_node('follower')
follower = Follower()
rospy.spin()

