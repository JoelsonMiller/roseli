#!/usr/bin/env python
import sys
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np 
import rospy
from std_msgs.msg import String

# initialize the camera and grab a reference to the raw camera capture
def raspicam():
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 64
	rawCapture = PiRGBArray(camera, size=(640, 480))

	#pub = rospy.Publisher('chatter', String, queue_size=10)
	#rospy.init_node('talker', anonymous=True)

	#rate = rospy.Rate(10)

	#Lower and Upper Limits
	lowerBound=np.array([0,100,100],np.uint8)
	upperBound=np.array([10,255,255],np.uint8)
 
	# allow the camera to warmup
	time.sleep(0.1)
 
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array
		hsvimage=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask=cv2.inRange(image,lowerBound, upperBound)
		# show the frame
		#cv2.imshow("HSV",hsvimage)
		#cv2.imshow("Binary",mask)
		cv2.imshow("Frame", image)
		#hello_str = "hello world %s" % rospy.get_time()
		#pub.publish(hello_str)
		#rate.sleep()
		key = cv2.waitKey(1) & 0xFF
 
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
 
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break


if __name__ == "__main__":
        try:
                raspicam()
        except rospy.ROSInterruptException:
                pass

