#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from roseli.srv import *
from std_msgs.msgs import Float32

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def server():
	rospy.init_node('sonar_hc_sr04_server')
	pub = rospy.Publisher('sonar_distance', Float32, queue_size = 10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# set Trigger to HIGH
		GPIO.output(GPIO_TRIGGER, True)
	    # set Trigger after 0.01ms to LOW
		time.sleep(0.00001)
		GPIO.output(GPIO_TRIGGER, False)
		StartTime = time.time()
		StopTime = time.time()
		
		while GPIO.input(GPIO_ECHO)==0:
			StartTime = time.time()
			
		while GPIO.input(GPIO_ECHO)==1:
			StopTime = time.time()
		    # time difference between start and arrival
			TimeElapsed = StopTime - StartTime
		    # multiply with the sonic speed (34300 cm/s)
		    # and divide by 2, because there and back
			distance = (TimeElapsed * 34300) / 2
			pub.publish(distance)
			rate.sleep()

if __name__ == '__main__':
	try:
		server()
		time.sleep(1)
        	# Reset by pressing CTRL + C
	except KeyboardInterrupt:
		print("Measurement stopped by User")
	except rospy.ROSInterruptException:
		print("Something happened with ROS communication")
	finally:
		print("Cleanning at all")
		GPIO.cleanup()
