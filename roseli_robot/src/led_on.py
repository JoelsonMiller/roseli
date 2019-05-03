#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Empty


def callback (data):
	state = GPIO.input(19)
	GPIO.output(19, not state)

def listener():

	rospy.init_node('led_strip', anonymous=True)
	rospy.Subscriber("led", Empty, callback)

	_state_ = rospy.get_param('~state_led', "OFF")
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(19,GPIO.OUT)

	if _state_ == "ON":
		GPIO.output(19, GPIO.HIGH)
	elif _state_ == "OFF":
		GPIO.output(19, GPIO.LOW)

	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~state_led'), _state_)
	rospy.spin()

if __name__ == '__main__':
	listener()
	GPIO.output(19, GPIO.LOW)

