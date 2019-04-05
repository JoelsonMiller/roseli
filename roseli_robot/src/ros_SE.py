#!/usr/bin/env python

import primitivas
import time
import rospy
from std_msgs.msg import String 


def blink(i):
	if i == 1:
		primitivas.led(0,8)
        	#time.sleep(0.5) # valor em segundos            
               # primitivas.led(1,8)
        	#time.sleep(0.5)
	else:
	#	print 'i am here'
		primitivas.led(1,8)

def callback(sign):
	if sign.data == 'IMAGE DETECT':
		blink(1)
	else:
		blink(0)

def seeandblink():
	rospy.init_node('SE', anonymous = True)
	rospy.Subscriber('Target', String, callback)
	rospy.spin()	
 
if __name__ == "__main__":
	try:
		seeandblink()
	except rospy.ROSInterruptException:
		pass
	
