#!/usr/bin/env python

import rospy
from roseli.srv import CreateMap, CreateMapRequest
from geometry_msgs.msg import Pose2D
import time

class simClient:

	def __init__(self):
                print("Programa se iniciou...")
		self.pose = Pose2D()
		self.simulator()

	def creating_map_client(self, pose2d, ip):

                rospy.wait_for_service('/pose2D')

                try:
                        create_map = rospy.ServiceProxy('/pose2D', CreateMap)
                        resp = CreateMapRequest(pose2d, ip)
                        return create_map(resp)
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

	def simulator(self):
		ip = {}
		pose2d = {}
		self.pose.x = 100.0
		self.pose.y = 0.0
		self.pose.theta = 90.0
		ip[0] = 0
		pose2d[0] = self.pose

		self.pose.x = 100.0
		self.pose.y = 100.0
		self.pose.theta = 90.0
		ip[1] = 0
		pose2d[1] = self.pose

		self.pose.x = 200.0
		self.pose.y = 100.0
		self.pose.theta = 90.0
		ip[2] = 1
		pose2d[2] = self.pose

		'''self.pose[3].x = 200.0
		self.pose[3].y = 200.0
		self.pose[3].theta = 90.0
		ip[3] = 0

		self.pose[4].x = 300.0
		self.pose[4].y = 200.0
		self.pose[4].theta = 90.0
		ip[4] = 0

		self.pose[5].x = 300.0
		self.pose[5].y = 100.0
		self.pose[5].theta = 270.0
		ip[5] = 1

		self.pose[6].x = 300.0
		self.pose[6].y = 0.0
		self.pose[6].theta = 270.0
		ip[6] = 0

		self.pose[7].x = 200.0
		self.pose[7].y = 0.0
		self.pose[7].theta = 270.0
		ip[7] = 1'''

		for x in range(3):
			_resp_ = self.creating_map_client(pose2d[x], ip[x])
			time.sleep(1)
		print("O programa terminou")              

if __name__=='__main__':
        try:
                rospy.init_node('client_test')
                sim = simClient()
                rospy.spin()
        except rospy.ROSInterruptException:
                pass

