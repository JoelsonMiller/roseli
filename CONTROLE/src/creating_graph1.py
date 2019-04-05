#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from roseli.srv import CreateMap, CreateMapResponse
from roseli.srv import GetOdom, GetOdomResponse
import networkx as nx
import matplotlib.pyplot as plt
import time
import math

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.DiGraph()
		self.n_node = 0

	def distance(self):
		rospy.wait_for_service('server_get_odom')
		try:
			get_odom = rospy.ServiceProxy('server_get_odom', GetOdom)
			resp = get_odom()
			return resp.dist.x
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def graph_map(self, data):

		test_node = False
		non = self.G.number_of_nodes()
		pose = nx.get_node_attributes(self.G, 'pose_graph')
		#print(pose)
		if (data.pose2d.x == float('inf') and data.pose2d.y == float('inf') and data.pose2d.theta == float('inf')):
				#Encontra uma interseção e infere sua posição
				print("Found an intersection")
				dist_move = distance()
				data.pose2d.x = dist_move*math.cos(pose.theta[n_node - 1])
				data.pose2d.y = dist_move*math.sin(pose.theta[n_node - 1])
		for node in range(non):

			if( data.pose2d.x == pose[node].x and data.pose2d.y == pose[node].y and data.pose2d.theta == pose[node].theta):
				test_node = True
				break

		if(test_node == False):
			print ("Novo noh adicionado")
			self.G.add_node(self.n_node, pose_graph = data.pose2d, ip = data.intr_pnt_brd)
			request = 0
			self.n_node = self.n_node + 1
		else:
			request = self.G.node[node]['ip']

		return CreateMapResponse(request)

if __name__=='__main__':
	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
