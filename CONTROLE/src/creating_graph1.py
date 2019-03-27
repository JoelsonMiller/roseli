#!/usr/bin/env python

import rospy
from roseli.srv import CreateMap, CreateMapResponse
from roseli.srv import GetOdom, GetOdomResponse
import networkx as nx
import matplotlib.pyplot as plt
import time
import math

global n_node

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.DiGraph()

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
		pose = nx.get_node_attributes(G, 'pose_graph')
		
		#Verifica se o nó já existe
		if all(x == float('inf') for x in (data.pose2d.x, data.pose2d.y, data.pose2d.theta)):
				#Encontra uma interseção e infere sua posição
				dist_move = distance()	
				data.pose2d.x = dist_move*math.cos(pose.theta[n_node - 1])
				data.pose2d.y = dist_move*math.sin(pose.theta[n_node - 1])
		
		for node in range(non):
				
			if( data.pose2d.x == pose.x[node] and data.pose2d.y == pose.y[node] and data.pose2d.theta == pose.theta[node]):
				test_node = True
				break

		if(test_node == False):
			print ("Novo noh adicionado")
			self.G.add_node(n_node, pose_graph = data.pose2d, ip = data.intr_pnt_brd)
			request = 0
			n_node = n_node + 1
		else:
			request = G.node[node]['ip']

		return CreateMapResponse(request)

if __name__=='__main__':
	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
