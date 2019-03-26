#!/usr/bin/env python

import rospy
from roseli.srv import CreateMap, CreateMapResponse
import networkx as nx
import matplotlib.pyplot as plt
import time


global n_node

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.DiGraph()

	def graph_map(self, data):

		test_node = False
		non = self.G.number_of_nodes()

		for node in range(non):
			pose = nx.get_node_attributes(G, 'pos_graph')
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
