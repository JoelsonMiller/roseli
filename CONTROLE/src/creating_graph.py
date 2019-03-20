#!/usr/bin/env python

import rospy
from roseli.srv import CreateMap, CreateMapResponse
import networkx as nx
import matplotlib.pyplot as plt
import time

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('pose2D', CreateMap, self.graph_map)
		self.G = nx.DiGraph()

	def graph_map(self, data):
		if (not self.G.has_node(data.pose2d)):
			self.G.add_node(data.pose2d, ip=data.intr_pnt_brd)
			request = 0
		name = self.G.node
		print(name)
		return CreateMapResponse(request)


if __name__=='__main__':
	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
