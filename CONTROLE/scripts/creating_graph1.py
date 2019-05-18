#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from roseli.srv import CreateMap, CreateMapResponse
from roseli.srv import GetOdom, GetOdomResponse
import networkx as nx
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import time
import math
import getpass

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.Graph()
		self.n_node = 0

	def distance(self):
		rospy.wait_for_service('odom_server')
		try:
			get_odom = rospy.ServiceProxy('odom_server', GetOdom)
			resp = get_odom()
			return resp.dist.x
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def plot_graph(self):

                        non = self.G.number_of_nodes()
                        pose = nx.get_node_attributes(self.G, 'pose_graph')
                        #fig = pylab.figure()

			pos = {}
                        for x in range(non):
                                pos[x] = (pose[x].x, pose[x].y)

                        nx.draw(self.G, pos, with_labels=True)
                        plt.savefig("/home/"+getpass.getuser()+"/Desktop/graph_map.png", format="PNG")
			time.sleep(1)
                        pass


	def choose_path(self):
		request = self.G.node[node]['ip']
		self.G.node[node]['ip']-=1
		return request
	
	def graph_map(self, data):

		test_node = False
		self.non = self.G.number_of_nodes()
		pose = nx.get_node_attributes(self.G, 'pose_graph')
		#print(pose)
		request = 0
		if (data.pose2d.x == float('inf') and data.pose2d.y == float('inf') and data.pose2d.theta == float('inf')):
				#Encontra uma interseção e infere sua posição
				if(self.n_node):
					dist_move = self.distance()
					print("Valor enc:"+ str(dist_move))
					_t_ = float(pose[self.n_node - 1].theta)
					if(_t_ != float('inf')):
						data.pose2d.x = pose[self.n_node-1].x + dist_move*math.cos(math.radians(_t_))
						data.pose2d.y = pose[self.n_node-1].y + dist_move*math.sin(math.radians(_t_))
						print("A pose da intersecao eh: x="+str(data.pose2d.x)+" e y="+str(data.pose2d.y))
					else:
						print("Not possible to add this intersection")
						return CreateMapResponse(request)	
				else:
					print("Not possible to add this intersection")
					return CreateMapResponse(request)	
		
		for node in range(self.non):

			if( data.pose2d.x == pose[node].x and data.pose2d.y == pose[node].y and data.pose2d.theta == pose[node].theta):
				test_node = True
				break

		if(test_node == False):
			print ("Novo noh adicionado")
			self.G.add_node(self.n_node, pose_graph = data.pose2d, ip = data.intr_pnt_brd)
			if(self.n_node != 0):
				length = math.sqrt(data.pose2d.x**2+data.pose2d.y**2) #corrigir está errado
				self.G.add_edge(self.n_node -1 , self.n_node, weight = length)
			request = 0
			self.n_node = self.n_node + 1
			self.plot_graph()
		else:
			request = self.choose_path()			

		return CreateMapResponse(request)


if __name__=='__main__':

	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
