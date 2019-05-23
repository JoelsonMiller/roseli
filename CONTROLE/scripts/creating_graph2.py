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
import numpy

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.Graph()
		self.n_node = 0
		self.past_node = 0

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

	def nav_path(self):
		

	def choose_path(self, node):
			
		length_min = 0
		aux = 0
		target = 0
		print("The lengths from the actual node"+str(node)+" from targets are: ")
		for index in range(self.non):
			if (self.G.node[index]['ip'] != 0 and index != node):
				length_min = nx.dijkstra_path_length(self.G, node, index, weight='weight')
				print(str(length_min))
				if(length_min < aux):
					aux = length_min
					target = index
		if(length_min != 0):
			print("The target is: "+str(target))
			shortest_path = nx.dijkstra_path(self.G, node, target, weight='weight')
			print("The shortest path's length is: "+str(length_min))
			print(shortest_path)
			#self.nav_path()
			
		request = self.G.node[node]['ip']
		print("A interseção retornada é: "+str(request))
		if(request!=0):
			self.G.node[node]['ip']-=1
		
		return request
	
	def graph_map(self, data):

		test_node = False
		self.non = self.G.number_of_nodes()
		pose = nx.get_node_attributes(self.G, 'pose_graph')
		#print(pose)
		request = 0
		tol = 0.1 #
		if (data.pose2d.x == float('inf') and data.pose2d.y == float('inf') and data.pose2d.theta == float('inf')):
				#Encontra uma interseção e infere sua posição
				if(self.n_node):
					dist_move = self.distance()
					print("Valor enc:"+ str(dist_move))
					print("O noh passado eh: "+str(self.past_node))
					_t_ = float(pose[self.past_node].theta)
					#if(_t_ != float('inf')):
					data.pose2d.x = pose[self.past_node].x + dist_move*math.cos(math.radians(_t_))
					data.pose2d.y = pose[self.past_node].y + dist_move*math.sin(math.radians(_t_))
					print("A pose da intersecao eh: x="+str(data.pose2d.x)+" e y="+str(data.pose2d.y))
						
				else:
					print("Not possible to add this intersection")
					return CreateMapResponse(request)	
		
		for node in range(self.non):

			#if( data.pose2d.x == pose[node].x and data.pose2d.y == pose[node].y and data.pose2d.theta == pose[node].theta):
			if( numpy.isclose(data.pose2d.x, pose[node].x, tol) and numpy.isclose(data.pose2d.y, pose[node].y, tol)):
				test_node = True
				break

		if(test_node == False):
			print ("Novo noh adicionado")
			self.G.add_node(self.n_node, pose_graph = data.pose2d, ip = data.intr_pnt_brd)
			if(self.n_node != 0):
				length = math.hypot(data.pose2d.x-pose[self.past_node].x, data.pose2d.y-pose[self.past_node].y)
				self.G.add_edge(self.past_node , self.n_node, weight = length)
			request = 0
			self.past_node = self.n_node
			self.n_node = self.n_node + 1
			self.plot_graph()
		else:
			if(not self.G.has_edge(self.past_node, node)):
				#print("i gonna add the edge between: "+str(self.past_node)+" e "+str(node))
				length = math.hypot(pose[node].x-pose[self.past_node].x, pose[node].y-pose[self.past_node].y)
				self.G.add_edge(self.past_node , node, weight = length)
				self.plot_graph()
			self.past_node=node
			request = self.choose_path(node)			

		return CreateMapResponse(request)


if __name__=='__main__':

	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
