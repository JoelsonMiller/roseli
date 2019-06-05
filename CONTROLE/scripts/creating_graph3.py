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
import os 
import sys
import roslaunch

class subscriber_graph_map:

	def __init__(self):
                subs = rospy.Service('/pose2D', CreateMap, self.graph_map)
		self.G = nx.Graph()
		self.n_node = 0
		self.past_node = 0
		self.map_completed = False
		self.actual_node = -1
		self.chasing_goal = False
		self.path_saved_map = rospy.get_param('/creating_map/path_from_saved_map', "/home/"+getpass.getuser()+"/Desktop/mapa.yaml")
		self.load_saved_map = rospy.get_param('/creating_map/load_saved_map', False)
		erase_last_node = rospy.get_param('/creating_map/erase_last_node', False)
		if(self.load_saved_map):
			if(os.path.isfile(self.path_saved_map)):
				self.G = nx.read_yaml(self.path_saved_map)
				rospy.loginfo("Loaded map")
				if(erase_last_node):
					self.n_node = self.G.number_of_nodes() - 1 
					self.G.remove_node(self.n_node)
				else:
					self.n_node = self.G.number_of_nodes()	
				self.past_node = self.n_node - 1
			else:
				rospy.logerr("File cannot be load or not exist")
				sys.exit(0)

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

	def nav_path(self, shortest_path, node):
		
		request = 0 
		print("The next node to be rechead is: "+str(shortest_path[1]))
		pose = nx.get_node_attributes(self.G, 'pose_graph')
		_theta_ = pose[node].theta
		_theta_ = math.radians(_theta_)

		if(_theta_ == float('inf')):
			print("Retornando: Se trata de uma interserção: Programar ainda")
			node == self.past_node
			_theta_ = pose[node].theta
			_theta_ = math.radians(_theta_)

			if(self.intr_type == 0): #Interseção do tipo 90
				request = 0

			elif(self.intr_type == 5): #Interseção do tipo T
				if(math.sin(pose[node].theta) != 0):
					if(math.sin(pose[node].theta) == 1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 0
						elif(pose[node].x > pose[shortest_path[1]].x):
							request = 1
					elif(math.sin(pose[node].theta) == -1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 1
						elif(pose[node].x > pose[shortest_path[1]].x):
							resquest = 0

				elif(math.sin(pose[node].theta) == 0):
					if(math.cos(pose[node].theta) == 1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 1
						elif(pose[node].x > pose[shortest_path[1]].x):
							request = 0
					elif(math.cos(pose[node].theta) == -1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 0
						elif(pose[node].x > pose[shortest_path[1]].x):
							resquest = 1

			elif(self.intr_type == 4): #Interseção do tipo '|-' ou '-|'
				if(math.sin(pose[node].theta) != 0):
					if(math.sin(pose[shortest_path[1]].theta) != 0):
						request = 0
					elif(math.sin(pose[shortest_path[1]].theta) == 0):
						request = 1
				elif(math.sin(pose[node].theta) == 0):
					if(math.sin(pose[shortest_path[1]].theta) == 0):
						request = 0
					elif(math.sin(pose[shortest_path[1]].theta) != 0):
						request = 1			
			
			elif(self.intr_type == 0): #Interseção do tipo 'Y'
				if(math.sin(pose[node].theta) != 0):
					if(math.sin(pose[node].theta) == 1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 0
						elif(pose[node].x > pose[shorteste_path[1]].x):
							request = 1
					elif(math.sin(pose[node].theta) == -1):
						if(pose[node].x < pose[shortest_path[1]].x):
							request = 1
						elif(pose[node].x > pose[shortest_path[1]].x):
							resquest = 0

				elif(math.sin(pose[node].theta) == 0):
					if(math.cos(pose[node].theta) == 1):
						if(pose[node].y < pose[shortest_path[1]].y):
							request = 1
						elif(pose[node].y > pose[shortest_path[1]].y):
							request = 0
					elif(math.cos(pose[node].theta) == -1):
						if(pose[node].y < pose[shortest_path[1]].y):
							request = 0
						elif(pose[node].y > pose[shortest_path[1]].y):
							resquest = 1	

			elif(self.intr_type == 6): #Interseção do tipo '+'
				if(math.sin(pose[node].theta) != 0):
					if(math.sin(pose[shortest_path[1]].theta) != 0):
						request = 0
					else:
						if(math.sin(pose[shortest_path[1]].theta) == 1):
							if(pose[shortest_path[1]].x > pose[node].x):
								request = 1
							elif(pose[shortest_path[1]].x < pose[node].x):
								request = 2
						elif(math.sin(pose[shortest_path[1]].theta) == -1):
							if(pose[shortest_path[1]].x > pose[node].x):
								request = 2
							elif(pose[shortest_path[1]].x < pose[node].x):
								request = 1
				elif(math.sin(pose[node].theta) == 0):
					if(math.sin(pose[shortest_path[1]].theta)==0):
						request = 0
					else:
						if(math.cos(pose[shortest_path[1]].theta)==1):
							if(pose[shortest_path[1]].y > pose[node].y):
								request = 2
							elif(pose[shortest_path[1]].y < pose[node].y):
								request = 1
						elif(math.cos(pose[shortest_path[1]].theta)==-1):
							if(pose[shortest_path[1]].y > pose[node].y):
								request = 1
							elif(pose[shortest_path[1]].y < pose[node].y):
								request = 2

			elif((self.intr_type == 1) or (self.intr_type == 2)):
				if((math.fabs(math.cos(pose[shortest_path[1]].theta)) == 1) or (math.fabs(math.sin(pose[shortest_path[1]].theta)) == 1)):
					request = 0
				else:
					request = 1
							

			return request
		

		if( math.fabs( math.cos( _theta_ ) ) == 1.0):
			print("Horizontal")
			print("O valor de x do noh atual eh: "+str(pose[node].x)+" e do noh aser alcançado eh: "+str(pose[shortest_path[1]].x))
			print("E o valor de theta eh: "+str(_theta_))
			if(pose[node].x < pose[shortest_path[1]].x):
				if(_theta_ == 0.0):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
			elif(pose[node].x > pose[shortest_path[1]].x):
				if(_theta_ == math.radians(180.0)):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1	
				
		elif(math.fabs( math.sin( _theta_ ) ) == 1.0):
			
			print("Vertical")
			if(pose[node].y < pose[shortest_path[1]].y):
				if(_theta_ == math.radians(90.0)):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
			elif(pose[node].x > pose[shortest_path[1]].x):
				if(_theta_ == math.radians(270.0)):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1

		elif(math.sin(2*_theta_) > 0):
			print("Diagonal /")
			aux_ang = math.atan((pose[shortest_path[1]].y - pose[node].y)/(pose[shortest_path[1]].x - pose[node].x))
			if(0 < aux_ang and aux_ang < math.pi/4):
				if(0 < _theta_ and theta < math.pi/2 ):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
			elif(math.pi/2 < aux_ang and aux_ang < 3*math.pi/4):
				if(math.pi < _theta_ and theta < 3*math.pi/4):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
		else:
			print("Diagonal ")
			aux_ang = math.atan((pose[shortest_path[1]].y - pose[node].y)/(pose[shortest_path[1]].x - pose[node].x))
			if(math.pi/4 < aux_ang and aux_ang < math.pi/2):
				if(math.pi/2 < _theta_ and theta < math.pi ):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
			elif(3*math.pi/4 < aux_ang and aux_ang < 2*math.pi):
				if(3*math.pi/4 < _theta_ and theta < 2*math.pi):
					print("Siga em frente")
				else:
					print("Vire 180")
					request = -1
		
			
		return request	

	def choose_path(self, node):
		
		if(self.G.node[node]['ip'] != 0):
			request = self.G.node[node]['ip']
			print("A interseção retornada é: "+str(request))
			self.G.node[node]['ip']-=1
			return request	
		request = 0
		length_min = 0
		aux = 0
		target = 0
		print("The lengths from the actual node "+str(node)+" from targets are: ")
		for index in range(self.non):
			if (self.G.node[index]['ip'] != 0 and index != node):
				aux = nx.dijkstra_path_length(self.G, node, index, weight='weight')
				print(str(aux)+" : to node "+str(index))
				if(aux < length_min or length_min == 0):
					length_min = aux
					target = index

		if(length_min != 0):
			print("The target is: "+str(target))
			shortest_path = nx.dijkstra_path(self.G, node, target, weight='weight')
			print("The shortest path's length is: "+str(length_min))
			print(shortest_path)
			request = self.nav_path(shortest_path, node)
			
		elif(length_min == 0):
			self.map_completed = True
			for index in range(self.non):
				neighbors=self.G.neighbors(self.non)
				if(neighbors <= 1):
					self.map_completed = False
					break;
			if(self.map_completed)
				rospy.loginfo("The mapping is finnish")

		return request
	
	def graph_map(self, data):

		
		##########################
		### Ajuste a tag errada###
		########################## 

		if((data.pose2d.x == 190.5) and (data.pose2d.y == 89.5)):
			data.pose2d.x, data.pose2d.y = data.pose2d.y, data.pose2d.x
			if(data.pose2d.theta != 60):
				data.pose2d.theta = 240.0
			print("Variaveis trocadas")

		####################################
			
		test_node = False
		self.non = self.G.number_of_nodes()
		pose = nx.get_node_attributes(self.G, 'pose_graph')
		self.intr_type = data.intr_type
		#print(pose)
		request = 0
		tol = 0.1 #
		if (data.pose2d.x == float('inf') and data.pose2d.y == float('inf') and data.pose2d.theta == float('inf')):
				#Encontra uma interseção e infere sua posição
				if(self.n_node):
					dist_move = self.distance()
					print("Valor enc:"+ str(dist_move))
					#print("O noh passado eh: "+str(self.past_node))
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
				if(data.pose2d.theta != float('inf')):
					pose[node].theta = data.pose2d.theta 
					nx.set_node_attributes(self.G, 'pose_graph', pose)
				break
		
		if(self.map_completed and pose[node].theta != float('inf')):
			if(not self.chasing_goal):
				ros_node = roslaunch.core.Node('roseli', 'imageconverter')
				launch = roslaunch.scriptapt.ROSLaunch()
				process = launch.launch(ros_node)
				process.stop()
				self.actual_node = node
				rospy.loginfo("Waiting the goal pose: ")
				return CreateMapResponse(0)
			else:
				print("Programar ainda")
		
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
			request = self.choose_path(node)
			self.past_node = node	
			print("The past node is: "+str(self.past_node))		
		nx.write_yaml(self.G, self.path_saved_map)
		return CreateMapResponse(request)


if __name__=='__main__':

	try:
		rospy.init_node('listener')
		subs = subscriber_graph_map()
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
