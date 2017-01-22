#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#


import sys, os, Ice, traceback, time
import networkx as nx

import matplotlib.pyplot as plt


from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"GotoPoint.ice")
from RoboCompGotoPoint import *
Ice.loadSlice(preStr+"AprilTags.ice")
from RoboCompAprilTags import *
Ice.loadSlice("/opt/robocomp/interfaces/DifferentialRobot.ice")
from RoboCompDifferentialRobot import *

g = nx.Graph()

class SpecificWorker(GenericWorker):
	
	
	ruta = [10,61]
	state = "INIT"
	posiciones = {}
	listan = []
	
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		
		self.timer.timeout.connect(self.compute)
		self.period = 2000
		self.timer.start(self.period)
		self.grafo()
		self.compute()
		
	def setParams(self, params):

		return True
	
	def init (self, lista):
		self.state = "TI"
		
	
	def ti (self, lista):
		
		if len(self.ruta)== 0:
			self.state = "INIT"
			return
		
		print "Searching node"
		print "Searching list"
		self.listan = nx.shortest_path(g,source= str(self.nodoCercano()), target=str(self.ruta[0]))
		self.ruta.pop(0)
		print self.listan
		self.state = "PI"
		
	def pi (self, lista):
		print "To PI"
		if len(self.listan) == 0:
			self.state = "PI"
			return
		nodoactual = self.listan[0]
		self.listan.pop(0)
		try:
			print "Pos robot ", self.differentialrobot_proxy.getBaseState(), ", pos target ", self.posiciones[nodoactual][0], self.posiciones[nodoactual][1]
			self.gotopoint_proxy.go("",self.posiciones[nodoactual][0],self.posiciones[nodoactual][1],0.3)
		
		except Ice.Exception as e:
			print e
			print " ha fallado el ir hacia el nodo"
		self.state = "GO" 
		
		
	def go (self, lista):
		print "To GO"
		try:
			if self.gotopoint_proxy.atTarget() :
				print "Target"
				self.state = "PI" 
				return
			else :
				print "Searching"
				
		except Ice.Exception as e:
			print e
		
		
	def grafo(self):#creacion de grafo	
		self.positions = {}
		with open("puntos.txt","r") as f:
			for line in f:
				l=line.strip("\n").split()
				if l[0]=="N":
					g.add_node(l[1], x= float(l[2]),y=float(l[3]),name="")
					self.positions[l[1]] = (float(l[2]),float(l[3]))
				else:
					g.add_edge(l[1], l[2])

		print self.positions
		img = plt.imread("plano.png")
		plt.imshow(img, extent = [-12284,25600,-3840,9023])
		nx.draw_networkx(g, self.positions)
		plt.show()
			
		
	@QtCore.Slot()
	def compute(self):
		bState = TBaseState()

		print 'SpecificWorker.compute...'
		
		switch = {"INIT":  { "command": self.init, "params": []},
		   "TI":  { "command": self.ti, "params": []},
		   "PI":{ "command": self.pi, "params": []},
		   "GO":{ "command": self.go, "params": []}}
		switch[self.state]["command"](switch[self.state]["params"])
		
	def nodoCercano(self):
		bState = TBaseState()
		bState = self.differentialrobot_proxy.getBaseState()
		r = (bState.x , bState.z)
		dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
		#funcion que devuele el nodo mas cercano al robot
		return  sorted(list (( n[0] ,dist(n[1],r)) for n in self.positions.items() ), key=lambda s: s[1])[0][0]
