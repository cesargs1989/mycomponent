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
Ice.loadSlice(preStr+"DifferentialRobot.ice")
from RoboCompDifferentialRobot import *
Ice.loadSlice(preStr+"GotoPoint.ice")
from RoboCompGotoPoint import *

class SpecificWorker(GenericWorker):
  
	posiciones={}
	g=nx.Graph()
	estado='INIT'
	nodos=[]
	ruta=[67]
	print "Ruta: ", ruta
	
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.grafo()
		self.compute()
		
	def setParams(self, params):
		return True
	
	def init(self, lista):
	 self.estado="TI"
	  
	  
	def ti(self, lista):
	  if len(self.ruta)==0:
	      self.estado="INIT"
	      return
	  
	  print "To TI"
	  
	  self.nodos=nx.shortest_path(self.g,source=str(self.nodoCercano()),target=str(self.ruta[0]))
	  self.ruta.pop(0)
	  print self.nodos
	  
	  self.estado="PI"
	  
	def pi(self, lista):
	  print "To PI"	  
	  if len(self.nodos)==0:
	      self.estado="INIT"
	      return
	    
	  primernodo=self.nodos[0]
	  self.nodos.pop(0)
	  
	  print "Pos robot ", self.posiciones[primernodo][0], self.posiciones[primernodo][1]
	  self.gotopoint_proxy.go("",self.posiciones[primernodo][0],self.posiciones[primernodo][1],0.3)
	  
	  self.estado="GO"
	  
	  
	def go(self, lista):
	  print "To GO"
	  
	  if self.gotopoint_proxy.atTarget():
		print "Target"
		self.estado="PI"
		return
	  
	  else:
		print "Searching"
	  	 
	def grafo(self):   
	  with open("puntos.txt","r") as f:
	    for line in f:
	      l=line.strip("\n").split()
	      if l[0]=="N":
		self.g.add_node(l[1], x= float(l[2]),y=float(l[3]),name="")
		self.posiciones[l[1]] = (float(l[2]),float(l[3]))
	      else:
		self.g.add_edge(l[1], l[2])
		  
          #print self.posiciones
          img = plt.imread("plano.png")
          plt.imshow(img, extent = [-12284,25600,-3840,9023])
          '''nx.draw_networkx(self.g, self.posiciones)
          
          print "Haciendo camino minimo"
          print nx.shortest_path(self.g,source="1", target="6")
          plt.show()'''
	  
	@QtCore.Slot()
	
	def compute(self):
	  bState=TBaseState()
	  
	  state = {
	    'INIT': {"command": self.init, "params": []},
	    'TI': {"command": self.ti, "params": []},
	    'PI': {"command": self.pi, "params": []},
	    'GO': {"command": self.go, "params": []}}
	  state[self.estado]["command"](state[self.estado]["params"])

	def nodoCercano(self):
	  bState = TBaseState()
	  bState = self.differentialrobot_proxy.getBaseState()
	  r = (bState.x , bState.z)
	  dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
	  #funcion que devuele el nodo mas cercano al robot
	  print "Nodo/s mas cercano/s"
	  return  sorted(list (( n[0] ,dist(n[1],r)) for n in self.posiciones.items() ), key=lambda s: s[1])[0][0]  


