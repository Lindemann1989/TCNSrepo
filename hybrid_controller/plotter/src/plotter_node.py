#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import matplotlib.colors as colors
from hybrid_controller.msg import BarrierFunction
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import os


class Plotter:
	def __init__(self):
		rospy.init_node("plotter")
		rospy.on_shutdown(self.plotFunctions)
		self.robot = rospy.get_param('~robot')
		self.bf = []
		self.t = []
		self.ap = 1
		self.t_max = 90
		
		
		self.u = []
		self.ux = []
		self.uy = []
		self.uppc = []
		self.upfc = []
		
		self.pose0 = []
		self.pose1 = []
		self.pose2 = []

		
		self.barrierfunction_sub = rospy.Subscriber("/barrierfunction"+str(self.robot), BarrierFunction, self.barrierfunctionCallback)
		self.u_sub = rospy.Subscriber("/cmdvel"+str(self.robot), Twist, self.uCallback)
		self.uppc_sub = rospy.Subscriber("/uppc"+str(self.robot), Twist, self.uppcCallback)
		self.upfc_sub = rospy.Subscriber("/upfc"+str(self.robot), Twist, self.upfcCallback)
		self.p0_sub = rospy.Subscriber("/nexus0/pose", PoseStamped, self.p0Callback)
		self.p1_sub = rospy.Subscriber("/nexus1/pose", PoseStamped, self.p1Callback)
		#self.p2_sub = rospy.Subscriber("pose_robot2", PoseStamped, self.p2Callback)
		self.p2_sub = rospy.Subscriber("/nexus2/pose", PoseStamped, self.p2Callback)


	def uppcCallback(self, msg):
		if self.ap==1:
			self.uppc.append(msg)

	def upfcCallback(self, msg):
		if self.ap==1:
			self.upfc.append(msg)

	def uCallback(self, msg):
		if self.ap==1:
			self.u.append(msg)
			self.ux.append(msg.linear.x)
			self.uy.append(msg.linear.y)
			

	def barrierfunctionCallback(self, msg):
		if self.ap==1:
			self.bf.append(msg.bf)
			self.t.append(msg.t)
			
			if msg.t>self.t_max:
				self.ap=0
		
	def p0Callback(self, msg):
		if self.ap==1:
			self.pose0.append([msg.pose.position.x, msg.pose.position.y, 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)*180/np.pi])

	def p1Callback(self, msg):
		if self.ap==1:
			self.pose1.append([msg.pose.position.x, msg.pose.position.y, 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)*180/np.pi])

	def p2Callback(self, msg):
		if self.ap==1:
			self.pose2.append([msg.pose.position.x, msg.pose.position.y, 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)*180/np.pi])

	def plotFunctions(self):

		fig = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		ax.set_ylim([-1, 1])
		ax.set_xlabel("t [s]")
		plt.plot(np.array(self.t), np.array(self.bf), color='black', linewidth='1.5', label='$\mathfrak{b}(\mathbf{x}(t),t)$')
		plt.legend(loc=4, fontsize='large')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/barrierfunction'+str(self.robot)+'.png')
		
		fig = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		# ax.set_ylim([-2, 2.5])
		ax.set_xlabel("x [m]")
		ax.set_ylabel("y [m]")
		
		self.pose0 = downSample(self.pose0, 0.005)
		self.pose1 = downSample(self.pose1, 0.005)
		self.pose2 = downSample(self.pose2, 0.005)
		
		plotSignal(self.pose0, 'Reds', 'Agent 1', '^', '2.0', 2)
		plotSignal(self.pose1, 'Blues', 'Agent 2', '^', '2.0', 2)
		plotSignal(self.pose2, 'Greens', 'Agent 3', '^', '2.0', 2)

		
		plt.legend(loc=3, fontsize='large')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/trajectory'+str(self.robot)+'.png')
		
		
		fig = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		ax.set_ylim([-200, 200])
		ax.set_xlabel("t [s]")
		if len(self.t)>len(self.ux):
			del self.t[-1]
		if len(self.t)<len(self.ux):
			del self.ux[-1]
			del self.uy[-1]
		plt.plot(np.array(self.t), np.array(self.ux), color='blue', linewidth='1.5', label='$u_x$')
		plt.plot(np.array(self.t), np.array(self.uy), color='red', linewidth='1.5', label='$u_y$')
		plt.legend(loc=4, fontsize='large')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/input'+str(self.robot)+'.png')
		


def plotSignal(signal, color, label, marker, linewidth, marker_version):

	
	xy = np.array(signal)[::25]
	cmap = cm.get_cmap(color)
	c = np.linspace(0.2, 1, xy.shape[0])
	colors = np.r_[c, c]
	mycolors = cmap(colors)
	for i in range(xy.shape[0]):
		plt.scatter(xy[i,0],
					xy[i,1], 
					label=label if i==int(xy.shape[0]/2) else "",
					marker=get_arrow(xy[i,2], marker_version),
					s = 25,
					linewidths=linewidth,
					color=mycolors[i])

def get_arrow(angle, version):
	a = np.deg2rad(angle)
	if version == 0:
		ar = np.array([[-.5,-.25],[-.5,.25],[.5,0],[-.5,-.25]]).T
	elif version == 1:
		ar = np.array([[-.5,-.25],[.5,0],[-.5,.25],[.5,.25],[.5,-.25]]).T
	elif version == 2:
		ar = np.array([[-.5,-.25],[.5,0],[-.5,.25],[-.2,0]]).T
	elif version == 3:
		ar = np.array([[.5,0],[.125,.25],[-.5,.25],[-.5,-.25],[.125,-.25]]).T
	elif version == 4:
		ar = np.array([[.5,0],[.125,.25],[-.5,.25],[-.125, 0],[-.5,-.25],[.125,-.25]]).T
	rot = np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
	return np.dot(rot,ar).T
	
def downSample(x, threshold):
	x = np.array(x)
	res = [x[0,:]]
	for i in range(len(x)):
		if np.sqrt(np.dot((x[i,:2]-res[-1][:2]).T, x[i,:2]-res[-1][:2])) > threshold:
			res.append(x[i,:])
	res.append(x[len(x),:])
	return res

if __name__ == '__main__':
	p = Plotter()
	
	rospy.spin()

