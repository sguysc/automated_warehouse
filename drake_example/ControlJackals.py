#!/usr/bin/env python

# general stuff
import numpy as np
import json
import argparse
import dill
import networkx as nx
import re

from warehouse_map import LoadMP, GetSpecificControl, cell, find_nearest, GetRotmat, FL_L

# ROS stuff
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

SYNTH_AUTOMATA_FILE = 'map_funnel.json'
MP_MAP_FILE = 'none.pickle'
LABEL2BIT_FILE = 'map_funnel.label2bit'
pix2m = 0.2 #[m]
bounds = np.array([ 0.,  0., 20., 40.])
#cell = 1.25 #[m]
W_xgrid = np.arange(bounds[0]+cell, bounds[2]-cell, cell)
W_ygrid = np.arange(bounds[1]+cell, bounds[3]-cell, cell)
umax    = 2.6 * 1.6 * 1000.0 / 3600.0  # mph -> m/sec     5.0
delmax  = 80.0*np.pi/180.0  #rad   30.0 80
			

# class to handle a single robot comm.
class Jackal:
	def __init__(self, idx, aut_file=SYNTH_AUTOMATA_FILE, map_file=MP_MAP_FILE, l2b_file=LABEL2BIT_FILE):
		self.idx = idx
		self.msg_num = 0
		self.pose = None
		self.L = 0.42 # for the jackal
		#self.L = FL_L #for the forklift
		
		# load the slugs solution
		aut = open(aut_file, 'r')
		self.automata = json.load(aut)
		aut.close()
		self.G = nx.read_gpickle(map_file)
		self.mps = LoadMP()
		
		dbfile = open(l2b_file, 'rb')
		self.map_label_2_bit = dill.load(dbfile)
		dbfile.close()
			
		# the reverse dictionary is useful
		self.map_bit_2_label = dict((v, k) for k, v in self.map_label_2_bit.items())
		self.states, self.actions = GetSpecificControl(self.automata, self.map_bit_2_label, debug=False)
		self.curr_state = 0
		self.N_state  = len(self.states)
		self.N_ellipse = len(self.mps[ self.actions[self.curr_state] ]['V'])
		self.curr_ell = 0
		self.K     = self.mps[self.actions[self.curr_state] ]['K'][self.curr_ell]
		self.x_ref = self.CovertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][self.curr_ell], \
										self.states[self.curr_state])
		self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][self.curr_ell]
		
		# handle the ROS topics
		# get data
		self.sensors_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.measurement_cb)
		# send commands
		self.control_pub = rospy.Publisher('/jackal%d/jackal_velocity_controller/cmd_vel' %self.idx, Twist, queue_size=1)

	#/gazebo/model_states
	def measurement_cb(self, msg):
		self.msg_num += 1
		
		# decipher the msg coming from gazebo (ground truth). later, switch to vicon in lab
		i      = msg.name.index('jackal%d' %(self.idx))
		pose   = msg.pose[i].position
		orient = msg.pose[i].orientation
		linvel = msg.twist[i].linear
		rotvel = msg.twist[i].angular
		
		# get euler angles to know heading
		angles = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
		
		# store for other uses
		self.pose = np.array([pose.x, pose.y, angles[2]]) # (gazebo->mine axes match )
		
		#import pdb; pdb.set_trace()
		# check if we're in the next funnel
		next_state = (self.curr_state + 1) if (self.curr_state < self.N_state - 1) else 0
		if (self.CheckFunnelCompletion(self.states[next_state], self.actions[next_state]) == True):
			rospy.loginfo('reached next state %d' %(next_state))
			#we've reached the beginning of a new funnel, so update the states
			self.curr_state = next_state
			self.curr_ell = 0
			# get how many points are in this new motion primitive
			self.N_ellipse = len(self.mps[ self.actions[self.curr_state] ]['V'])
			self.K     = self.mps[self.actions[self.curr_state] ]['K'][self.curr_ell]
			self.x_ref = self.CovertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][self.curr_ell], \
										self.states[self.curr_state])
			self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][self.curr_ell]
		
		# check if we're in the next ellipse on the same funnel
		next_ell = (self.curr_ell + 1) if (self.curr_ell < self.N_ellipse - 1) else self.N_ellipse
		if(self.CheckEllipseCompletion(self.states[self.curr_state], \
									   self.actions[self.curr_state], next_ell) == True):
			self.K     = self.mps[self.actions[self.curr_state] ]['K'][next_ell]
			self.x_ref = self.CovertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][next_ell], \
										self.states[self.curr_state])
			self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][next_ell]
			self.curr_ell = next_ell
		
		# do control and call the robot
		self.control(self.K, self.x_ref, self.u_ref)
		
		self.telemetry()

	def telemetry(self):
		# telemetry
		rospy.logdebug('F=%d; x=%f; y=%f; th=%f; xr=(%f, %f, %f); ur=(%f, %f); s=%d; e=%d' \
					   %(self.msg_num, self.pose[0], self.pose[1], self.pose[2], \
						 self.x_ref[0], self.x_ref[1], self.x_ref[2], self.u_ref[0], self.u_ref[1], \
						self.curr_state, self.curr_ell))
	
	
	def InEllipse(self, S, x0, pose=None):
		if(pose):
			ret = (pose-x0).dot(S.dot(pose-x0))
		else:
			ret = (self.pose-x0).dot(S.dot(self.pose-x0))
			
		return (ret<=1)
		
	def CheckEllipseCompletion(self, funnel, action, ellipse_num):
		S, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, action, ellipse_num)
		if( self.InEllipse(S, ellipse_pose) ):
			return True
		
		return False
	
	def CheckFunnelCompletion(self, funnel, action):
		return self.CheckEllipseCompletion(funnel, action, 0)

	def GetCoordAndEllipseFromLabel(self, funnel, action, ellipse_num):
		mp = self.mps[action]
		# do I need to rotate the ellipse as well??? CHECK THIS!
		S = mp['V'][ellipse_num]
		x0 = mp['xcenter'][ellipse_num]
		
		e_center = self.CovertRelPos2Global(x0, funnel)

		return S, e_center

	def CovertRelPos2Global(self, rel_pose, funnel):
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', funnel)] # extract first ellipse pose
		xs = W_xgrid[xs]
		ys = W_ygrid[ys]
		
		rotmat = GetRotmat(orient)
		
		if(orient == 3):
			orient = -1
		
		theta = rel_pose[2]
		x_rel = rotmat.dot(rel_pose[:2]) # just x,y	
		
		pose = np.array([xs, ys]) + x_rel # location in the world of that ellipse
		pose = np.hstack((pose, theta + orient*np.pi/2.0 )) 
		
		return pose
		
	def control(self, K, x_0, u_0):
		u = u_0 - K.dot(self.pose - x_0)	
		
		#saturations
		if(u[1] > umax):
			u[1] = umax
		if(u[1] < -umax):
			u[1] = -umax
		if(u[0] > delmax):
			u[0] = delmax
		if(u[0] < -delmax):
			u[0] = -delmax
			
		self.actuation(u)
	
	def actuation(self, u):
		# Jackal accepts commands in linear velocity and angular velocity.
		# our model assumed u = [steering angle, linear velocity]
		# This is our model:
		# xdot = v*cos(teta)
		# ydot = v*sin(teta)
		# tetadot = v/L * tan( delta )
		# This is probably) Jackal's model:
		# xdot = v*cos(teta)
		# ydot = v*sin(teta)
		# tetadot = omega
		#
		# so we can easily see that we can get omega = v/L * tan( delta )
		t = Twist()
		t.angular.z = u[1] / self.L * np.tan(u[0]) # delta
		t.linear.x  = u[1] # V
		#print('v = %f' %u[1])
		self.control_pub.publish(t)
			
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Robot number to control.')
	parser.add_argument('--n', type=int, default=0,
					help='an integer for the robot number')
	args = parser.parse_args()
	print('Controller for Jackal%d' %args.n)

	rospy.init_node('run_jackal_%d' %args.n)#, log_level=rospy.DEBUG)
	J = Jackal(args.n)
			
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down\n")
	
	# END ALL
