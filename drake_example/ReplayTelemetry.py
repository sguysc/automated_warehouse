#!/usr/bin/env python2

# general stuff
import numpy as np
import json
import dill
import networkx as nx
import re

from warehouse_map import LoadMP, GetSpecificControl, find_nearest, GetRotmat, FL_L

MAP = 'lab'
MAP_FILE = MAP + '.map'
SYNTH_AUTOMATA_FILE = MAP + '.json'
MP_MAP_FILE = MAP + '.pickle'
LABEL2BIT_FILE = MAP + '.label2bit'
TELEMETRY_FILE = '../telemetry/telemetry_2020_01_06_15_55_16_no_bias_angle.log'
pix2m  = 1.0 #0.2 #[m]
ft2m   = 0.3048
umax   = 0.3 #1.0  # jackal m/sec     5.0
delmax = 80.0*np.pi/180.0  #rad   30.0 80

# class to handle a single robot comm.
class Jackal:
	def __init__(self, idx, map_file=MAP_FILE, aut_file=SYNTH_AUTOMATA_FILE, mp_file=MP_MAP_FILE, l2b_file=LABEL2BIT_FILE, \
				telem_file=TELEMETRY_FILE):
		self.Fs = 10.
		self.pose   = np.array([0.0, 0.0, 0.0*np.pi/2.0]) #None
		#self.L = 0.42 # for the jackal
		self.L = FL_L #*15.0 #for the forklift
		self.u = np.array([0.0, 0.0]) # stores the last controls
		# integrators
		self.v_prev = 0.0
		self.delta_prev = 0.0

		# load the map properties
		aut = open(map_file, 'r')
		map_prop = json.load(aut)
		aut.close()
		#import pdb; pdb.set_trace()
		cell = map_prop['cell']
		bounds = np.array([ map_prop['workspace'][0]['x'],  map_prop['workspace'][0]['y'], \
							 map_prop['workspace'][0]['X'],  map_prop['workspace'][0]['Y']])
		self.W_xgrid = np.arange(bounds[0]+cell/2.0, bounds[2]-cell/2.0, cell)
		self.W_ygrid = np.arange(bounds[1]+cell/2.0, bounds[3]-cell/2.0, cell)

		# load the slugs solution
		aut = open(aut_file, 'r')
		self.automata = json.load(aut)
		aut.close()
		self.G = nx.read_gpickle(mp_file)
		self.mps = LoadMP()

		dbfile = open(l2b_file, 'rb')
		self.map_label_2_bit = dill.load(dbfile)
		dbfile.close()

		# the reverse dictionary is useful
		self.map_bit_2_label = dict((v, k) for k, v in self.map_label_2_bit.items())
		self.states, self.actions = GetSpecificControl(self.automata, self.map_bit_2_label, debug=False)

		for state in self.states:
			print('R%d (%s)' %(self.map_label_2_bit[state],state))

		with open(telem_file, 'rb') as f:
			self.data = np.loadtxt(f,delimiter=';',skiprows=1)
		#import pdb; pdb.set_trace()
		self.t = self.data[:,0]
		self.t = self.t-self.t[0]
		self.frame = self.data[:,1].astype(dtype='int')
		#self.frame = self.frame.astype(dtype='int')
		self.poses  = self.data[:,2:5]
		self.x_refs = self.data[:,5:8]
		self.u_ref = self.data[:,8:10]
		self.curr_states = self.data[:,10].astype(dtype='int')
		self.curr_ells = self.data[:,11].astype(dtype='int')
		self.change_control = self.data[:,12].astype(dtype='int')
		self.action  = self.data[:,13].astype(dtype='int')
		self.u_telem = self.data[:,14:16]
		self.linvel  = self.data[:,16:19]
		self.rotvel  = self.data[:,19:22]
		self.x0s = self.data[:,22:25]
		self.u0s = self.data[:,25:27]
		
		self.lin_v = []
		self.omega = []

	# main loop
	def Run(self):
		for i in range(self.frame.shape[0]):
			#import pdb; pdb.set_trace()
			self.curr_state = self.curr_states[i]
			self.curr_ell = self.curr_ells[i]
			self.pose  = self.poses[i,:]
			self.pose[2]  = self.pose[2] * np.pi/180.0
			#x_ref = self.x_refs[i,:]
			#x_ref[2]  = x_ref[2] * np.pi/180.0
			x_0 = self.x0s[i,:]
			x_0[2]  = x_0[2] * np.pi/180.0
			u_0     = self.u0s[i,:]
			K       = self.mps[self.actions[self.curr_state] ]['K'][self.curr_ell]

			if(self.curr_state == 31):
				import pdb; pdb.set_trace()

			self.control(K, x_0, u_0, do_calc=True, index=i)

	# y/x = 1/(tau*s+1) ==>  y(k) = T/(tau+T) * (x(k)+tau/T*y(k-1))
	def LPF(self, x, y_prev, tau=1.):
		T = 1./self.Fs
		y = T/(tau + T) * (x + tau/T*y_prev)
		return y

	# compute the actual controls to the robot
	def control(self, K, x_0, u_0, do_calc=True, index=0):
		# error comes in global coordinates while the K computed in the optimization
		# assumes y-up x-right and we face right
		label = self.states[self.curr_state]
		orient, __, __ = [int(s) for s in re.findall(r'-?\d+\.?\d*', label)] # extract first ellipse pose
		rotmat = GetRotmat(orient)
		rotmat = np.linalg.inv(rotmat)
		err = self.pose - x_0
		# find the smallest angle difference
		ang_diff = (err[2] + np.pi) % (2*np.pi) - np.pi
		err_rel = rotmat.dot(err[:2])
		err_rel = np.hstack((err_rel, ang_diff )) # add the theta component

		#import pdb; pdb.set_trace()
		u = u_0 - K.dot(err_rel)
		#u[0] = self.LPF(u[0], self.delta_prev, tau=10.)
		#self.delta_prev = u[0]

		u[1] = self.LPF(u[1], self.u[1], tau=2.)
		self.v_prev = u[1]

		#saturations
		if(u[1] > umax):
			u[1] = umax
		if(u[1] < 0):
			u[1] = 0.0
		if(u[0] > delmax):
			u[0] = delmax
		if(u[0] < -delmax):
			u[0] = -delmax

		self.u = u

		self.actuation(u, index=index)

	# send the commands to ROS
	def actuation(self, u, index=0):
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
		#t = Twist()
		#saturations
		#V,W = self.LimitCmds(u[1], u[1] / self.L * np.tan(u[0]))
		#self.u = np.array([W,V])
		#t.angular.z = W
		self.omega.append( u[1] / self.L * np.tan(u[0]) ) # delta; 1.0/umax is a gain to compensate 1.0/umax *
		# the fact that I am running in less than 1.0m/s in the lab
		#t.linear.x  = V #u[1] # V
		self.lin_v.append( u[1] ) # V
		#print('v = %f' %u[1])
		#self.control_pub.publish(t)
		print( 'state=%d:\tdelta=(%.3f,%.3f)\tu=(%.3f,%.3f)' %(self.curr_state, u[0], self.u_telem[index,0], \
															   u[1], self.u_telem[index,1]) )

if __name__ == '__main__':
	J = Jackal(1)

	#try:
		#rospy.spin()
	J.Run()
	#except KeyboardInterrupt:
	#	print("Shutting down\n")
	#except:
	#	pass
	# END ALL
