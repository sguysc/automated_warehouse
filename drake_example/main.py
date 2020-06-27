#!/usr/bin/env python

# general stuff
import numpy as np
import json
import argparse
import dill
import networkx as nx
import re
import sys
import logging # can't handle the ros logging :(
from time import localtime, strftime
from timeit import default_timer as timer

# my stuff
from warehouse_map import LoadMP, GetSpecificControl, find_nearest, GetRotmat, FL_L, FL_W
from SlugsInterface import *
import GeometryFunctions_fcl as gf
#import GeometryFunctions as gf

# lab or simulation
SIMULATION = True

# ROS stuff
import rospy
if(SIMULATION):
	from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped
	from gazebo_msgs.msg import ModelStates
else:
	from geometry_msgs.msg import Twist, PoseStamped, Pose2D
	from nav_msgs.msg import Odometry
	from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_inverse
from tf2_geometry_msgs import do_transform_vector3


ROBOT_TYPE = 'JACKAL'  # original JACKAL run with 'roslaunch jackal_gazebo jackal_world.launch'
#ROBOT_TYPE = 'TURTLEBOT'
#MAP    = 'raymond'
MAP    = 'lab'
umax   = 0.3 #1.0  # jackal m/sec     5.0
delmax = 45.0*np.pi/180.0  # jackal rad   30.0 80
MEAS_FS = 100 #100 for gazebo, not sure about vicon/optitrack

SLUGS_DIR = '/home/cornell/Tools/slugs_ltl_stack/src/slugs'
MAP_FILE = MAP + '.map'
SYNTH_AUTOMATA_FILE = MAP
MP_MAP_FILE = MAP + '.pickle'
LABEL2BIT_FILE = MAP + '.label2bit'
pix2m  = 1.0 #0.2 #[m]
ft2m   = 0.3048
logger = None
logger_state = None
CALIB_ANGLE_THETA = 0.0 #I've changed the angle in the optitrack software-0.16 #[rad]

# class to handle a single robot comm.
class Jackal:
	def __init__(self, idx, total_robots, list_obs, map_file=MAP_FILE, aut_file=SYNTH_AUTOMATA_FILE, \
				                          mp_file=MP_MAP_FILE, l2b_file=LABEL2BIT_FILE):
		global CALIB_ANGLE_THETA
		
		self.all_topics_loaded = False
		self.Fs = 10 # main/control loop frequency
		self.idx = idx
		self.total_robots = total_robots
		self.msg_num = 0
		self.pose   = np.array([0.0, 0.0, 0.0*np.pi/2.0]) #None
		self.other_robots_arr = np.setdiff1d(np.arange(0,self.total_robots),[self.idx]) # everything but ego
		self.others_pose = {} #np.zeros((total_robots-1,3)) # in index 0 will be the first robot that is not ego
		self.list_obs = list_obs
		self.linvel = None
		self.rotvel = None
		self.last_linvel = None
		self.last_rotvel = None
		#self.L = 0.42 # for the jackal
		self.L = FL_L #*15.0 #for the forklift
		self.do_calc = True
		self.u = np.array([0.0, 0.0]) # stores the last controls
		# integrators
		self.v_prev = 0.0
		self.delta_prev = 0.0
		self._sensing_function_time = 0.0
		
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
		
		# create the funnel path file
		#self.states_fid = open('../telemetry/r%d_%s.states' %(idx, MAP), 'wt')

		# load the slugs solution
		if(SIMULATION == False):
			idx -= 1 #this is because in the lab the index is +1
		
		self.G = nx.read_gpickle(mp_file)
		self.mps = LoadMP()
		self.Nactions = len(self.mps)
		self.STAY_IN_PLACE = self.Nactions

		dbfile = open(l2b_file, 'rb')
		self.map_label_2_bit = dill.load(dbfile)
		dbfile.close()
		# the reverse dictionary is also useful
		self.map_bit_2_label = dict((v, k) for k, v in self.map_label_2_bit.items())
		#import pdb; pdb.set_trace()
		# old, interface with the explicitStrategy of slugs
		#aut_file = aut_file + ('_r%d' %idx) + '.json'
		#aut = open(aut_file, 'r')
		#self.automata = json.load(aut)
		#aut.close()
		# new, interface the interactive mode of slugs
		tic = timer()
		self.slugs = SlugsInterface(aut_file + ('_r%d' %idx), simulate=False, slugsLink = SLUGS_DIR)
		toc = timer()
		if(not self.slugs.enabled):
			print('Cannot create slugs interface.')
			return
		print('Done loading slugs (%.2f[sec]).' %(toc-tic))
		self.slugs.DiscoverInputs()
		self.slugs.DiscoverOutputs()
		self.slugs.DiscoverGoals()
		self.slugs.GetInitialPos()
		
		self.funnel_sensors = dict(zip(range(self.slugs._Nsens), [False]*self.slugs._Nsens))		
		
		# use this break point as a synchronizer between multiple runs
		print('Press c to start ...')
		import pdb; pdb.set_trace()

		#self.states, self.actions = GetSpecificControl(self.automata, self.map_bit_2_label, debug=False)
		self.curr_state, self.action = self.slugs.GetNumericState()
		#self.states_fid.write('%d;%s;%d\n' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
		logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
		#import pdb; pdb.set_trace()
		# now, get the new next state
		if(self.action == self.STAY_IN_PLACE):
			print('oh no, R%d starts with action stay in place' %(self.idx))
			self.next_state =  self.curr_state
			self.do_calc = False
		else:
			for key, val in self.G[self.map_bit_2_label[self.curr_state]].items():
				if( val['motion'] == self.action):
					self.next_state = self.map_label_2_bit[key]
					break
		self.next_state, self.next_action = \
			self.slugs.FindNextStep(self.next_state, self.funnel_sensors)
		self.goal = 0
		print('R%d starts in region %s (%d)' %(idx, self.map_bit_2_label[self.curr_state],self.curr_state))

		#self.N_state  = len(self.states)
		self.N_ellipse = len(self.mps[self.action]['V'])
		self.curr_ell  = self.FindNextValidEllipse(self.action)
		self.K         = self.mps[self.action]['K'][self.curr_ell]
		self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.action]['xcenter'][self.curr_ell], \
										self.map_bit_2_label[self.curr_state])
		self.x_ref_hires = self.mps[self.action]['xtraj']
		self.u_ref = self.mps[self.action]['unom'][self.curr_ell]
		self.u_ref_hires = self.mps[self.action]['utraj']
	
		#import pdb; pdb.set_trace()
		# handle the ROS topics
		if(SIMULATION):
			# send commands
			if('JACKAL' in ROBOT_TYPE):
				self.control_pub = rospy.Publisher('/jackal%d/jackal_velocity_controller/cmd_vel' %self.idx, Twist, queue_size=1) #Jackal
				#self.control_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1) #Jackal
			elif('TURTLEBOT' in ROBOT_TYPE):
				self.control_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) #turtlebot
			# get data (here, it includes both ego and other robots)
			self.sensors_sub  = rospy.Subscriber('/gazebo/model_states', ModelStates, self.measurement_cb)
			#self.sensors_sub = rospy.Subscriber('/gazebo/default/pose/info', ModelStates, self.measurement_cb)
			CALIB_ANGLE_THETA = 0.0 # don't apply this to simulation mode
			for i in self.other_robots_arr:
				self.others_pose.update({i: np.zeros(3)}) # add a dictionary slot for every one
			for obs in self.list_obs:
				self.others_pose.update({obs: np.zeros(3)}) # add a dictionary slot for any obstacle
		else:
			#self.control_pub  = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1) #Jackal
			self.control_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Jackal
			# get data
			self.sensors1_sub = rospy.Subscriber("/mocap_node/Jackal%d/pose" %self.idx, PoseStamped, self.measurement_cb)
			self.other_sensors_sub = []
			for i in self.other_robots_arr:
				self.others_pose.update({i: np.zeros(3)}) # add a dictionary slot for every one
				# send the extra parameter of which one are you
				self.other_sensors_sub.append(rospy.Subscriber("/mocap_node/Jackal%d/pose" %(i+1), PoseStamped, self.other_meas_cb, i))
			for obs in self.list_obs:
				self.others_pose.update({i: np.zeros(3)}) # add a dictionary slot for any obstacle
				self.other_sensors_sub.append(rospy.Subscriber("/mocap_node/%s/pose" %(obs), PoseStamped, self.other_meas_cb, obs))
			
			self.sensors2_sub = rospy.Subscriber("/imu/data", Imu, self.imu_cb)
			self.sensors3_sub = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_cb)

		if(False):
			self.xinterp, self.uinterp = self.GetClosestInterpPoint(self.x_ref_hires, self.map_bit_2_label[self.curr_state])
		else:
			self.xinterp = self.x_ref
			self.uinterp = self.u_ref

		self.control(self.K, self.xinterp, self.uinterp, do_calc=False) #send a 0 for the first time!

		self.timer = -1E6
		self.funnel_timer = 0.0
		self.disable_debug = True #False
		self.r = rospy.Rate(self.Fs)


	def odom_cb(self, msg):
		self.last_linvel = msg.twist.twist.linear

	def imu_cb(self, msg):
		#self.linvel = msg.linear_acceleration  # this is not correct, need to integrate
		self.last_rotvel = msg.angular_velocity

	#/gazebo/model_states. This runs in 100Hz, so we basically just ignore most of the measurements
	# or from mocap if you run in lab with optitrack
	def measurement_cb(self, msg):
		#if(self.all_topics_loaded == False):
		#	return
		self.all_topics_loaded = True
		#print(self.msg_num)
		self.msg_num += 1

		if(SIMULATION):
			# decipher the msg comm. coming from gazebo (ground truth). later, switch to vicon in lab
			if('JACKAL' in ROBOT_TYPE):
				i      = msg.name.index('jackal%d' %(self.idx)) # Jackal
				#i      = msg.name.index('jackal') # Jackal
			elif('TURTLEBOT' in ROBOT_TYPE):
				i      = msg.name.index('mobile_base') # TurtleBot
			pose = msg.pose[i].position
			Q    = msg.pose[i].orientation
			self.linvel = self.TransformVectorToBody(msg.twist[i].linear, Q)
			self.rotvel = msg.twist[i].angular
			# get location of other robots
			for j in self.other_robots_arr:
				if('JACKAL' in ROBOT_TYPE):
					i      = msg.name.index('jackal%d' %(j)) # Jackal
					#i      = msg.name.index('jackal') # Jackal
				poseother = msg.pose[i].position
				Qother    = msg.pose[i].orientation
				anglesother = euler_from_quaternion([Qother.x, Qother.y, Qother.z, Qother.w])
				self.others_pose[j] = np.array([poseother.x, poseother.y, anglesother[2]])
			# get location of other obstacles
			for obs in self.list_obs:
				try:
					i      = msg.name.index(obs) 
					poseother = msg.pose[i].position
					Qother    = msg.pose[i].orientation
					anglesother = euler_from_quaternion([Qother.x, Qother.y, Qother.z, Qother.w])
					self.others_pose[obs] = np.array([poseother.x, poseother.y, anglesother[2]])
				except:
					self.others_pose[obs] = np.array([-1000., -1000., 0.0])
		else:
			#import pdb; pdb.set_trace()
			pose   = msg.pose.position
			pose.x = pose.x*ft2m
			pose.y = pose.y*ft2m
			Q      = msg.pose.orientation
			# get the rates in the same rate as the pose
			self.linvel = self.last_linvel
			self.rotvel = self.last_rotvel
		
		# get euler angles to know heading
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		theta = angles[2] + CALIB_ANGLE_THETA # remove bias from yaw in case we're in the lab
		
		# if we move quadrant, unwrap from the previous angle
		theta = np.unwrap([self.pose[2], theta])

		# store for other uses
		self.pose = np.array([pose.x, pose.y, theta[1]]) # (gazebo->mine axes match )

	def other_meas_cb(self, msg, i):
		#import pdb; pdb.set_trace()
		pose   = msg.pose.position
		pose.x = pose.x*ft2m
		pose.y = pose.y*ft2m
		Q      = msg.pose.orientation
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

		# GUY, TODO: find a way to know which jackal is broadcasting
		self.others_pose[i] = np.array([pose.x, pose.y, angles[2]]) # (gazebo->mine axes match )

	# main loop
	def Run(self):
		while not self.all_topics_loaded:
			# wait till we start to get measurements
			self.r.sleep()

		while not rospy.is_shutdown():
			#import pdb; pdb.set_trace()
			#if(self.msg_num > 120*1000):
			#	import pdb; pdb.set_trace()
			# do control and call the robot (you first execute the control for the given ellipse you're in,
			# only then, you see if you're at a new funnel/knot point and switch controls)
			if(True):
				self.xinterp, self.uinterp = self.GetClosestInterpPoint(self.x_ref_hires, self.map_bit_2_label[self.curr_state])
			else:
				self.xinterp = self.x_ref
				self.uinterp = self.u_ref

			# populates the funnel restriction for the ego-robot
			self.funnel_sensors = self.SenseEnvironment()
			# if we currently run through an action that is supposed to be 
			# disabled, then quickly stop!!
			if(self.slugs._Nsens > 0):
				if(self.funnel_sensors[self.action] == True):
					print(self.colorize('RED', 'need to do emergency stop'))
					self.do_calc = False

			# take action!
			self.control(self.K, self.xinterp, self.uinterp, do_calc=self.do_calc)

			# check if we're in the next ellipse on the same funnel
			if(self.curr_ell < self.N_ellipse - 1):
				# because sometimes we run too fast and we're going through the ellipses in a flash
				# this goes from the end of the funnel, backwards, to see the furthest away ellipse
				# that it is in
				next_ell = self.N_ellipse - 1
				while( next_ell > self.curr_ell ):
					if(self.CheckEllipseCompletion(self.map_bit_2_label[self.curr_state], \
												   self.action, next_ell) == True):
						self.K     = self.mps[self.action]['K'][next_ell]
						self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.action]['xcenter'][next_ell], \
													self.map_bit_2_label[self.curr_state])
						self.u_ref = self.mps[self.action]['unom'][next_ell]
						# this doesn't change when we go through another ellipse
						#self.x_ref_hires = self.mps[ self.actions[self.curr_state] ]['xtraj']
						#self.u_ref_hires = self.mps[ self.actions[self.curr_state] ]['utraj']
						self.curr_ell = next_ell
						#self.do_calc = True
						#self.timer = self.msg_num
						break;
					next_ell -= 1

			#import pdb; pdb.set_trace()
			# check if we're in the next funnel (and overflow when in the final one) and if next action
			# is self.STAY_IN_PLACE, keep polling until it's unstuck. if we stopped in the middle, check until you have
			# a different action to take
			if(self.do_calc == False or self.action == self.STAY_IN_PLACE):
				#just a way to reduce the frequency that we call setinitialpos from slugs interface
				if((self.msg_num % MEAS_FS) <= 10):
					# change the funnel in slugs (setpos) to the actual location it is currently at
					# to get a better chance to get a new route
					self.curr_state = self.GetClosestNode(self.pose)
					print('in a stopped situation: R%d is now in %s (%d), checking sensors again ' %(self.idx, self.map_bit_2_label[self.curr_state], \
																					self.curr_state))
					self.funnel_sensors = self.SenseEnvironment() # sensing might be different now because we might be far from first ellipse
					# reset the position in slugs
					self.curr_state, self.action = self.slugs.SetInitialPos(self.curr_state, self.funnel_sensors)
					self.curr_ell = self.FindNextValidEllipse(self.action)
					# extension to the telemetry file
					#self.states_fid.write('%d;%s;%d\n' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
					logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
					# get how many points are in this new motion primitive
					self.N_ellipse = len(self.mps[ self.action]['V'])
					# update all the new motion parameters
					self.K     = self.mps[self.action]['K'][self.curr_ell]
					self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.action]['xcenter'][self.curr_ell], \
												self.map_bit_2_label[self.curr_state])
					self.u_ref = self.mps[self.action]['unom'][self.curr_ell]
					self.x_ref_hires = self.mps[ self.action]['xtraj']
					self.u_ref_hires = self.mps[ self.action]['utraj']
					self.timer = self.msg_num
					if(self.action != self.STAY_IN_PLACE):
						# now, get the new next state
						for key, val in self.G[self.map_bit_2_label[self.curr_state]].items():
							if( val['motion'] == self.action):
								self.next_state = self.map_label_2_bit[key]
								break
						# sensing what happens in the future funnel
						next_funnel_sensors = self.SenseEnvironment(funnel=self.map_bit_2_label[self.next_state]) 
						self.next_state, self.next_action = \
								self.slugs.FindNextStep(self.next_state, next_funnel_sensors)
						self.do_calc = True
						print(self.colorize('GREEN', 'R%d is re-routing with action=%d ...' %(self.idx, self.action)) )
						if(self.next_action==self.STAY_IN_PLACE):
						# we're stuck for now, check again later
							print(self.colorize('YELLOW', 'oh no, R%d got \'stay in place\' for next action (%d)' %(self.idx, \
																												   self.STAY_IN_PLACE)) )
					else:
						# it's still in do not calculate new controls
						print(self.colorize('RED', 'R%d is still getting stay in place action ...' %self.idx) )
			elif(self.next_action == self.STAY_IN_PLACE):
				if(self.msg_num - self.timer > 1*MEAS_FS):
					# if we still don't have any progress after 10*1/100 seconds (msg_num is in gazebo's framerate), then try resetting with 
					# action=self.STAY_IN_PLACE. this will force a search for a new route right now
					self.do_calc = False #self.action = self.STAY_IN_PLACE
				else:
					# sensing what happens in the future funnel (as if robot is now there)
					next_funnel_sensors = self.SenseEnvironment(funnel=self.map_bit_2_label[self.next_state]) 
					self.next_state, self.next_action = \
						self.slugs.FindNextStep(self.next_state, next_funnel_sensors)
					if(self.next_action != self.STAY_IN_PLACE):
						self.do_calc = True
						print(self.colorize('GREEN', 'R%d got a good (next) action now ...' %self.idx))
					else:
						print(self.colorize('YELLOW', 'R%d is still getting stay in place (but we\'re not there yet) ...' %self.idx) )
			elif (self.CheckInNextFunnel(self.map_bit_2_label[self.next_state], self.next_action) == True):
				self.funnel_timer = self.msg_num
				#rospy.loginfo
				print('R%d reached funnel %d (%s) with action %d' \
							  %(self.idx, self.next_state, self.map_bit_2_label[self.next_state], self.action))
				#we've reached the beginning of a new funnel, so update the states
				self.slugs.MoveNextStep()
				self.curr_state = self.next_state
				# sometimes the first ellipse has zero speed and it causes trouble. skip ellipses ahead
				# it's fine because if it doesn't have controls, then it is also at the same place as first ellipse.
				# this is usually only one ellipse. Maybe should take care of it when creating the funnels or
				self.curr_ell = self.FindNextValidEllipse(self.next_action)
				self.action = self.next_action
				if(self.action == self.STAY_IN_PLACE):
					self.do_calc = False # now, stop in place
				# extension to the telemetry file
				logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
				# get how many points are in this new motion primitive
				self.N_ellipse = len(self.mps[ self.action]['V'])
				# update all the new motion parameters
				self.K     = self.mps[self.action]['K'][self.curr_ell]
				self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.action]['xcenter'][self.curr_ell], \
											self.map_bit_2_label[self.curr_state])
				self.u_ref = self.mps[self.action]['unom'][self.curr_ell]
				self.x_ref_hires = self.mps[ self.action]['xtraj']
				self.u_ref_hires = self.mps[ self.action]['utraj']
				self.timer = self.msg_num
				# now, get the new next state
				for key, val in self.G[self.map_bit_2_label[self.curr_state]].items():
					if( val['motion'] == self.action):
						self.next_state = self.map_label_2_bit[key]
						break
				# sensing what happens in the future funnel
				next_funnel_sensors = self.SenseEnvironment(funnel=self.map_bit_2_label[self.next_state]) 
				self.next_state, self.next_action = \
					self.slugs.FindNextStep(self.next_state, next_funnel_sensors)
				if(self.next_action==self.STAY_IN_PLACE):
					# we're stuck for now, check again later
					#import pdb; pdb.set_trace()
					#self.do_calc = False
					print(self.colorize('YELLOW', 'oh no, R%d got \'stay in place\' for next action (%d)' %(self.idx,self.STAY_IN_PLACE)) )

			# GUY TODO, remove when done with debugging
			if((self.msg_num - self.funnel_timer > 15*MEAS_FS) and (self.disable_debug == False)):
				print(self.map_bit_2_label[self.next_state])
				print(self.next_action)
				print(self.pose)
				find_non_zero = self.FindNextValidEllipse(self.next_action)
				S, ellipse_pose = self.GetCoordAndEllipseFromLabel(self.map_bit_2_label[self.next_state], self.next_action, find_non_zero)
				print( self.InEllipse(S, ellipse_pose, pose=self.pose) )
				import pdb; pdb.set_trace() 
			
			# output the telemetry to file
			self.telemetry()
			self.r.sleep()
		#end of main loop, Fs [Hz]

	# populates the funnel restrictions variable by deciding if there's a robot or obstacle
	# somewhere in that funnel
	def SenseEnvironment(self, funnel=None):
		tic = timer()
		# doesn't do anything if we run in a single mode robot
		if(self.slugs._Nsens > 0):
			# reset the measurements
			funnel_sensors = dict(zip(range(self.slugs._Nsens), [False]*self.slugs._Nsens))
			# the funnel you're currently in
			if(funnel == None):
				# here we're checking sensor for the actual location of the robot
				funnel = self.map_bit_2_label[self.curr_state]
				# GUY: TBR the +1 is a dirty fix to not be affected by an obstacle that touches the ellipse behind
				# the ego robot. we pay by also not being completly safe in that ellipse infront of the ego robot.
				# the rationale is that the next ellipse covers current ellipse and the area not covered by next
				# that is infront of the robot in the current ellipse, is minimal
				curr_ell = np.min([self.curr_ell + 1, self.N_ellipse-1])
			else:
				# here we're checking for the next funnel usually
				curr_ell = 0
				
			# go over all funnels
			for mp_i, mp in self.mps.items():
				# go through ellipses in this motion primitive, starting from where we currently are
				# because there's no need of stopping if it was in the past
				for S_i in range(curr_ell, len(mp['V'])):
					# GUY, TODO, i'm just ignoring the first ellipse so it won't fall if the is an obstacle behind the robot
					# but this should be better handled
					# get the ellipse coordinates and matrix in motion primitive's relative coordinates
					S, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, mp_i, S_i)
					S = S[:2,:2]
					ellipse_pose = ellipse_pose[:2]
					e = gf.Ellipse(ellipse_pose, S)
					# first, check other robots (excluding ego)
					for r in self.other_robots_arr:
						adv_pose = self.others_pose[r]
						# check if the x-y position is in some ellipse.
						# basically we treat ego-robot as a point and project (just add) the maximum between length-width
						# to the other robot to detect collision between ellipse to rectangle.
						# GUY, TODO, see if we can actually project with theta of ego to be less conservative
						box_approx_of_robot  = gf.Rectangle(adv_pose[:2], FL_L*2.0, FL_L*2.0)
						overlaps = gf.TestIntersectionRectangleEllipse(box_approx_of_robot, e)
						# 2nd option, not good, just treat them as point robots
						#overlaps = self.InEllipse(S, ellipse_pose, pose=adv_pose)
						if(overlaps == True):
							funnel_sensors[mp_i] = True
							if(self.action == mp_i):
								# do less printing on screen
								print('Robot #%d caused funnel %d to be blocked (ellipse %d)' %(r, mp_i, S_i))
							#import pdb; pdb.set_trace()
							break
					#means something already occupied it so no need to check other things as well
					if(funnel_sensors[mp_i] == True):
						break
					# next, check obstacles
					for obs in self.list_obs:
						adv_pose = self.others_pose[obs]
						# GUY TODO, obviously there should be some method to estimate the width&length of the obstacle
						# meanwhile, assume the obs is a person and it has got same dim. as the jackal
						box_approx_of_robot  = gf.Rectangle(adv_pose[:2], FL_L*2.0, FL_L*2.0)
						overlaps = gf.TestIntersectionRectangleEllipse(box_approx_of_robot, e)
						if(overlaps == True):
							funnel_sensors[mp_i] = True
							if(self.action == mp_i):
								# do less printing on screen, only if it is blocking us
								print('%s caused funnel %d to be blocked (ellipse %d)' %(obs, mp_i, S_i))
							#import pdb; pdb.set_trace()
							break
					#means something already occupied it so no need to check other things as well
					if(funnel_sensors[mp_i] == True):
						break
						
			toc = timer()
			self._sensing_function_time = toc-tic
			return funnel_sensors
		else:
			toc = timer()
			self._sensing_function_time = toc-tic
			return {}
							
	def telemetry(self):
		# telemetry
		now = rospy.get_rostime()
		t = now.secs + now.nsecs/1.0E9
		#rospy.logdebug
		try:
			logger.debug('%.3f;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;' \
					 '%.2f;%.2f;%.2f;%.2f;%.2f;%.3f' \
					   %(t, self.msg_num, self.pose[0], self.pose[1], self.pose[2]*180.0/np.pi, \
						 self.x_ref[0], self.x_ref[1], self.x_ref[2]*180.0/np.pi, self.u_ref[0], self.u_ref[1], \
						self.curr_state, self.curr_ell, int(self.do_calc), self.action, \
						self.u[0], self.u[1], self.linvel.x, self.linvel.y, self.linvel.z,\
						self.rotvel.x, self.rotvel.y, self.rotvel.z, self.xinterp[0],self.xinterp[1],self.xinterp[2]*180.0/np.pi, \
						self.uinterp[0],self.uinterp[1],self._sensing_function_time))
		except:
			pass

	# rotate vectors from world frame to body frame
	def TransformVectorToBody(self, vect, q):
		v = Vector3Stamped()
		v.vector.x = vect.x
		v.vector.y = vect.y
		v.vector.z = vect.z

		t = TransformStamped()

		quaternion = np.array((q.x, q.y, q.z, q.w))
		quat_conj = np.array((-quaternion[0], -quaternion[1], \
							  -quaternion[2], quaternion[3]))
		quat_inv = quat_conj / np.dot(quaternion, quaternion)


		t.transform.rotation.x = quat_inv[0]
		t.transform.rotation.y = quat_inv[1]
		t.transform.rotation.z = quat_inv[2]
		t.transform.rotation.w = quat_inv[3]

		vt = do_transform_vector3(v, t)

		return vt.vector #np.array([vt.vector.x, vt.vector.y, vt.vector.z ])

	# y/x = 1/(tau*s+1) ==>  y(k) = T/(tau+T) * (x(k)+tau/T*y(k-1))
	def LPF(self, x, y_prev, tau=1.):
		T = 1./self.Fs
		y = T/(tau + T) * (x + tau/T*y_prev)
		return y
	
	# conversion from arbitrary location on map to the closest funnel (location & orientation)
	def GetClosestNode(self, pose):
		orient = int(np.round( pose[2] / (np.pi/2.0) ) % 4)  # what quadrant you're closest to
		label = 'H' + str(orient) + 'X' + str(find_nearest(self.W_xgrid, pose[0])) + \
									'Y' + str(find_nearest(self.W_ygrid, pose[1]))

		return self.map_label_2_bit[ label ]

	# check if a point is in a predefined ellipse
	def CloseEnoughToNextPoint(self, funnel, action, ellipse_num, dist=0.2, ang=0.2):
		__, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, action, ellipse_num)

		# create a circle/ellipse and use the current infrastructure to see if we're close enough to switch controls
		S = np.diag([dist**-2, dist**-2, ang**-2])

		if( self.InEllipse(S, ellipse_pose) ):
			return True

		return False

	# point is inside the ellipse iff (x-x0)'S(x-x0)<=0
	def InEllipse(self, S, x0, pose=None):
		# adjust both angles to be (-pi : +pi]
		#x0[2] = (( -x0[2] + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
		if(pose is None):
			xbar = self.pose - x0
		else:
			xbar = pose - x0

		# find the smallest angle difference
		ang_diff = (xbar[2] + np.pi) % (2*np.pi) - np.pi
		xbar[2] = ang_diff
		#(x-x0)'S(x-x0)<=0
		if(S.shape[0] == 3):
			ret = xbar.dot(S.dot(xbar))
		else:
			# we only care about two dimensions x,y (to check when obstacle is in the path)
			xbar = xbar[:2]
			ret = xbar.dot(S.dot(xbar))

		return (ret<=1)

	# sometimes the first ellipse has zero speed and it causes trouble. skip ellipses ahead
	def FindNextValidEllipse(self, action):
		find_non_zero = 0
		N_tot_ellipses = len(self.mps[action]['V']) # because it could also be not the action we currently take
		while True:
			if(np.linalg.norm(self.mps[action]['unom'][find_non_zero]) < 0.01):
				find_non_zero += 1
				if(find_non_zero == N_tot_ellipses):
					break
			else:
				break
		return find_non_zero

	# check if the pose of robot is in the ellipse
	def CheckEllipseCompletion(self, funnel, action, ellipse_num):
		# gets the S matrix already rotated to the right global orientation
		S, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, action, ellipse_num)
		# check if the ellipse equation holds
		if( self.InEllipse(S, ellipse_pose) ):
			return True

		return False

	# check if the pose of robot is in the next funnel
	def CheckInNextFunnel(self, funnel, action):
		# see if we are at the first ellipse of the new funnel
		find_non_zero = self.FindNextValidEllipse(action)
		# check if we are in the ellipse
		return self.CheckEllipseCompletion(funnel, action, find_non_zero)

	# extract S (rotated to current pose) and center of ellipse from current mp (funnel&ellipse)
	def GetCoordAndEllipseFromLabel(self, funnel, action, ellipse_num):
		mp = self.mps[action]

		S  = mp['V'][ellipse_num]
		x0 = mp['xcenter'][ellipse_num]

		e_center, rotmat = self.ConvertRelPos2Global(x0, funnel)
		# rotate the ellipse to fit the global coordinate system
		# x'*V*x<1 & z=R*x ==> x=invR*x ==> z'*invR'*V*invR*z < 1
		rotmat = np.array([[rotmat[0,0], rotmat[0,1], 0.], [rotmat[1,0],rotmat[1,1],0.], [0.,0.,1.]]) # convert to 3D
		rotmat = np.linalg.inv(rotmat)
		S = rotmat.T.dot(S.dot(rotmat))

		return S, e_center

	# coordinates transfer from mp frame to global frame
	def ConvertRelPos2Global(self, rel_pose, funnel):
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', funnel)] # extract first ellipse pose
		xs = self.W_xgrid[xs]
		ys = self.W_ygrid[ys]

		rotmat = GetRotmat(orient)

		if(orient == 3):
			orient = -1

		theta = rel_pose[2]
		x_rel = rotmat.dot(rel_pose[:2]) # just x,y

		pose = np.array([xs, ys]) + x_rel # location in the world of that ellipse
		pose = np.hstack((pose, theta + orient*np.pi/2.0 ))

		return pose, rotmat

	# limits the commands in such a way as to preserve the ratio between them
	def LimitCmds(self, V, omega):

		if(V <= 0.0):
			V = 0.0
		else:
			ratiov = np.abs(V / umax)
			ratiow = np.abs(omega / ( V/self.L *np.tan(delmax)))

			if(ratiow > ratiov and ratiow > 1.):
				omega = omega/ratiow
				V = V/ratiow
			elif(ratiov > ratiow and ratiov > 1.):
				omega = omega/ratiov
				V = V/ratiov

		return V, omega

	# gives look-ahead point to where to robot should go (and the commands it needs to take)
	def GetClosestInterpPoint(self, rel_poses, funnel):
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', funnel)] # extract first ellipse pose
		xs = self.W_xgrid[xs]
		ys = self.W_ygrid[ys]

		rotmat = GetRotmat(orient)

		if(orient == 3):
			orient = -1

		__, N = rel_poses.shape
		theta = rel_poses[2,:]
		x_rel = np.dot(rotmat, rel_poses[:2,:])
		poses = np.array([[xs], [ys]]) + x_rel # location in the world of that ellipse
		#import pdb; pdb.set_trace()
		diff = self.pose[:2].reshape((2,1))-poses
		dist = np.linalg.norm( diff, axis=0 )

		index = np.argmin(dist, axis=0) + 0 #1 #2
		# skip the zero velocities ellipses
		while((index < N-1) and (np.abs(self.u_ref_hires[1,index]) < 0.05)):
			index += 1
		if(index > N-1):
			index = N-1

		pose_ref = np.hstack((poses[:,index], theta[index] + orient*np.pi/2.0 ))
		u_ref = self.u_ref_hires[:,index]

		return pose_ref, u_ref

	# compute the actual controls to the robot
	def control(self, K, x_0, u_0, do_calc=True):
		#if(do_calc):

		#print('x_0=(%f,%f,%f)\tu_0=(%f,%f)' %(x_0[0],x_0[1],x_0[2], u_0[0],u_0[1]))
		if(self.all_topics_loaded == False):
			return

		if(do_calc == True):
			# error comes in global coordinates while the K computed in the optimization
			# assumes y-up x-right and we face right
			label = self.map_bit_2_label[self.curr_state]
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
			#if(do_calc == False):
			u[1] = self.LPF(u[1], self.u[1], tau=2.)
			self.v_prev = u[1]
			#else:
				# this is basically a reset when switching funnels
				#self.v_prev = 0.0
				#u[1] = 0.0

			#saturations
			if(u[1] > umax):
				u[1] = umax
			if(u[1] < -umax):
				u[1] = -umax
			if(u[0] > delmax):
				u[0] = delmax
			if(u[0] < -delmax):
				u[0] = -delmax

			self.u = u

		else:
			u = self.u*0.0  #stay in place

		self.actuation(u)

	# send the commands to ROS
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
		#saturations
		#V,W = self.LimitCmds(u[1], u[1] / self.L * np.tan(u[0]))
		#self.u = np.array([W,V])
		#t.angular.z = W
		t.angular.z = 0.9*0.5/0.3 * u[1] / self.L * np.tan(u[0]) # delta; 1.0/umax is a gain to compensate 1.0/umax *
		# the fact that I am running in less than 1.0m/s in the lab
		#t.linear.x  = V #u[1] # V
		t.linear.x  = u[1] # V
		#print('v = %f' %u[1])
		self.control_pub.publish(t)

	# utility function to highlight the text on the console
	def colorize(self, level, string):
		if sys.platform in ['win32', 'cygwin']:
			# Message with color is not yet supported in Windows
			return string

		else:
			colors = {'RED': 91, 'YELLOW': 93, 'WHITE': 97, 'BLUE': 94, 'GREY': 90, \
					  'PINK': 95, 'HIGHLIGHT': "7;95", 'TURQUOISE': 96, 'GREEN': 92}
			return "\033[{0}m{1}\033[0m".format(colors[level], string)
			
	def Shutdown(self):
		self.slugs.Shutdown() #hopefully destroy the memory taken by slugs
		#self.states_fid.close()
		print("Shutting down\n")

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Robot number to control.')
	parser.add_argument('--i', type=int, default=0,
					help='an integer for the robot number (index)')
	parser.add_argument('--n', type=int, default=1,
					help='an integer for total amount of robots on the field')
	parser.add_argument('--obs', type=str, default='',
					help='names of other obstacles, comma separated, must have ROS messages (gazebo or on the field)')
	args = parser.parse_args()
	print('Controller for Jackal%d' %args.i)
	list_obs = args.obs.split(',')
	#import pdb; pdb.set_trace()
	# Create a custom logger
	timenow     = localtime()
	log_file    = strftime('telemetry_%Y_%m_%d_%H_%M_%S', timenow)
	print('Started program at ' + strftime('%H:%M:%S', timenow))
	
	level       = logging.DEBUG
	logger      = logging.getLogger(__name__ + str(args.i))
	logger_state= logging.getLogger(__name__ + '_state_' + str(args.i))
	#formatter = logging.Formatter('%(asctime)s - %(message)s')
	formatter   = logging.Formatter('%(message)s')
	fileHandler = logging.FileHandler('../telemetry/r%d_%s.log' %(args.i, log_file), mode='w')
	fileHandler_state = logging.FileHandler('../telemetry/r%d_%s.state' %(args.i, log_file), mode='w')
	fileHandler.setFormatter(formatter)
	fileHandler_state.setFormatter(formatter)
	logger.setLevel(level)
	logger_state.setLevel(level)
	logger.addHandler(fileHandler)
	logger_state.addHandler(fileHandler_state)
	# set header of telemetry file
	logger.debug('t;frame;x;y;z;x_ref;y_ref;z_ref;delta_ref;u_ref;state;ellipse;control;action;delta;u;vx;vy;vz;wx;wy;wz;' \
				'xr;yr;zr;delr;ur')

	rospy.init_node('run_jackal_%d' %args.i)#, log_level=rospy.DEBUG)
	J = Jackal(args.i, args.n, list_obs)

	try:
		#rospy.spin()
		J.Run()
	except KeyboardInterrupt:
		pass
	except rospy.ROSInterruptException:
		pass
	except:
		print "Unexpected error:", sys.exc_info()[0]
		raise
	finally:
		timenow     = localtime()
		print('Ended program at ' + strftime('%H:%M:%S', timenow))
		J.Shutdown() #hopefully destroy the memory taken by slugs
	# END ALL
