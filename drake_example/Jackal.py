#!/usr/bin/env python

# general stuff
import numpy as np
import json
import argparse
import dill
import networkx as nx
import re
import sys
import os
import logging # can't handle the ros logging :(
from time import localtime, strftime, sleep
from timeit import default_timer as timer
from shutil import copyfile

# my stuff
from warehouse_map import LoadMP, GetSpecificControl, find_nearest, GetRotmat, FL_L, FL_W, UpdateRestrictionSlugsInputFile, Convert2Slugsin, CreateSlugsInputFile
from SlugsInterface import *
import GeometryFunctions_fcl as gf
from shapely.geometry import box
import global_parameters as glob_p
#import GeometryFunctions as gf

# lab or simulation
SIMULATION = False

# ROS stuff
import rospy
if(SIMULATION):
	from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped
	from gazebo_msgs.msg import ModelStates, ModelState
else:
	from geometry_msgs.msg import Twist, PoseStamped, Pose2D, TransformStamped
	from nav_msgs.msg import Odometry
	from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_from_euler
from tf2_geometry_msgs import do_transform_vector3



# class to handle a single robot comm.
class Jackal:
	def __init__(self, idx, total_robots, list_obs=[], list_robots=[], first_goal_for_gazebo=None, reactive='F'):
		self.ROBOT_TYPE = 'JACKAL'  # original JACKAL run with 'roslaunch jackal_gazebo jackal_world.launch'
		#self.ROBOT_TYPE = 'TURTLEBOT'
		#self.MAP    = 'raymond'
		self.MAP    = glob_p.MAP_KIND
		self.umax   = glob_p.UMAX #0.3 #1.0  # jackal m/sec     5.0
		self.delmax = 45.0*np.pi/180.0  # jackal rad   30.0 80
		self.MEAS_FS = 100 #100 for gazebo, not sure about vicon/optitrack

		self.SLUGS_DIR = glob_p.slugsLink #'/home/cornell/Tools/slugs_ltl_stack/src/slugs'
		self.MAP_FILE = self.MAP + '.map'
		self.SYNTH_AUTOMATA_FILE = self.MAP
		self.MP_MAP_FILE = self.MAP + '.pickle'
		self.LABEL2BIT_FILE = self.MAP + '.label2bit'
		self.pix2m  = 1.0 #0.2 #[m]
		self.ft2m   = 0.3048
		self.CALIB_ANGLE_THETA = 0.0 #I've changed the angle in the optitrack software-0.16 #[rad]

		mp_file =self.MP_MAP_FILE
		map_file=self.MAP_FILE
		aut_file=self.SYNTH_AUTOMATA_FILE
		l2b_file=self.LABEL2BIT_FILE
		
		self.check_blocks_only_at_beginning = False
		
		# Create a custom logger
		timenow     = localtime()
		log_file    = strftime('telemetry_%Y_%m_%d_%H_%M_%S', timenow)
		print('Started program at ' + strftime('%H:%M:%S', timenow))
		level          = logging.DEBUG
		self.logger      = logging.getLogger(__name__ + str(idx))
		self.logger_state= logging.getLogger(__name__ + '_state_' + str(idx))
		#formatter = logging.Formatter('%(asctime)s - %(message)s')
		formatter   = logging.Formatter('%(message)s')
		fileHandler = logging.FileHandler('../telemetry/r%d_%s.log' %(idx, log_file), mode='w')
		fileHandler_state = logging.FileHandler('../telemetry/r%d_%s.state' %(idx, log_file), mode='w')
		fileHandler.setFormatter(formatter)
		fileHandler_state.setFormatter(formatter)
		self.logger.setLevel(level)
		self.logger_state.setLevel(level)
		self.logger.addHandler(fileHandler)
		self.logger_state.addHandler(fileHandler_state)
		# set header of telemetry file
		self.logger.debug('t;frame;x;y;z;x_ref;y_ref;z_ref;delta_ref;u_ref;state;ellipse;control;action;delta;u;vx;vy;vz;wx;wy;wz;' \
					'xr;yr;zr;delr;ur;obs1x;obs1y;obs2x;obs2y')
		
		self.all_topics_loaded = False
		self.Fs = 10 # main/control loop frequency
		if(SIMULATION == False):
			self.idx = idx - 1 #this is because in the lab the index is +1
		else:
			self.idx = idx
		self.total_robots = total_robots
		self.msg_num = 0
		self.pose   = np.array([0.0, 0.0, 0.0*np.pi/2.0]) #None
		self.other_robots_arr = np.setdiff1d(np.arange(0,self.total_robots),[self.idx]) # everything but ego
		self.others_pose = {} #np.zeros((total_robots-1,3)) # in index 0 will be the first robot that is not ego
		self.list_obs = list_obs
		self.list_robots = list_robots
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
		self.external_shutdown = False
		self.reactive = reactive # Full, Semi, Graph based
		self.InSynthesisProcedure = False
		self.resynth_cnt = 0

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
		#self.states_fid = open('../telemetry/r%d_%s.states' %(idx, self.MAP), 'wt')

		self.G = nx.read_gpickle(mp_file)
		self.mps = LoadMP()
		self.Nactions = len(self.mps)
		self.STAY_IN_PLACE = self.Nactions

		dbfile = open(l2b_file, 'rb')
		self.map_label_2_bit = dill.load(dbfile)
		dbfile.close()
		# the reverse dictionary is also useful
		self.map_bit_2_label = dict((v, k) for k, v in self.map_label_2_bit.items())
		goals = map_prop['r%d' %self.idx]
		self.goals = []
		self.goals_ic = []
		for goal in goals:
			self.goals.append(self.GetClosestNode([goal['x'], goal['y'], goal['teta']]))
			self.goals_ic.append([goal['x'], goal['y'], goal['teta']])
		self.goal = 0
		self.nez = []
		no_enters = map_prop['no_enter']
		for no_enter in no_enters:
			self.nez.append(box( no_enter['x'], no_enter['y'], no_enter['X'], no_enter['Y'] ))
			
		# old, interface with the explicitStrategy of slugs
		#aut_file = aut_file + ('_r%d' %idx) + '.json'
		#aut = open(aut_file, 'r')
		#self.automata = json.load(aut)
		#aut.close()
		
		# GUY: JUST FOR TEST
		'''
		first_goal_for_gazebo = [0.7, -2.2, 1.57]
		pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
		set_first_pose = ModelState()
		set_first_pose.model_name = 'jackal%d' %self.idx
		set_first_pose.pose.position.x    = first_goal_for_gazebo[0]
		set_first_pose.pose.position.y    = first_goal_for_gazebo[1]
		q = quaternion_from_euler(0.0, 0.0, first_goal_for_gazebo[2])
		set_first_pose.pose.orientation.z = q[2]
		set_first_pose.pose.orientation.w = q[3]
		#pub.publish(set_first_pose)
		sleep(3.0) # give it a chance to actually move the robot
		pub.publish(set_first_pose)
		sleep(0.5)
		import pdb; pdb.set_trace()
		'''
		####################
		
		# new, interface the interactive mode of slugs
		tic = timer()
		#just make sure that we have the correct slugsin that corresponds to the structuredslugs
		if(reactive == 'S'):
			print(self.colorize('YELLOW','the following is to make sure the input is valid. can remove this at some point in the future'))
			Convert2Slugsin(aut_file, [self.idx]) 
		
		self.slugs = SlugsInterface(aut_file + ('_r%d' %self.idx), simulate=False, slugsLink = self.SLUGS_DIR)
		toc = timer()
		if(not self.slugs.enabled):
			print('Cannot create slugs interface.')
			return
		print('Done loading slugs (%.2f[sec]).' %(toc-tic))
		self.slugs.DiscoverInputs()
		self.slugs.DiscoverOutputs()
		self.slugs.DiscoverGoals()
		self.slugs.GetInitialPos()
		#import pdb; pdb.set_trace()
		# GUY: changed from Nsens to _Nout due to semi-reactive
		#self.funnel_sensors = dict(zip(range(self.slugs._Nsens), [False]*self.slugs._Nsens))		
		self.funnel_sensors = dict(zip(range(self.Nactions), [False]*self.Nactions))
		self.blocking_obs = [[]]*self.Nactions #keep holding the locations of blocking obstacles
		
		# use this break point as a synchronizer between multiple runs
		# (not working well with the GUI.py)
		#first_goal_for_gazebo = [0.7, -2.2, 1.57]
		if(first_goal_for_gazebo == None or SIMULATION == False):
			print('Press c to start ...')
			import pdb; pdb.set_trace()
		else:
			# in the lab it should do nothing
			try:
				pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

				set_first_pose = ModelState()
				set_first_pose.model_name = 'jackal%d' %self.idx
				set_first_pose.pose.position.x    = first_goal_for_gazebo[0]
				set_first_pose.pose.position.y    = first_goal_for_gazebo[1]
				q = quaternion_from_euler(0.0, 0.0, first_goal_for_gazebo[2])
				set_first_pose.pose.orientation.z = q[2]
				set_first_pose.pose.orientation.w = q[3]
				sleep(3.0) # give it a chance to actually move the robot
				pub.publish(set_first_pose)
				sleep(1.0) # give it a chance to actually move the robot
				
			except:
				pass

		#self.states, self.actions = GetSpecificControl(self.automata, self.map_bit_2_label, debug=False)
		self.curr_state, self.action = self.slugs.GetNumericState()
		#self.states_fid.write('%d;%s;%d\n' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
		self.logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
		#import pdb; pdb.set_trace()
		# now, get the new next state
		if(self.action == self.STAY_IN_PLACE):
			if(self.list_robots == []):
				print('oh no, R%d starts with action stay in place' %(self.idx))
			else:
				print('oh no, R%d (%s) starts with action stay in place' %(self.idx, self.list_robots[self.idx]))
			self.next_state =  self.curr_state
			self.do_calc = False
		else:
			for key, val in self.G[self.map_bit_2_label[self.curr_state]].items():
				if( val['motion'] == self.action):
					self.next_state = self.map_label_2_bit[key]
					break
		self.next_state, self.next_action = \
			self.slugs.FindNextStep(self.next_state, self.funnel_sensors)
		#import pdb; pdb.set_trace()
		if(self.list_robots == []):
			print('R%d starts in region %s (%d)' %(self.idx, self.map_bit_2_label[self.curr_state],self.curr_state))
		else:
			print('R%d (%s) starts in region %s (%d)' %(self.idx, self.list_robots[self.idx], self.map_bit_2_label[self.curr_state],self.curr_state))

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
		far_away_array = np.array([-100.,-100.,0.])
		# handle the ROS topics
		if(SIMULATION):
			# send commands
			if('JACKAL' in self.ROBOT_TYPE):
				self.control_pub = rospy.Publisher('/jackal%d/jackal_velocity_controller/cmd_vel' %self.idx, Twist, queue_size=1) #Jackal
				#self.control_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1) #Jackal
			elif('TURTLEBOT' in self.ROBOT_TYPE):
				self.control_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) #turtlebot
			# get data (here, it includes both ego and other robots)
			self.sensors_sub  = rospy.Subscriber('/gazebo/model_states', ModelStates, self.measurement_cb)
			#self.sensors_sub = rospy.Subscriber('/gazebo/default/pose/info', ModelStates, self.measurement_cb)
			self.CALIB_ANGLE_THETA = 0.0 # don't apply this to simulation mode
			for i in self.other_robots_arr:
				self.others_pose.update({i: far_away_array}) # add a dictionary slot for every one
			for obs in self.list_obs:
				self.others_pose.update({obs: far_away_array}) # add a dictionary slot for any obstacle
		else:
			#self.control_pub  = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1) #Jackal
			if(self.list_robots == []):
				self.control_pub  = rospy.Publisher('/jackal%d/cmd_vel'%(self.idx+1), Twist, queue_size=1) #Jackal
			else:
				self.control_pub  = rospy.Publisher('/%s/cmd_vel'%(self.list_robots[self.idx]), Twist, queue_size=1) #Jackal
			# get data
			# optitrack
			#self.sensors1_sub = rospy.Subscriber("/mocap_node/Jackal%d/pose" %self.idx, PoseStamped, self.measurement_cb)
			# vicon
			if(self.list_robots == []):
				# because we changed idx to be the index, and not represent the name which starts at jackal1 in the lab
				self.sensors1_sub = rospy.Subscriber("/vicon/jackal%d/jackal%d" %(self.idx+1,self.idx+1), TransformStamped, self.measurement_cb)
			else:
				self.sensors1_sub = rospy.Subscriber("/vicon/%s/%s" %(self.list_robots[self.idx],self.list_robots[self.idx]), \
													 TransformStamped, self.measurement_cb)
			self.other_sensors_sub = []
			for i in self.other_robots_arr:
				self.others_pose.update({i: far_away_array}) # add a dictionary slot for every one
				# send the extra parameter of which one are you
				# optitrack
				#self.other_sensors_sub.append(rospy.Subscriber("/mocap_node/Jackal%d/pose" %(i+1), PoseStamped, self.other_meas_cb, i))
				# vicon
				if(self.list_robots == []):
					self.other_sensors_sub.append(rospy.Subscriber("/vicon/jackal%d/jackal%d" %(i+1,i+1), TransformStamped, self.other_meas_cb, i))
				else:
					self.other_sensors_sub.append(rospy.Subscriber("/vicon/%s/%s" %( \
						self.list_robots[i],self.list_robots[i]), TransformStamped, self.other_meas_cb, i))
					
			for obs in self.list_obs:
				self.others_pose.update({obs: far_away_array}) # add a dictionary slot for any obstacle
				# optitrack
				#self.other_sensors_sub.append(rospy.Subscriber("/mocap_node/%s/pose" %(obs), PoseStamped, self.other_meas_cb, obs))
				self.other_sensors_sub.append(rospy.Subscriber("/vicon/%s/%s" %(obs,obs), TransformStamped, self.other_meas_cb, obs))
			
			if(self.list_robots == []):
				self.sensors2_sub = rospy.Subscriber("/jackal%d/imu/data"%(self.idx+1), Imu, self.imu_cb)
				self.sensors3_sub = rospy.Subscriber("/jackal%d/jackal_velocity_controller/odom"%(self.idx+1), Odometry, self.odom_cb)
			else:
				self.sensors2_sub = rospy.Subscriber("/%s/imu/data"%(self.list_robots[self.idx]), Imu, self.imu_cb)
				self.sensors3_sub = rospy.Subscriber("/%s/jackal_velocity_controller/odom"%(self.list_robots[self.idx]), Odometry, self.odom_cb)

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
		reject_measurement = False

		if(SIMULATION):
			# decipher the msg comm. coming from gazebo (ground truth). later, switch to vicon in lab
			if('JACKAL' in self.ROBOT_TYPE):
				i      = msg.name.index('jackal%d' %(self.idx)) # Jackal
				#i      = msg.name.index('jackal') # Jackal
			elif('TURTLEBOT' in self.ROBOT_TYPE):
				i      = msg.name.index('mobile_base') # TurtleBot
			pose = msg.pose[i].position
			Q    = msg.pose[i].orientation
			self.linvel = self.TransformVectorToBody(msg.twist[i].linear, Q)
			self.rotvel = msg.twist[i].angular
			# get location of other robots
			for j in self.other_robots_arr:
				if('JACKAL' in self.ROBOT_TYPE):
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
			try:
				# optitrack - PoseStamped
				pose   = msg.pose.position
				pose.x = pose.x*self.ft2m
				pose.y = pose.y*self.ft2m
				Q      = msg.pose.orientation
			except:
				# vicon - TransformStamped
				pose   = msg.transform.translation
				pose.x = pose.x # this is already in [m]
				pose.y = pose.y # this is already in [m]
				Q      = msg.transform.rotation
				#vicon has some slips, so reject the measurement if it wasn't good
				if( np.abs(pose.x-self.pose[0])>0.5 or np.abs(pose.y-self.pose[1])>0.5 ):
					# do not ignore the first measurements, otherwise you have nothing
					if(self.msg_num > 5):
						reject_measurement = True
			
			# get the rates in the same rate as the pose
			self.linvel = self.last_linvel
			self.rotvel = self.last_rotvel
		
		# get euler angles to know heading
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		theta = angles[2] + self.CALIB_ANGLE_THETA # remove bias from yaw in case we're in the lab
		
		# if we move quadrant, unwrap from the previous angle
		theta = np.unwrap([self.pose[2], theta])

		# store for other uses
		if(not reject_measurement):
			self.pose = np.array([pose.x, pose.y, theta[1]]) # (gazebo->mine axes match )
		else:
			print(self.colorize('YELLOW','R%d, rejecting measurement from vicon' %self.idx))

	def other_meas_cb(self, msg, i):
		reject_measurement = False
		
		try:
			# optitrack - PoseStamped
			pose   = msg.pose.position
			pose.x = pose.x*self.ft2m
			pose.y = pose.y*self.ft2m
			Q      = msg.pose.orientation
			angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		except:
			# vicon - TransformStamped
			pose   = msg.transform.translation
			pose.x = pose.x  # this is already in [m]
			pose.y = pose.y  # this is already in [m]
			Q      = msg.transform.rotation
			angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
			#vicon has some slips, so reject the measurement if it wasn't good
			if(i == 'Helmet'):
				if(pose.z < 1.0):
					#and self.msg_num > 5):
					reject_measurement = True
			elif( np.abs(pose.x-self.others_pose[i][0])>0.5 or \
			    np.abs(pose.y-self.others_pose[i][1])>0.5 ):
				# do not ignore the first measurements, otherwise you have nothing
				if(self.msg_num > 5):
					reject_measurement = True
				# only accept Helmet if z>1.0m (meaning: I wear it)
				# GUY, maybe remove in the future
			
			#GUY: JUST TO DEBUG, REMOVE IT IMEDIATELY
			#import pdb; pdb.set_trace()
			#if(i==0):
			#	pose.x -= 20.  # this is already in [m]
			#	pose.y -= 20.  # this is already in [m]

		# save it only if it is not garbage
		if(not reject_measurement):
			self.others_pose[i] = np.array([pose.x, pose.y, angles[2]]) # (gazebo->mine axes match )
		else:
			if((i == 'Helmet' and pose.z >= 1.0) or (i != 'Helmet')):
				print(self.colorize('YELLOW','R%d, rejecting measurement of obstacle from vicon' %self.idx))

	# main loop
	def Run(self):
		while not self.all_topics_loaded:
			# wait till we start to get measurements
			self.r.sleep()

		while not rospy.is_shutdown():
			if(self.external_shutdown):
				self.external_shutdown = False
				break
			
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
			if(not self.check_blocks_only_at_beginning):
				self.funnel_sensors = self.SenseEnvironment()
			#import pdb; pdb.set_trace()
			# re-synthesizing takes a while, so during this time, wait and do nothing
			# GUY: in the future, perhaps check if the obstacle moved and keep the old
			# synthesis too, so we can continue with the old one
			if(self.InSynthesisProcedure == True):
				# take (no) action!
				#self.control(self.K, self.xinterp, self.uinterp, do_calc=False)
				continue
			
			# if we currently run through an action that is supposed to be 
			# disabled, then quickly stop!!
			if(self.slugs._Nsens > 0):
				# this is basically the fully reactive, so 'F' is assumed otherwise
				# Nsens would not be > 0 (unless it wasn't synthesized correctly and called with wrong parameter)
				if(self.funnel_sensors[self.action] == True):
					print(self.colorize('RED', 'need to do emergency stop R%d (%s), state=%s(%d), mp=%d' %(self.idx, self.list_robots[self.idx], \
												self.map_bit_2_label[self.curr_state], self.curr_state, self.action) ))
					self.do_calc = False
			else:
				# in normal situation, we'd be here if we chose semi-reactive or graph based
				if(self.funnel_sensors[self.action] == True):
					print(self.colorize('RED', 'need to do emergency stop and re-synthesize'))
					self.do_calc = False
					if(self.reactive in ['S', 'G']):
						#import pdb; pdb.set_trace()
						# we need to re-synthesize a new solution because something that is non-compliant to the spec has happened.
						# if we don't do this here, slugs might switch to the new cell and tell the robot to move backwards. once
						# it's there, tell it to move forward and be stuck again.
						#self.curr_state = self.GetClosestNode(self.pose)
						print('in a stopped situation: R%d is now in %s (%d) mp=%d, re-synthesizing ... ' %(\
							  self.idx, self.map_bit_2_label[self.curr_state], self.curr_state, self.action))
						self.ReSynthesizeSpec()
					else:
						pass

			# take control action!
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

			# check if we're in the next funnel (and overflow when in the final one) and if next action
			# is self.STAY_IN_PLACE, keep polling until it's unstuck. if we stopped in the middle, check until you have
			# a different action to take
			if(self.do_calc == False or self.action == self.STAY_IN_PLACE):
				#just a way to reduce the frequency that we call setinitialpos from slugs interface
				if((self.msg_num % (self.MEAS_FS*2)) <= 10):
					# change the funnel in slugs (setpos) to the actual location it is currently at
					# to get a better chance to get a new route
					self.curr_state = self.GetClosestNode(self.pose)
					print('in a stopped situation: R%d is now in %s (%d), checking sensors again ' %(\
						  self.idx, self.map_bit_2_label[self.curr_state], self.curr_state))
					# sensing might be different now because we might be far from first ellipse
					self.funnel_sensors = self.SenseEnvironment()
					# reset the position in slugs
					try:
						self.curr_state, self.action = self.slugs.SetInitialPos(self.curr_state, self.funnel_sensors)
						self.curr_ell = self.FindNextValidEllipse(self.action)
					except:
						import pdb; pdb.set_trace()
					# extension to the telemetry file
					#self.states_fid.write('%d;%s;%d\n' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
					self.logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
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
							print(self.colorize('YELLOW', 'oh no, R%d got \'stay in place\' for next action (%d)' %\
												(self.idx, self.STAY_IN_PLACE)))
					else:
						# it's still in do not calculate new controls
						print(self.colorize('RED', 'R%d is still getting stay in place action ...' %self.idx) )
							
			elif(self.next_action == self.STAY_IN_PLACE):
				if(self.msg_num - self.timer > 1*self.MEAS_FS):
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
				#advance the next goal
				if(self.goals[self.goal] == self.curr_state):
					print(self.colorize('GREEN', 'Passed through a goal!'))
					self.goal += 1
					#reset counter if this is the last one
					if( self.goal >= len(self.goals) ):
						self.goal = 0

				self.funnel_timer = self.msg_num
				#rospy.loginfo
				print('R%d reached funnel %d (%s) from %d (%s) with action %d' \
							  %(self.idx, self.next_state, self.map_bit_2_label[self.next_state], \
								self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
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
				self.logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
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
					print(self.colorize('YELLOW', 'oh no, R%d got \'stay in place\' for next action (%d)' %\
										(self.idx,self.STAY_IN_PLACE)) )

			# GUY TODO, remove when done with debugging
			if((self.msg_num - self.funnel_timer > 15*self.MEAS_FS) and (self.disable_debug == False)):
				print(self.map_bit_2_label[self.next_state])
				print(self.next_action)
				print(self.pose)
				find_non_zero = self.FindNextValidEllipse(self.next_action)
				S, ellipse_pose = self.GetCoordAndEllipseFromLabel(self.map_bit_2_label[self.next_state], \
																   self.next_action, find_non_zero)
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
		#GUY: added support for Nsens=0 too. CHECK
		if(self.slugs._Nsens >= 0):
			# reset the measurements
			#import pdb; pdb.set_trace()
			#funnel_sensors = dict(zip(range(self.slugs._Nout), [False]*self.slugs._Nout)) # Nout is a problem because it's bits
			funnel_sensors = dict(zip(range(self.Nactions), [False]*self.Nactions)) #workaround 
			self.blocking_obs = [[]]*self.Nactions #keep holding the locations of blocking obstacles
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
							self.blocking_obs[mp_i] = adv_pose
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
							self.blocking_obs[mp_i] = adv_pose
							if(self.action == mp_i):
								# do less printing on screen, only if it is blocking us
								print('%s caused funnel %d to be blocked (ellipse %d, R=%d)' %(obs, mp_i, S_i, self.curr_state))
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

	# get the cells that would be blocked by a specific obstacle
	def FindAllIntersectingCells(self, obstacle_region, obstacle_pose):
		tic = timer()
		intersecting_cells = []
		adjacent_cells = []
		
		center_cell = self.map_bit_2_label[obstacle_region]
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', center_cell)] # extract first ellipse pose
 
		# check only cells in the vicinity and not the whole workspace because it makes no sense
		bloat_dim = int(np.ceil(FL_L*2.0)*2)
		for rot in range(4):
			for x in range(np.max([0, xs-bloat_dim]), xs+bloat_dim):
				for y in range(np.max([0, ys-bloat_dim]), ys+bloat_dim):
					adjacent_cells.append('H%dX%dY%d'%(rot,x,y)) 
		
		# bloat the obstacle to account its dimensions
		box_approx_of_robot  = gf.Rectangle(obstacle_pose[:2], FL_L*2.0, FL_L*2.0)
		
		# go over all cells
		mp = 0
		ell  = self.FindNextValidEllipse(mp)
		for cell in adjacent_cells:
			S, ellipse_pose = self.GetCoordAndEllipseFromLabel(cell, mp, ell) #the first ellipse is the "cell"
			S = S[:2,:2]
			ellipse_pose = ellipse_pose[:2]
			e = gf.Ellipse(ellipse_pose, S)
			
			overlaps = gf.TestIntersectionRectangleEllipse(box_approx_of_robot, e)
			if(overlaps):
				# store them as cell numbers
				try:
					intersecting_cells.append(self.map_label_2_bit[cell])
				except:
					# if the cell is out of bounds or on a static obstacle which was removed from the graph
					pass
			else:
				print('%s is not intersecting the obstacle' %(cell))
		
		toc = timer()
		#self._sensing_function_time = toc-tic
		return intersecting_cells
		
	# if we get here it means we found an obstacle and we need to rewrite a spec such that
	# this blocked motion is not allowed
	def ReSynthesizeSpec(self):
		if(self.reactive == 'S'):
			self.InSynthesisProcedure = True
			# send the new order of goals to reach (because we might have completed some goals already)
			'''
			# option a: (not good at all) just remove the motion primitives that were blocked
			# it is not good because we can get blocked standing in a different cell
			current_pose = []
			current_pose.append(self.curr_state)
			self.curr_state = self.GetClosestNode(self.pose)
			current_pose.append(self.curr_state)
			
			blocked_funnels_pre = []
			blocked_funnels_post = []
			for key, bf in self.funnel_sensors.items():
				if(bf):
					blocked_funnels_pre.append(key)
			#get new restricitions
			self.funnel_sensors = self.SenseEnvironment()
			for key, bf in self.funnel_sensors.items():
				if(bf):
					blocked_funnels_post.append(key)
			
			blocked_funnels = []
			blocked_funnels.append(blocked_funnels_pre)
			blocked_funnels.append(blocked_funnels_post)
			'''
			'''
			# option b: (not good at all) block the regions intersecting with the obstacle .
			# if the obstacle wasn't in the last ellipse of the blocked motion primitive, the specification
			# won't do anything with it because it only knows the initial region and final region
			current_pose = self.GetClosestNode(self.pose)
			print('in terms of location, closest region R=%d' %current_pose)
			blocked_funnels = []
			unique_regions = []
			for i, obs_pose in enumerate(self.blocking_obs):
				if (obs_pose != []):
					obstacle_region = self.GetClosestNode(obs_pose)
					# only deal with a new obstacle, don't count same obstacle twice even if it affected two motion primitives
					if(obstacle_region not in unique_regions):
						print('robot in R=%d, obstacle in R=%d: (%.2f,%.2f,%.2f)' % \
							  (current_pose, obstacle_region,obs_pose[0],obs_pose[1],obs_pose[2]))
						unique_regions.append(obstacle_region)
						closeby_regions = self.FindAllIntersectingCells(obstacle_region, obs_pose)
						blocked_funnels.append(closeby_regions)
			goals = []
			for i in range(self.goal, len(self.goals)):
				goals.append(self.goals[i])
			for i in range(0, self.goal):
				goals.append(self.goals[i])

			# use the same spec file, just add in the information that something is blocked
			UpdateRestrictionSlugsInputFile(current_pose, blocked_funnels, goals, self.idx, filename=self.MAP)
			'''
			# option c: (good, but is it best i can do?) create the structuredslugs again with a do not enter zone
			# about the new obstacle
			
			current_pose = self.GetClosestNode(self.pose)
			goals = []
			nxt_goal = self.goal - 1 #it's because the slugsin file already shifts one by itself
			prv_goal = self.goal
			if(nxt_goal < 0):
				nxt_goal = len(self.goals)
				
			for i in range(nxt_goal, len(self.goals)):
				goals.append(self.goals_ic[i])
			for i in range(0, nxt_goal):
				goals.append(self.goals_ic[i])
			no_enter = self.nez[:] # copy the original no enter zones
			# add fictitious no_enter_zones around the problematic obstacles
			unique_regions = []
			for i, obs_pose in enumerate(self.blocking_obs):
				if (obs_pose != []):
					obstacle_region = self.GetClosestNode(obs_pose)
					# only deal with a new obstacle, don't count same obstacle twice even if it affected two motion primitives
					if(obstacle_region not in unique_regions):
						print('robot in R=%d, obstacle in R=%d: (%.2f,%.2f,%.2f)' % \
							  (current_pose, obstacle_region,obs_pose[0],obs_pose[1],obs_pose[2]))
						unique_regions.append(obstacle_region)
						# add the no enter zone, bloat by FL_L
						no_enter.append(box(obs_pose[0]-FL_L, obs_pose[1]-FL_L, obs_pose[0]+FL_L, obs_pose[1]+FL_L))
						
			#import pdb; pdb.set_trace()
			#save the last file for reference:
			copyfile(self.MAP + '_r' + str(self.idx) + '.structuredslugs', \
					 self.MAP + '_r' + str(self.idx) + '_' + str(self.resynth_cnt) + '.structuredslugs')
			self.resynth_cnt += 1
			
			CreateSlugsInputFile(self.G, [goals], self.mps, no_enter, self.total_robots, \
						robot_idx=self.idx, filename=self.MAP, ext_xgrid=self.W_xgrid, ext_ygrid=self.W_ygrid, \
						ext_pix2m=1.0, ext_ic=current_pose, map_label_2_bit=self.map_label_2_bit) 
			#import pdb; pdb.set_trace()
			self.slugs.Shutdown()
			# here we have a new slugsin file, so re-load slugs
			self.slugs = SlugsInterface(self.SYNTH_AUTOMATA_FILE + ('_r%d' %self.idx), simulate=False, slugsLink = self.SLUGS_DIR)
			if(not self.slugs.enabled):
				print('Cannot re-create slugs interface.')
				self.InSynthesisProcedure = False
				return False
			
			print('Successfully re-loaded slugs.')
			self.slugs.DiscoverInputs()
			self.slugs.DiscoverOutputs()
			self.slugs.DiscoverGoals()
			self.slugs.GetInitialPos()
			#import pdb; pdb.set_trace()
			#reset all the variables concerning the new situation
			self.curr_state, self.action = self.slugs.GetNumericState()
			#self.goals = goals[:] # copy using slicing
			self.goal = prv_goal
			self.logger_state.debug('%d;%s;%d' %(self.curr_state, self.map_bit_2_label[self.curr_state], self.action))
			# now, get the new next state
			if(self.action == self.STAY_IN_PLACE):
				print('oh no, R%d starts with action stay in place' %(self.idx))
				self.next_state =  self.curr_state
				self.do_calc = False
			else:
				self.do_calc = True
				for key, val in self.G[self.map_bit_2_label[self.curr_state]].items():
					if( val['motion'] == self.action):
						self.next_state = self.map_label_2_bit[key]
						break
			self.next_state, self.next_action = \
				self.slugs.FindNextStep(self.next_state, self.funnel_sensors)
			print('R%d starts in region %s (%d)' %(self.idx, self.map_bit_2_label[self.curr_state],self.curr_state))
			self.N_ellipse = len(self.mps[self.action]['V'])
			self.curr_ell  = self.FindNextValidEllipse(self.action)
			self.K         = self.mps[self.action]['K'][self.curr_ell]
			self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.action]['xcenter'][self.curr_ell], \
											self.map_bit_2_label[self.curr_state])
			self.x_ref_hires = self.mps[self.action]['xtraj']
			self.u_ref = self.mps[self.action]['unom'][self.curr_ell]
			self.u_ref_hires = self.mps[self.action]['utraj']
			#import pdb; pdb.set_trace()
		elif(self.reactive == 'G'):
			pass
		
		self.InSynthesisProcedure = False
		return True
	
	
	def telemetry(self):
		# telemetry
		now = rospy.get_rostime()
		t = now.secs + now.nsecs/1.0E9
		#rospy.logdebug
		other = []
		for key,val in self.others_pose.items():
			other.append([val[0], val[1]])
		# GUY: I hardcode and assume that there are at least two "other" objects. this is not generic, change in the future
		while(len(other) < 2):
			other.append([-100., -100.])
		try:
			self.logger.debug('%.3f;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;' \
					 '%.2f;%.2f;%.2f;%.2f;%.2f;%.3f;%.2f;%.2f;%.2f;%.2f' \
					   %(t, self.msg_num, self.pose[0], self.pose[1], self.pose[2]*180.0/np.pi, \
						 self.x_ref[0], self.x_ref[1], self.x_ref[2]*180.0/np.pi, self.u_ref[0], self.u_ref[1], \
						self.curr_state, self.curr_ell, int(self.do_calc), self.action, \
						self.u[0], self.u[1], self.linvel.x, self.linvel.y, self.linvel.z,\
						self.rotvel.x, self.rotvel.y, self.rotvel.z, self.xinterp[0],self.xinterp[1],self.xinterp[2]*180.0/np.pi, \
						self.uinterp[0],self.uinterp[1],self._sensing_function_time,other[0][0],other[0][1],other[1][0], \
						 other[1][1]))
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
			ratiov = np.abs(V / self.umax)
			ratiow = np.abs(omega / ( V/self.L *np.tan(self.delmax)))

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
			if(u[1] > self.umax):
				u[1] = self.umax
			if(u[1] < -self.umax):
				u[1] = -self.umax
			if(u[0] > self.delmax):
				u[0] = self.delmax
			if(u[0] < -self.delmax):
				u[0] = -self.delmax

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
		t.angular.z = 0.9*0.5/0.3 * u[1] / self.L * np.tan(u[0]) # delta; 1.0/self.umax is a gain to compensate 1.0/self.umax *
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
	parser.add_argument('--robots', type=str, default='',
					help='id numbers of other robots, in order and comma separated, must have ROS messages (gazebo or on the field)')
	parser.add_argument('--x0', type=str, default='',
					help='initial x position')
	parser.add_argument('--y0', type=str, default='',
					help='initial y position')
	parser.add_argument('--teta0', type=str, default='',
					help='initial z position')
	args = parser.parse_args()
	print('Controller for Jackal%d' %args.i)
	if(args.obs == ''):
		list_obs = []
	else:
		list_obs = args.obs.split(',')
	
	if(args.robots == ''):
		list_robots = []
	else:
		list_robots = args.robots.split(',')
	#import pdb; pdb.set_trace()
	
	pos0 = None
	if(args.x0=='' or args.y0=='' or args.teta0==''):
		pos0 = None
	else:
		pos0 = [float(args.x0), float(args.y0), float(args.teta0)]

	# connect to the relevant master (just because I couldn't make the slaves to connect and run :( )
	if(list_robots == []):
		os.environ['ROS_MASTER_URI'] = 'http://jackal%d:11311/'%args.i
	else:
		os.environ['ROS_MASTER_URI'] = 'http://%s:11311/'%(list_robots[args.i-1] )
	
	rospy.init_node('run_jackal_%d' %args.i)#, log_level=rospy.DEBUG)
	J = Jackal(args.i, args.n, list_obs=list_obs, list_robots=list_robots, first_goal_for_gazebo=pos0, reactive=glob_p.REACTIVE)

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
